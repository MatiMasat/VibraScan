"""
VibraScan - visor en tiempo real de los tres ejes.

Grafica lo que manda la Black Pill por el puerto COM virtual, para ver como
responde el sensor al moverlo. Ademas registra la sesion en pc/capturas/:
    monitor_<fecha>.log   eventos (saltos, cortes, golpes, caras, contadores),
                          una linea por segundo y un resumen al cerrar
    monitor_<fecha>.csv   todas las muestras crudas (t_pc_s, n, x, y, z)
    monitor_<fecha>_N.png imagen del grafico: con la tecla "g" y al cerrar

Uso:
    python monitor_vivo.py --port COM8
    python monitor_vivo.py --port COM8 --window 5 --ylim 4
    python monitor_vivo.py --port COM8 --odr 400      (cambia el ODR del sensor)
    python monitor_vivo.py --port COM8 --no-csv       (no guarda las muestras)

Cerrar la ventana para terminar. El puerto no puede estar abierto por otro
programa (monitor serie, etapa2_prueba.py) al mismo tiempo.
"""

import argparse
import collections
import math
import os
import sys
import threading
import time

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

from etapa2_prueba import COUNTERS, G_PER_LSB, RATE_TOLERANCE, parse_sample, parse_status

AXES = (("X", "tab:blue"), ("Y", "tab:orange"), ("Z", "tab:green"))
FS_MAX = 3200           # el buffer alcanza para la ventana completa hasta este ODR
MAX_PLOT_POINTS = 4000  # por eje; con mas, se dibuja 1 de cada k muestras
CAPTURES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "capturas")

FREEZE_S = 0.5          # sin muestras durante este tiempo = corte de datos
SHOCK_G = 0.5           # |a| se aparta de 1 g mas que esto = golpe
SHOCK_END_S = 0.25      # el golpe termina tras este tiempo por debajo del umbral
FACE_TOLERANCE_G = 0.15 # cara: un eje a 1 g +/- esto y los otros dos por debajo
STILL_G = 0.10          # quieto: |a| varia menos que esto en un segundo


class SessionLog:
    """Escribe los eventos con el tiempo desde el inicio y los repite por consola."""

    def __init__(self, path):
        self.t0 = time.monotonic()
        self.file = open(path, "w", encoding="utf-8")
        self.lock = threading.Lock()

    def now(self):
        return time.monotonic() - self.t0

    def write(self, text, echo=True):
        line = f"[{self.now():7.1f} s] {text}"
        with self.lock:
            self.file.write(line + "\n")
            self.file.flush()
        if echo:
            print(line)

    def raw(self, text):
        with self.lock:
            self.file.write(text + "\n")
            self.file.flush()
        print(text)

    def close(self):
        self.file.close()


def face_of(x, y, z):
    """Devuelve 'Z+', 'X-', etc. si un eje esta en +/-1 g y los otros cerca de 0."""
    values = {"X": x, "Y": y, "Z": z}
    axis = max(values, key=lambda k: abs(values[k]))
    if abs(abs(values[axis]) - 1.0) > FACE_TOLERANCE_G:
        return None
    if any(abs(v) > 0.3 for k, v in values.items() if k != axis):
        return None
    return axis + ("+" if values[axis] > 0 else "-")


class Reader(threading.Thread):
    """Lee el puerto en segundo plano, guarda las ultimas muestras y registra la sesion."""

    def __init__(self, port, maxlen, log, csv_path, default_fs, odr=None):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.odr = odr
        self.log = log
        self.csv = open(csv_path, "w", encoding="ascii") if csv_path else None
        if self.csv:
            self.csv.write("t_pc_s,n,x,y,z\n")
        self.fs = default_fs
        self.samples = collections.deque(maxlen=maxlen)   # (n, x_g, y_g, z_g)
        self.status = {}
        self.gaps = 0
        self.lock = threading.Lock()
        self.running = True
        self.plot_span = 0.0     # segundos que ocupa la curva (lo escribe el grafico)

        # Estadisticas para el log y el resumen
        self.total = 0
        self.last_n = None
        self.last_sample_t = None
        self.frozen_since = None
        self.freezes = []          # duracion de cada corte [s]
        self.first_counters = None
        self.last_counters = {}
        self.sps_seen = []
        self.tmin_seen = []
        self.tmax_seen = []
        self.statuses_bad = 0
        self.shock = None          # golpe en curso: [inicio, pico, muestras_debajo]
        self.shocks = []           # (t, pico_g, duracion_ms)
        self.faces = []            # caras en el orden en que aparecieron
        self.face = None
        self.spans = []            # (t, segundos visibles)
        self._sec = []             # muestras desde la ultima linea de estado

    def run(self):
        pending = b""
        self.ser.reset_input_buffer()
        if self.odr:
            self.ser.write(f"odr={self.odr}\n".encode("ascii"))
        first_line = True
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                if self.running:
                    self.log.write(f"ERROR: se perdio el puerto ({e})")
                break
            self._check_freeze()
            if not chunk:
                continue
            pending += chunk
            *lines, pending = pending.split(b"\n")
            for raw in lines:
                line = raw.decode("ascii", errors="replace").strip()
                if first_line:
                    first_line = False   # puede venir cortada
                    continue
                if line.startswith(">"):
                    self.log.write(f"respuesta del firmware: {line}")
                    continue
                if line.startswith("#"):
                    self._on_status(line)
                    continue
                sample = parse_sample(line)
                if sample is not None:
                    self._on_sample(*sample)

    def _check_freeze(self):
        if self.last_sample_t is None or self.frozen_since is not None:
            return
        if time.monotonic() - self.last_sample_t > FREEZE_S:
            self.frozen_since = self.last_sample_t
            self.log.write(f"CORTE: sin muestras hace {FREEZE_S} s (ultima n={self.last_n})")

    def _on_sample(self, n, x, y, z):
        now = time.monotonic()
        if self.frozen_since is not None:
            dur = now - self.frozen_since
            self.freezes.append(dur)
            self.log.write(f"DATOS DE NUEVO tras {dur:.2f} s sin muestras (n={n})")
            self.frozen_since = None
        self.last_sample_t = now
        if self.csv:
            self.csv.write(f"{self.log.now():.4f},{n},{x},{y},{z}\n")

        gx, gy, gz = x * G_PER_LSB, y * G_PER_LSB, z * G_PER_LSB
        with self.lock:
            if self.last_n is not None and n != self.last_n + 1:
                if not self.status:
                    # Al abrir el puerto llegan primero muestras viejas que
                    # quedaron en el USB de la sesion anterior: no es un salto
                    self.log.write(f"inicio: se descartan muestras viejas (n {self.last_n} -> {n})")
                    self.samples.clear()
                elif n == 0:
                    self.log.write("n volvio a 0: el firmware se reinicio")
                else:
                    self.gaps += 1
                    self.log.write(f"SALTO: n paso de {self.last_n} a {n} "
                                   f"({n - self.last_n - 1} muestras perdidas)")
            self.last_n = n
            self.samples.append((n, gx, gy, gz))
        self.total += 1
        self._sec.append((gx, gy, gz))
        self._track_shock(math.sqrt(gx * gx + gy * gy + gz * gz))

    def _track_shock(self, mod):
        over = abs(mod - 1.0) > SHOCK_G
        if self.shock is None:
            if over:
                self.shock = [self.total, mod, 0]
            return
        if over:
            self.shock[2] = 0
            if abs(mod - 1.0) > abs(self.shock[1] - 1.0):
                self.shock[1] = mod
            return
        self.shock[2] += 1
        if self.shock[2] >= SHOCK_END_S * self.fs:
            start, peak, below = self.shock
            dur_ms = (self.total - start - below) / self.fs * 1000
            self.shocks.append((self.log.now(), peak, dur_ms))
            self.log.write(f"GOLPE: pico |a| = {peak:.2f} g, {dur_ms:.0f} ms sobre el umbral")
            self.shock = None

    def _on_status(self, line):
        status = parse_status(line)
        with self.lock:
            self.status = status
            span = self.plot_span
        try:
            self.fs = float(status.get("odr", 0)) or self.fs
        except ValueError:
            pass
        counters = {k: int(status[k]) for k in COUNTERS if status.get(k, "").isdigit()}
        if self.first_counters is None:
            self.first_counters = dict(counters)
        for k, v in counters.items():
            prev = self.last_counters.get(k)
            if prev is not None and v != prev:
                self.log.write(f"CONTADOR: {k} paso de {prev} a {v}")
        self.last_counters = counters
        if status.get("estado") != "OK":
            self.statuses_bad += 1
            self.log.write(f"ESTADO NO OK: {line}")
        for key, dest in (("sps", self.sps_seen), ("tmin", self.tmin_seen), ("tmax", self.tmax_seen)):
            if status.get(key, "").isdigit():
                dest.append(int(status[key]))
        self.spans.append((self.log.now(), span))

        sec, self._sec = self._sec, []
        text = (f"odr={status.get('odr', '?')} sps={status.get('sps', '?')} "
                f"periodo={status.get('tmin', '?')}-{status.get('tmax', '?')} us "
                + " ".join(f"{k}={status.get(k, '?')}" for k in COUNTERS)
                + f" saltos={self.gaps} curva={span:.1f} s")
        if sec:
            mx = sum(s[0] for s in sec) / len(sec)
            my = sum(s[1] for s in sec) / len(sec)
            mz = sum(s[2] for s in sec) / len(sec)
            mods = [math.sqrt(a * a + b * b + c * c) for a, b, c in sec]
            text += (f" | {len(sec)} muestras, media x/y/z = {mx:+.2f}/{my:+.2f}/{mz:+.2f} g, "
                     f"|a| {min(mods):.2f}-{max(mods):.2f} g")
            face = face_of(mx, my, mz) if max(mods) - min(mods) < STILL_G else None
            if face and face != self.face:
                self.face = face
                if face not in self.faces:
                    self.faces.append(face)
                self.log.write(f"CARA: eje {face[0]} en {face[1]}1 g "
                               f"(x={mx:+.2f} y={my:+.2f} z={mz:+.2f})")
        else:
            text += " | sin muestras en este segundo"
        self.log.write(text, echo=False)

    def snapshot(self):
        with self.lock:
            return list(self.samples), dict(self.status), self.gaps

    def close(self):
        self.running = False
        self.join(timeout=1.0)
        self.ser.close()
        if self.frozen_since is not None:
            self.freezes.append(time.monotonic() - self.frozen_since)
        if self.csv:
            self.csv.close()


def summarize(reader, log, window):
    """Resumen al cerrar y chequeo de lo que se puede medir de las 5 pruebas manuales."""
    r = reader
    dur = log.now()
    log.raw("")
    log.raw("=" * 70)
    log.raw(f"RESUMEN - {dur:.1f} s de sesion, {r.total} muestras recibidas")
    odr = r.fs
    if r.sps_seen:
        log.raw(f"  sps: {min(r.sps_seen)}-{max(r.sps_seen)} (odr={odr:.0f})")
    if r.tmin_seen:
        log.raw(f"  periodo entre interrupciones: {min(r.tmin_seen)}-{max(r.tmax_seen)} us")
    log.raw(f"  saltos de n: {r.gaps}")
    log.raw(f"  cortes de datos: {len(r.freezes)}"
            + (f" (el mas largo {max(r.freezes):.2f} s)" if r.freezes else ""))
    if r.first_counters is not None:
        log.raw("  contadores inicio -> fin: " + ", ".join(
            f"{k} {r.first_counters.get(k, '?')}->{r.last_counters.get(k, '?')}" for k in COUNTERS))
    log.raw(f"  lineas de estado no OK: {r.statuses_bad}")
    log.raw(f"  golpes: {len(r.shocks)}" + (
        f" (pico maximo {max(abs(s[1] - 1) for s in r.shocks) + 1:.2f} g)" if r.shocks else ""))
    log.raw(f"  caras vistas: {', '.join(r.faces) if r.faces else 'ninguna'}")

    # Curva: se mira despues de que pasaron 'window' segundos con datos
    spans = [s for t, s in r.spans if t > window + 2]
    log.raw("")
    log.raw("CHEQUEO AUTOMATICO (complementa lo que viste en el grafico)")

    def check(name, ok, detail):
        log.raw(f"  [{'OK' if ok else 'REVISAR'}] {name}: {detail}")

    if spans:
        check("1 Reposo - curva", min(spans) >= window * 0.95,
              f"la curva ocupo {min(spans):.1f}-{max(spans):.1f} s de {window:.0f} s")
    else:
        check("1 Reposo - curva", False, f"la sesion duro menos de {window + 2:.0f} s")
    if r.sps_seen:
        # El reloj del sensor da ~795 sps para 800 Hz nominal
        steady = sorted(r.sps_seen)[len(r.sps_seen) // 2]
        check("1 Reposo - sps", abs(steady - odr) <= odr * RATE_TOLERANCE,
              f"mediana {steady} sps para odr={odr:.0f}")
    check("2 Caras", len(r.faces) >= 2,
          f"{len(r.faces)} de 6 ({', '.join(r.faces) or '-'})")
    check("3 Golpe", len(r.shocks) >= 1,
          f"{len(r.shocks)} golpe(s) detectado(s)" + (
              ", con saltos en la sesion" if r.gaps else ", sin saltos"))
    stall0 = (r.first_counters or {}).get("stall")
    stall1 = r.last_counters.get("stall")
    recovered = bool(r.freezes) and r.frozen_since is None
    check("4 Robustez INT1", recovered and stall0 is not None and stall1 is not None and stall1 > stall0,
          f"{len(r.freezes)} corte(s), stall {stall0}->{stall1}")
    check("5 Saltos", r.gaps == 0, f"saltos={r.gaps}")
    log.raw("=" * 70)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Puerto serie, ej: COM8")
    parser.add_argument("--window", type=float, default=10.0, help="Segundos visibles (default: 10)")
    parser.add_argument("--ylim", type=float, default=2.0, help="Escala vertical en g, +/- (default: 2)")
    parser.add_argument("--fs", type=float, default=800.0,
                        help="Tasa de muestreo hasta recibir la primera linea de estado (default: 800)")
    parser.add_argument("--odr", type=int, default=None, metavar="HZ",
                        help="Cambia el ODR del sensor al arrancar: 100, 200, 400 u 800")
    parser.add_argument("--no-csv", action="store_true", help="No guarda las muestras crudas")
    args = parser.parse_args()

    os.makedirs(CAPTURES_DIR, exist_ok=True)
    base = os.path.join(CAPTURES_DIR, f"monitor_{time.strftime('%Y%m%d_%H%M%S')}")
    log = SessionLog(base + ".log")
    log.write(f"monitor_vivo en {args.port}, ventana {args.window:.0f} s"
              + (f", odr pedido {args.odr} Hz" if args.odr else ""))

    try:
        reader = Reader(args.port, int(args.window * FS_MAX), log,
                        None if args.no_csv else base + ".csv", args.fs, args.odr)
    except serial.SerialException as e:
        log.write(f"No se pudo abrir {args.port}: {e}")
        log.close()
        return 2
    reader.start()

    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    lines, labels = [], []
    for ax, (name, color) in zip(axs, AXES):
        (ln,) = ax.plot([], [], color=color, linewidth=1.0)
        lines.append(ln)
        ax.set_ylabel(f"a{name.lower()} [g]")
        ax.set_ylim(-args.ylim, args.ylim)
        ax.grid(True, alpha=0.3)
        labels.append(ax.text(0.01, 0.92, "", transform=ax.transAxes, va="top",
                              family="monospace", fontsize=10))
    axs[-1].set_xlim(-args.window, 0)
    axs[-1].set_xlabel('Tiempo [s] (0 = ahora)   -   tecla "g": guardar imagen')
    title = fig.suptitle(f"VibraScan - {args.port} - esperando datos...")
    fig.tight_layout()

    shots = [0]

    def save_png(reason):
        shots[0] += 1
        path = f"{base}_{shots[0]}.png"
        try:
            fig.savefig(path, dpi=100)
        except Exception as e:   # al cerrar, algunos backends ya no pueden dibujar
            log.write(f"no se pudo guardar la imagen ({reason}): {e}")
            return
        log.write(f"imagen {os.path.basename(path)} guardada ({reason})")

    def on_key(event):
        if event.key == "g":
            save_png("tecla g")

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        samples, status, gaps = reader.snapshot()
        if not samples:
            return
        fs = float(status.get("odr", 0) or 0) or args.fs
        n_last = samples[-1][0]
        # Solo la ventana visible; si son muchas muestras se dibuja 1 de cada k
        visible = [s for s in samples if (n_last - s[0]) <= args.window * fs]
        step = max(1, math.ceil(len(visible) / MAX_PLOT_POINTS))
        shown = visible[::step]
        t = [(s[0] - n_last) / fs for s in shown]
        with reader.lock:
            reader.plot_span = -t[0]
        for i, ln in enumerate(lines):
            ln.set_data(t, [s[i + 1] for s in shown])
        x, y, z = samples[-1][1:]
        for lbl, v in zip(labels, (x, y, z)):
            lbl.set_text(f"{v:+.3f} g")
        mod = math.sqrt(x * x + y * y + z * z)
        title.set_text(f"VibraScan - {args.port} - |a| = {mod:.3f} g - "
                       f"odr={status.get('odr', '?')} sps={status.get('sps', '?')} "
                       f"periodo={status.get('tmin', '?')}-{status.get('tmax', '?')} us\n"
                       f"estado={status.get('estado', '?')} drop={status.get('drop', '?')} "
                       f"stall={status.get('stall', '?')} saltos={gaps}"
                       + (f" (dibujando 1 de cada {step})" if step > 1 else ""))

    fig.canvas.mpl_connect("close_event", lambda _e: save_png("al cerrar"))
    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841
    plt.show()
    reader.close()
    summarize(reader, log, args.window)
    log.raw(f"Registro: {base}.log")
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
