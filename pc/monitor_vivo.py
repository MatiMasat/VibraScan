"""
VibraScan - visor en tiempo real de los tres ejes.

Grafica lo que manda la Black Pill por el puerto COM virtual, para ver como
responde el sensor al moverlo. Ademas registra la sesion en pc/capturas/:
    monitor_<fecha>.log   eventos (tramas perdidas, descartes, huecos, CRC,
                          cortes, golpes, caras, contadores), una linea por
                          segundo y un resumen al cerrar
    monitor_<fecha>.csv   la captura, en el formato de captura.py (seg,n,t_s,x,y,z)
    monitor_<fecha>.json  y sus metadatos: se abre con ver_captura.py
    monitor_<fecha>_N.png imagen del grafico: con la tecla "g" y al cerrar

Si el USB se desconecta, el monitor sigue intentando y retoma solo.

Uso:
    python monitor_vivo.py --port COM8
    python monitor_vivo.py --port COM8 --window 5 --ylim 4
    python monitor_vivo.py --port COM8 --odr 400      (cambia el ODR del sensor)
    python monitor_vivo.py --port COM8 --rango 16     (cambia el rango: 2, 4, 8 o 16 g)
    python monitor_vivo.py --port COM8 --no-csv       (no guarda las muestras)

Cerrar la ventana para terminar. El puerto no puede estar abierto por otro
programa (monitor serie, etapa3_prueba.py) al mismo tiempo.

Usa el protocolo binario de la etapa 3 (vibrascan_proto.py): al abrir manda
"start" y al cerrar "stop".
"""

import argparse
import collections
import math
import os
import sys
import threading
import re
import time
import traceback

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

import vibrascan_archivo as va
import vibrascan_proto as vp
from vibrascan_captura import COUNTERS, G_PER_LSB, Captura

RATE_TOLERANCE = 0.05   # sps puede diferir 5 % del ODR nominal
EVENT_LABELS = {"seq": "TRAMA PERDIDA", "n": "DESCARTE", "hueco": "HUECO", "crc": "CRC",
                "puerto_perdido": "PUERTO PERDIDO", "reconexion": "RECONEXION", "odr": "ODR",
                "rango": "RANGO", "estado": "ESTADO NO OK", "inicio": "inicio",
                "respuesta": "respuesta del firmware", "ovr": "OVERRUN"}

AXES = (("X", "tab:blue"), ("Y", "tab:orange"), ("Z", "tab:green"))
FS_MAX = 3200           # el buffer alcanza para la ventana completa hasta este ODR
CAPTURES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "capturas")

FREEZE_S = 0.5          # sin muestras durante este tiempo = corte de datos
                        # (o 3 bloques, si es mas: a 100 Hz un bloque tarda 320 ms)
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
    """Lee con el motor de captura en segundo plano (graba, se reconecta), guarda
    las ultimas muestras para el grafico y registra la sesion."""

    def __init__(self, port, maxlen, log, base, default_fs, odr=None, rango=None):
        super().__init__(daemon=True)
        self.cap = Captura(port, base, odr, rango, nota="monitor_vivo")
        self.log = log
        self.fs = default_fs
        self.samples = collections.deque(maxlen=maxlen)   # (n, x_g, y_g, z_g)
        self.status = {}
        self.lock = threading.Lock()
        self.running = True
        self.plot_span = 0.0     # segundos que ocupa la curva (lo escribe el grafico)
        self.losses = ""         # lo calcula este hilo; el grafico solo lo lee

        # Estadisticas para el log y el resumen
        self.total = 0
        self.last_n = None
        self.last_sample_t = None
        self.frozen_since = None
        self.freezes = []          # duracion de cada corte [s]
        self.holes = []            # (t, ms) de cada hueco
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
        self._sec = []             # muestras desde la ultima trama de estado

    def run(self):
        while self.running:
            try:
                frames, events = self.cap.poll()
            except Exception:
                self.log.write("ERROR leyendo:\n" + traceback.format_exc())
                time.sleep(0.5)
                continue
            for kind, text in events:
                self._on_event(kind, text)
            self._check_freeze()
            for frame in frames:
                self._on_frame(frame)
            self.losses = self._losses()

    def _on_event(self, kind, text):
        if kind == "hueco":
            m = re.search(r"hueco de ([\d.]+) ms", text)
            self.holes.append((self.log.now(), float(m.group(1)) if m else 0.0))
        self.log.write(f"{EVENT_LABELS.get(kind, kind.upper())}: {text}", echo=kind != "ovr")

    def _on_frame(self, frame):
        if frame.ftype == vp.TYPE_SAMPLES:
            blk = vp.parse_block(frame)
            if blk is None:
                return
            self.fs = float(blk.odr) or self.fs
            for i, (x, y, z) in enumerate(blk.samples):
                self._on_sample(blk.n0 + i, x, y, z)
        elif frame.ftype == vp.TYPE_STATUS:
            self._on_status(frame.payload.decode("ascii", errors="replace"))

    def _check_freeze(self):
        if self.last_sample_t is None or self.frozen_since is not None:
            return
        limit = max(FREEZE_S, 3 * vp.BLOCK_SAMPLES / self.fs)
        if time.monotonic() - self.last_sample_t > limit:
            self.frozen_since = self.last_sample_t
            self.log.write(f"CORTE: sin muestras hace {limit:.1f} s (ultima n={self.last_n})")

    def _on_sample(self, n, x, y, z):
        now = time.monotonic()
        if self.frozen_since is not None:
            dur = now - self.frozen_since
            self.freezes.append(dur)
            self.log.write(f"DATOS DE NUEVO tras {dur:.2f} s sin muestras (n={n})")
            self.frozen_since = None
        self.last_sample_t = now

        gx, gy, gz = x * G_PER_LSB, y * G_PER_LSB, z * G_PER_LSB
        with self.lock:
            if self.last_n is not None and n < self.last_n:
                self.samples.clear()     # n retrocedio: la placa se reinicio (reconexion)
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
        status = vp.parse_fields(line)
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
                + f" perdidas={self._losses()} curva={span:.1f} s")
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

    def _losses(self):
        t = self.cap.totals()
        return (f"tramas:{t['tramas_perdidas']} micro:{t['descartadas_micro']} huecos:{t['huecos']} "
                f"crc:{t['crc_mal']} reconexiones:{t['reconexiones']}")

    def snapshot(self):
        with self.lock:
            return list(self.samples), dict(self.status), self.losses

    def close(self):
        self.running = False
        self.join(timeout=1.0)
        self.cap.close()       # manda stop y cierra los archivos (tolera un puerto perdido)
        if self.frozen_since is not None:
            self.freezes.append(time.monotonic() - self.frozen_since)


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
    c = r.cap.totals()
    log.raw(f"  tramas: {c['tramas']}, con CRC mal: {c['crc_mal']}, "
            f"bytes descartados: {c['bytes_descartados']}")
    log.raw(f"  tramas perdidas (USB/PC): {c['tramas_perdidas']} ({c['muestras_en_tramas_perdidas']} muestras)")
    log.raw(f"  muestras descartadas en el micro: {c['descartadas_micro']}")
    log.raw(f"  reconexiones del USB: {c['reconexiones']}")
    log.raw(f"  huecos de INT1: {c['huecos']} (~{c['muestras_en_huecos']} muestras)"
            + (": " + ", ".join(f"{ms / 1000:.1f} s" for _, ms in r.holes) if r.holes else ""))
    log.raw(f"  cortes de datos: {len(r.freezes)}"
            + (f" (el mas largo {max(r.freezes):.2f} s)" if r.freezes else ""))
    if r.first_counters is not None:
        log.raw("  contadores inicio -> fin: " + ", ".join(
            f"{k} {r.first_counters.get(k, '?')}->{r.last_counters.get(k, '?')}" for k in COUNTERS))
    log.raw(f"  tramas de estado no OK: {r.statuses_bad}")
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
    check("3 Golpe", len(r.shocks) >= 1, f"{len(r.shocks)} golpe(s) detectado(s)")
    recovered = bool(r.holes) and r.frozen_since is None
    check("4 Robustez INT1", recovered,
          f"{len(r.holes)} hueco(s) informado(s) por el firmware"
          + (" (" + ", ".join(f"{ms / 1000:.1f} s" for _, ms in r.holes) + ")" if r.holes else "")
          + f", {len(r.freezes)} corte(s) del grafico, "
          + ("se recupero" if r.frozen_since is None else "SIGUE CORTADO al cerrar"))
    lost = c["tramas_perdidas"] + c["descartadas_micro"] + c["crc_mal"]
    check("5 Sin perdidas", lost == 0,
          f"tramas perdidas={c['tramas_perdidas']}, descartadas en el micro={c['descartadas_micro']}, "
          f"CRC mal={c['crc_mal']} (los huecos de la prueba 4 no cuentan"
          + (f"; hubo {c['reconexiones']} reconexion(es) del USB" if c["reconexiones"] else "") + ")")
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
                        help="Cambia el ODR del sensor al arrancar: 100, 200, 400, 800, 1600 o 3200")
    parser.add_argument("--rango", type=int, choices=(2, 4, 8, 16), default=None,
                        help="Cambia el rango del sensor al arrancar, en g")
    parser.add_argument("--no-csv", action="store_true", help="No guarda las muestras crudas")
    args = parser.parse_args()

    os.makedirs(CAPTURES_DIR, exist_ok=True)
    base = os.path.join(CAPTURES_DIR, f"monitor_{time.strftime('%Y%m%d_%H%M%S')}")
    log = SessionLog(base + ".log")
    log.write(f"monitor_vivo en {args.port}, ventana {args.window:.0f} s"
              + (f", odr pedido {args.odr} Hz" if args.odr else ""))

    try:
        reader = Reader(args.port, int(args.window * FS_MAX), log,
                        None if args.no_csv else base, args.fs, args.odr, args.rango)
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
    fig.tight_layout(rect=(0, 0, 1, 0.9))   # lugar para las 3 lineas del titulo

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
        samples, status, losses = reader.snapshot()
        if not samples:
            return
        fs = float(status.get("odr", 0) or 0) or args.fs
        n_last = samples[-1][0]
        # Solo la ventana visible. Si hay mas muestras que pixeles, cada pixel
        # muestra el minimo y el maximo de sus muestras: ningun pico se pierde
        arr = np.array([s for s in samples if 0 <= n_last - s[0] <= args.window * fs])
        t = (arr[:, 0] - n_last) / fs
        with reader.lock:
            reader.plot_span = -t[0]
        width = max(200, int(axs[0].bbox.width))
        for i, ln in enumerate(lines):
            ln.set_data(*va.envolvente(t, arr[:, i + 1], width))
        x, y, z = samples[-1][1:]
        for lbl, v in zip(labels, (x, y, z)):
            lbl.set_text(f"{v:+.3f} g")
        mod = math.sqrt(x * x + y * y + z * z)
        title.set_text(f"VibraScan - {args.port} - |a| = {mod:.3f} g - "
                       f"odr={status.get('odr', '?')} sps={status.get('sps', '?')} "
                       f"periodo={status.get('tmin', '?')}-{status.get('tmax', '?')} us\n"
                       f"estado={status.get('estado', '?')} drop={status.get('drop', '?')} "
                       f"stall={status.get('stall', '?')} rango={status.get('rango', '?')} g\n"
                       f"perdidas {losses}" + ("  (SIN PUERTO, reintentando)" if reader.cap.ser is None else ""))

    fig.canvas.mpl_connect("close_event", lambda _e: save_png("al cerrar"))
    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841
    plt.show()
    reader.close()
    try:
        summarize(reader, log, args.window)
    except Exception:
        log.raw("ERROR armando el resumen:\n" + traceback.format_exc())
    log.raw(f"Registro: {base}.log" + ("" if args.no_csv else
            f"\nCaptura: {base}.csv/.json  ->  python ver_captura.py {base}"))
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
