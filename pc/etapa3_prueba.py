"""
VibraScan - prueba de la etapa 3 (buffer por bloques y trama binaria con CRC).

Manda "start", captura unos segundos de tramas y dice si la etapa funciona:
tramas integras (CRC), ninguna trama perdida, ninguna muestra descartada en
el micro, ningun hueco, arranque limpio, tasa correcta y 1 g en reposo.

El protocolo esta en vibrascan_proto.py.

Uso:
    python etapa3_prueba.py --list
    python etapa3_prueba.py --port COM8
    python etapa3_prueba.py --port COM8 --seconds 600            (10 minutos)
    python etapa3_prueba.py --port COM8 --odr 100 400 800

Dejar el sensor quieto durante la prueba.
"""

import argparse
import csv
import math
import os
import statistics
import sys
import time

import serial

import vibrascan_proto as vp
from etapa2_prueba import (G_PER_LSB, GRAVITY_TOLERANCE, PERIOD_TOLERANCE, RATE_TOLERANCE,
                           list_ports)

COUNTERS = ("ovr", "i2c_err", "drop", "stall", "bfull", "hueco")
START_TIMEOUT_S = 3.0
CAPTURES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "capturas")


class Capture:
    """Lo que llego entre "start" y el final de la captura."""

    def __init__(self):
        self.dec = vp.Decoder()
        self.chk = vp.Checker()
        self.started = False
        self.first_frames = []    # (tipo, seq, flags) de las primeras tramas tras start
        self.statuses = []
        self.replies = []
        self.blocks = []          # (Block, t0 sin vueltas en us)
        self.events = []
        self.bytes = 0
        self.crc0 = self.skip0 = self.badlen0 = 0
        self.t_first = self.t_last = None   # time.monotonic() del primer y ultimo bloque

    def feed(self, data):
        self.bytes += len(data)
        for fr in self.dec.feed(data):
            text = fr.payload.decode("ascii", errors="replace") if fr.ftype != vp.TYPE_SAMPLES else ""
            if fr.ftype == vp.TYPE_REPLY:
                self.replies.append(text)
                if text == "start ok" and not self.started:
                    # Lo anterior (respuesta a stop, restos de otra sesion) no cuenta
                    self.started = True
                    self.crc0, self.skip0 = self.dec.crc_errors, self.dec.skipped
                    self.badlen0 = self.dec.bad_length
                    self.bytes = 0
            if not self.started:
                continue
            self.events += self.chk.frame(fr)
            if len(self.first_frames) < 3:
                blk = vp.parse_block(fr) if fr.ftype == vp.TYPE_SAMPLES else None
                self.first_frames.append((fr.ftype, fr.seq, blk.flags if blk else None))
            if fr.ftype == vp.TYPE_STATUS:
                self.statuses.append(vp.parse_fields(text))
            elif fr.ftype == vp.TYPE_SAMPLES:
                blk = vp.parse_block(fr)
                if blk is not None:
                    self.blocks.append((blk, self.chk.t_us))
                    self.t_last = time.monotonic()
                    if self.t_first is None:
                        self.t_first = self.t_last

    @property
    def crc_errors(self):
        return self.dec.crc_errors - self.crc0

    @property
    def skipped(self):
        return self.dec.skipped - self.skip0

    @property
    def bad_length(self):
        return self.dec.bad_length - self.badlen0


def capture(port, seconds, odr=None):
    cap = Capture()
    ser = vp.open_stream(port, odr)
    try:
        t_start = t_print = time.monotonic()
        t_end = None
        while True:
            now = time.monotonic()
            if t_end is None and cap.t_first is not None:
                t_end = cap.t_first + seconds
            if t_end is None and now - t_start > START_TIMEOUT_S:
                break                     # no llego ningun bloque
            if t_end is not None and now >= t_end:
                break
            cap.feed(ser.read(ser.in_waiting or 1))
            if t_end is not None and now - t_print >= 1.0:
                t_print = now
                print(f"\r  {now - cap.t_first:5.0f} / {seconds:.0f} s  "
                      f"tramas={cap.dec.frames} muestras={cap.chk.samples}", end="", flush=True)
        print()
    finally:
        ser.write(b"stop\n")
        ser.close()
    return cap


def evaluate(cap):
    """Devuelve (lista de (nombre, ok, detalle), tasa medida en Hz o None)."""
    checks = []
    chk = cap.chk
    ok_statuses = [s for s in cap.statuses if s.get("estado") == "OK"]

    # 1) El firmware respondio y el sensor esta
    if not cap.started:
        checks.append(("Respuesta a start", False,
                       "no llego 'start ok': revisar que el firmware sea el de la etapa 3"))
        return checks, None
    if not cap.statuses:
        checks.append(("Trama de estado recibida", False, "no llego ninguna trama de estado"))
        return checks, None
    last = cap.statuses[-1]
    sensor_ok = last.get("estado") == "OK" and last.get("devid", "").upper() == "0XE5"
    detail = (f"DEVID {last.get('devid')} en la direccion {last.get('addr', '?')}" if sensor_ok else
              f"causa={last.get('causa', '?')} devid={last.get('devid', '?')}"
              " -> revisar cableado SDA/SCL, alimentacion y CS a 3V3")
    checks.append(("Sensor detectado", sensor_ok, detail))
    if not sensor_ok or not cap.blocks:
        if sensor_ok:
            checks.append(("Muestras recibidas", False, "no llego ningun bloque"))
        return checks, None

    # 2) Arranque limpio: respuesta con seq 0, estado y primer bloque marcado
    ff = cap.first_frames
    clean = (len(ff) == 3 and ff[0][:2] == (vp.TYPE_REPLY, 0) and ff[1][0] == vp.TYPE_STATUS
             and ff[2][0] == vp.TYPE_SAMPLES and (ff[2][2] or 0) & vp.FLAG_FIRST)
    checks.append(("Arranque limpio", bool(clean),
                   "start ok (seq 0) -> estado -> primer bloque con marca de inicio" if clean
                   else f"primeras tramas: {ff}"))

    # 3) Integridad: ningun CRC mal ni byte suelto despues de start
    checks.append(("Tramas integras (CRC)",
                   cap.crc_errors == 0 and cap.skipped == 0 and cap.bad_length == 0,
                   f"{cap.dec.frames} tramas, {cap.crc_errors} con CRC mal, "
                   f"{cap.skipped} bytes descartados"))

    # 4) Perdidas, separadas por lugar
    checks.append(("Sin tramas perdidas (USB/PC)", chk.lost_frames == 0,
                   f"{chk.lost_frames} tramas perdidas ({chk.lost_in_transit} muestras)"))
    checks.append(("Sin muestras descartadas en el micro", chk.lost_in_micro == 0,
                   f"{chk.lost_in_micro} muestras (buffer lleno)"))
    checks.append(("Sin huecos de INT1", chk.gaps == 0,
                   f"{chk.gaps} huecos (~{chk.gap_samples} muestras sin flanco)"))

    # 5) Tasa de muestreo con el reloj del micro: muestras entre el t0 del
    #    primer y del ultimo bloque. Las muestras de los huecos se suman (n no
    #    avanza durante un hueco, el tiempo si): asi la tasa mide el reloj del
    #    sensor y el hueco queda solo en su propio chequeo.
    (b0, t0), (b1, t1) = cap.blocks[0], cap.blocks[-1]
    odr = float(b1.odr)
    rate = (b1.n0 - b0.n0 + chk.gap_samples) / ((t1 - t0) / 1e6) if t1 > t0 else None
    if rate:
        checks.append(("Tasa de muestreo", abs(rate - odr) <= RATE_TOLERANCE * odr,
                       f"{rate:.2f} muestras/s medidas con t0, ODR configurado {odr:.0f} Hz, "
                       f"{chk.samples} muestras en {(t1 - t0) / 1e6:.1f} s"))
    else:
        checks.append(("Tasa de muestreo", False, "captura muy corta para medirla"))

    # 5b) INT1 y periodo entre flancos (del firmware). Se descarta la primera
    #     trama de estado: su segundo es anterior a start.
    steady = ok_statuses[1:]
    int1 = [int(s["int1"]) for s in steady if "int1" in s]
    if int1:
        int1_rate = statistics.fmean(int1)
        checks.append(("INT1 conectado", abs(int1_rate - odr) <= RATE_TOLERANCE * odr,
                       f"{int1_rate:.1f} flancos/s en PB0"))
    periods = [(int(s["tmin"]), int(s["tmax"])) for s in steady if int(s.get("tmin", 0)) > 0]
    if periods and rate:
        nominal_us = 1e6 / rate
        tmin, tmax = min(p[0] for p in periods), max(p[1] for p in periods)
        checks.append(("Periodo estable",
                       tmax <= PERIOD_TOLERANCE * nominal_us and tmin >= nominal_us / PERIOD_TOLERANCE,
                       f"entre {tmin} y {tmax} us (medio {nominal_us:.0f} us)"))

    # 6) Contadores del firmware: no deben crecer durante la captura
    if len(ok_statuses) >= 2:
        deltas = {k: int(ok_statuses[-1].get(k, 0)) - int(ok_statuses[0].get(k, 0)) for k in COUNTERS}
        checks.append(("Contadores de error sin cambios", all(v == 0 for v in deltas.values()),
                       " ".join(f"{k}=+{v}" for k, v in deltas.items())))

    # 7) Gravedad en reposo
    samples = [s for blk, _ in cap.blocks for s in blk.samples]
    mean = [statistics.fmean(s[i] for s in samples) * G_PER_LSB for i in range(3)]
    module = math.sqrt(sum(m * m for m in mean))
    noise = [statistics.pstdev(s[i] for s in samples) * G_PER_LSB * 1000 for i in range(3)]
    checks.append(("1 g en reposo", abs(module - 1.0) <= GRAVITY_TOLERANCE,
                   f"|a| = {module:.3f} g  (x={mean[0]:+.3f} y={mean[1]:+.3f} z={mean[2]:+.3f}), "
                   f"ruido rms x/y/z = {noise[0]:.1f}/{noise[1]:.1f}/{noise[2]:.1f} mg"))
    return checks, rate


CSV_COLUMNS = [
    "muestra",      # indice desde el inicio de la captura (un salto = muestra perdida)
    "tiempo_s",     # reloj del micro: t0 del bloque + i x periodo medido
    "ax_g", "ay_g", "az_g",
    "modulo_g",     # sqrt(ax^2 + ay^2 + az^2): ~1 g en reposo
    "ax_cuentas", "ay_cuentas", "az_cuentas",   # valor crudo del sensor, 3.9 mg/cuenta
]


def save_csv(path, cap, period_us):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    n_first, t_first = cap.blocks[0][0].n0, cap.blocks[0][1]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLUMNS)
        for blk, t_us in cap.blocks:
            for i, (x, y, z) in enumerate(blk.samples):
                gx, gy, gz = x * G_PER_LSB, y * G_PER_LSB, z * G_PER_LSB
                t = (t_us - t_first + i * period_us) / 1e6
                w.writerow([blk.n0 + i - n_first, f"{t:.6f}",
                            f"{gx:.4f}", f"{gy:.4f}", f"{gz:.4f}",
                            f"{math.sqrt(gx * gx + gy * gy + gz * gz):.4f}", x, y, z])


def run_one(args, odr_req):
    """Captura y evalua con un ODR. Devuelve True si pasaron todos los chequeos."""
    print()
    print(f"Capturando {args.seconds:.0f} s de {args.port}"
          + (f" con ODR = {odr_req} Hz" if odr_req else "") + ". Dejar el sensor quieto...")
    try:
        cap = capture(args.port, args.seconds, odr_req)
    except serial.SerialException as e:
        print(f"No se pudo leer {args.port}: {e}")
        return False

    for ev in cap.events:
        if ev.kind not in ("inicio",):
            print(f"  evento: {ev.text}")
    checks, rate = evaluate(cap)
    print()
    for name, ok, detail in checks:
        print(f"  [{'OK   ' if ok else 'FALLA'}] {name}: {detail}")
    passed = all(ok for _, ok, _ in checks)
    if cap.t_first is not None and cap.t_last > cap.t_first:
        print(f"  (flujo: {cap.bytes / (cap.t_last - cap.t_first) / 1000:.1f} kB/s)")
    print()
    print("ETAPA 3 APROBADA" if passed else "ETAPA 3 CON FALLAS")

    if cap.blocks:
        odr = cap.blocks[-1][0].odr
        period_us = 1e6 / rate if rate else 1e6 / odr
        path = args.csv or os.path.join(
            CAPTURES_DIR, f"etapa3_{time.strftime('%Y%m%d_%H%M%S')}_{odr}hz.csv")
        save_csv(path, cap, period_us)
        print(f"Datos guardados en {path}")
    return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="Puerto serie, ej: COM8")
    parser.add_argument("--seconds", type=float, default=20.0, help="Duracion de la captura (default: 20)")
    parser.add_argument("--csv", default=None, help="CSV de salida (default: capturas/etapa3_<fecha>.csv)")
    parser.add_argument("--list", action="store_true", help="Lista los puertos serie y termina")
    parser.add_argument("--odr", type=int, nargs="+", default=[None], metavar="HZ",
                        help="ODR a configurar antes de capturar; con varios valores "
                             "prueba cada uno (ej: --odr 100 400 800). Sin esto usa el actual")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return 0
    if not args.port:
        print("Falta --port (usa --list para ver los puertos disponibles).")
        return 2

    results = [run_one(args, odr) for odr in args.odr]
    if len(args.odr) > 1:
        print()
        print("RESULTADO GENERAL: " + ("ETAPA 3 APROBADA" if all(results) else "ETAPA 3 CON FALLAS"))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
