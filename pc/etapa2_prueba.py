"""
VibraScan - prueba de la etapa 2.

Captura unos segundos de lo que manda la Black Pill por el puerto COM virtual
y dice si la etapa funciona: sensor detectado, tasa de muestreo correcta,
ninguna muestra perdida y 1 g medido en reposo.

Formato que manda el firmware (ver Core/Src/app.c):
    n,x,y,z                     una linea por muestra (cuentas crudas)
    # estado=OK clave=valor ... una linea de estado por segundo

Uso:
    python etapa2_prueba.py --list
    python etapa2_prueba.py --port COM5
    python etapa2_prueba.py --port COM5 --seconds 20 --plot
    python etapa2_prueba.py --port COM5 --odr 100 400 800

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
import serial.tools.list_ports

G_PER_LSB = 0.0039        # FULL_RES: 3.9 mg/LSB en cualquier rango
RATE_TOLERANCE = 0.05     # la tasa medida puede diferir 5 % del ODR nominal
GRAVITY_TOLERANCE = 0.10  # el modulo en reposo puede diferir 10 % de 1 g
PERIOD_TOLERANCE = 1.5    # periodo entre flancos: entre 1/1,5 y 1,5 veces el medio
SETTLE_S = 2.5            # espera despues de cambiar el ODR (leyendo y descartando)
PURGE_S = 1.0             # descarte inicial sin cambio de ODR
COUNTERS = ("ovr", "i2c_err", "drop", "stall", "bfull")


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No se detectaron puertos serie.")
    for p in ports:
        print(f"  {p.device}  -  {p.description}")


def parse_status(line):
    """'# estado=OK odr=100 ...' -> {'estado': 'OK', 'odr': '100', ...}"""
    fields = {}
    for token in line[1:].split():
        key, sep, value = token.partition("=")
        if sep:
            fields[key] = value
    return fields


def parse_sample(line):
    """'n,x,y,z' -> (n, x, y, z) o None si la linea no es una muestra valida."""
    parts = line.split(",")
    if len(parts) != 4:
        return None
    try:
        n, x, y, z = (int(p) for p in parts)
    except ValueError:
        return None
    return n, x, y, z


def set_odr(ser, odr):
    """Manda 'odr=<Hz>' y espera la respuesta '> ...'. Devuelve el texto de la respuesta."""
    ser.reset_input_buffer()
    ser.write(f"odr={odr}\n".encode("ascii"))
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line.startswith(">"):
            return line
    return "> sin respuesta del firmware"


def capture(port, seconds, odr=None):
    """Lee el puerto durante 'seconds' y devuelve (muestras, estados, lineas_invalidas).
    Si se pasa odr, antes cambia el ODR del sensor y espera a que se estabilice."""
    samples, statuses, bad = [], [], 0
    pending = b""
    with serial.Serial(port, 115200, timeout=0.1) as ser:
        if odr is not None:
            print(f"  {set_odr(ser, odr)}")
        # Mientras nadie lee el puerto, el buffer de la placa se llena y guarda
        # muestras viejas. Se lee y descarta un rato (sin dormir: si no se lee,
        # Windows deja de pedir datos y el buffer se vuelve a llenar) para que
        # la captura empiece con datos frescos. Despues de cambiar el ODR se
        # espera mas: el primer segundo despues del cambio es mixto.
        t_settle = time.monotonic() + (SETTLE_S if odr is not None else PURGE_S)
        while time.monotonic() < t_settle:
            ser.read(ser.in_waiting or 1)
        ser.reset_input_buffer()
        first_line = True
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            pending += chunk
            *lines, pending = pending.split(b"\n")
            for raw in lines:
                line = raw.decode("ascii", errors="replace").strip()
                if first_line:
                    first_line = False   # puede venir cortada: se descarta
                    continue
                if not line or line.startswith(">"):
                    continue
                if line.startswith("#"):
                    statuses.append(parse_status(line))
                    continue
                sample = parse_sample(line)
                if sample is None:
                    bad += 1
                else:
                    samples.append(sample)
    return samples, statuses, bad


def evaluate(samples, statuses, bad):
    """Devuelve (lista de (nombre, ok, detalle), tasa medida en Hz o None)."""
    checks = []
    ok_statuses = [s for s in statuses if s.get("estado") == "OK"]

    # 1) Sensor detectado
    if not statuses:
        checks.append(("Linea de estado recibida", False,
                       "no llego ninguna linea '#': revisar que el firmware sea el de la etapa 2"))
        return checks, None
    last = statuses[-1]
    sensor_ok = last.get("estado") == "OK" and last.get("devid", "").upper() == "0XE5"
    if sensor_ok:
        detail = f"DEVID {last['devid']} en la direccion {last.get('addr', '?')}"
    else:
        detail = (f"causa={last.get('causa', '?')} devid={last.get('devid', '?')}"
                  " -> revisar cableado SDA/SCL, alimentacion y CS a 3V3")
    checks.append(("Sensor detectado", sensor_ok, detail))
    if not sensor_ok or not samples:
        if sensor_ok:
            checks.append(("Muestras recibidas", False, "no llego ninguna muestra"))
        return checks, None

    # 2) Tasa de muestreo, medida con el reloj del micro: cuanto avanza el
    #    contador n entre lineas de estado, que salen una por segundo.
    rate = None
    odr = float(last.get("odr", "0") or 0)
    if len(ok_statuses) >= 2:
        n0, n1 = int(ok_statuses[0]["n"]), int(ok_statuses[-1]["n"])
        rate = (n1 - n0) / (len(ok_statuses) - 1)
        rate_ok = odr > 0 and abs(rate - odr) <= RATE_TOLERANCE * odr
        checks.append(("Tasa de muestreo", rate_ok,
                       f"{rate:.2f} muestras/s medidas, ODR configurado {odr:.0f} Hz"))
    else:
        checks.append(("Tasa de muestreo", False, "captura muy corta para medirla"))

    # 2b) INT1 -> PB0: flancos de DATA_READY por segundo, contados por el micro.
    #     Se descarta la primera linea (su segundo puede estar incompleto).
    int1 = [int(s["int1"]) for s in ok_statuses[1:] if "int1" in s]
    if int1:
        int1_rate = statistics.fmean(int1)
        int1_ok = odr > 0 and abs(int1_rate - odr) <= RATE_TOLERANCE * odr
        detail = f"{int1_rate:.1f} flancos/s en PB0, ODR configurado {odr:.0f} Hz"
        if int1_rate == 0:
            detail += " -> revisar el cable INT1 -> PB0"
        checks.append(("INT1 conectado", int1_ok, detail))
    else:
        checks.append(("INT1 conectado", False, "el firmware no informa int1 o la captura es muy corta"))

    # 2c) Periodo entre flancos de INT1, medido por el micro (tmin/tmax en us
    #     por segundo). Un flanco perdido hace que tmax llegue a ~2 periodos.
    periods = [(int(s["tmin"]), int(s["tmax"])) for s in ok_statuses[1:]
               if "tmin" in s and int(s.get("tmin", 0)) > 0]
    if periods and rate:
        nominal_us = 1e6 / rate
        tmin = min(p[0] for p in periods)
        tmax = max(p[1] for p in periods)
        period_ok = tmax <= PERIOD_TOLERANCE * nominal_us and tmin >= nominal_us / PERIOD_TOLERANCE
        checks.append(("Periodo estable", period_ok,
                       f"entre {tmin} y {tmax} us (medio {nominal_us:.0f} us), "
                       f"variacion {tmax - tmin} us"))
    elif "tmin" in last:
        checks.append(("Periodo estable", False, "captura muy corta para medirlo"))

    # 3) Secuencia sin saltos
    gaps = sum(1 for a, b in zip(samples, samples[1:]) if b[0] != a[0] + 1)
    missing = (samples[-1][0] - samples[0][0] + 1) - len(samples)
    checks.append(("Secuencia sin saltos", gaps == 0 and bad == 0,
                   f"{len(samples)} muestras, {missing} faltantes en {gaps} saltos, "
                   f"{bad} lineas invalidas"))

    # 4) Contadores de error del firmware: no deben crecer durante la captura
    if len(ok_statuses) >= 2:
        deltas = {k: int(ok_statuses[-1].get(k, 0)) - int(ok_statuses[0].get(k, 0))
                  for k in COUNTERS}
        checks.append(("Contadores de error en 0", all(v == 0 for v in deltas.values()),
                       " ".join(f"{k}=+{v}" for k, v in deltas.items())))

    # 5) Gravedad en reposo
    mean = [statistics.fmean(s[i] for s in samples) * G_PER_LSB for i in (1, 2, 3)]
    module = math.sqrt(sum(m * m for m in mean))
    noise = [statistics.pstdev(s[i] for s in samples) * G_PER_LSB * 1000 for i in (1, 2, 3)]
    checks.append(("1 g en reposo", abs(module - 1.0) <= GRAVITY_TOLERANCE,
                   f"|a| = {module:.3f} g  (x={mean[0]:+.3f} y={mean[1]:+.3f} z={mean[2]:+.3f}), "
                   f"ruido rms x/y/z = {noise[0]:.1f}/{noise[1]:.1f}/{noise[2]:.1f} mg"))
    return checks, rate


CSV_COLUMNS = [
    "muestra",      # indice desde el inicio de la captura (un salto = muestra perdida)
    "tiempo_s",     # muestra / fs, con fs medida
    "ax_g", "ay_g", "az_g",
    "modulo_g",     # sqrt(ax^2 + ay^2 + az^2): ~1 g en reposo
    "ax_cuentas", "ay_cuentas", "az_cuentas",   # valor crudo del sensor, 3.9 mg/cuenta
]


def save_csv(path, samples, fs):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    n0 = samples[0][0]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLUMNS)
        for n, x, y, z in samples:
            gx, gy, gz = x * G_PER_LSB, y * G_PER_LSB, z * G_PER_LSB
            w.writerow([n - n0, f"{(n - n0) / fs:.4f}",
                        f"{gx:.4f}", f"{gy:.4f}", f"{gz:.4f}",
                        f"{math.sqrt(gx * gx + gy * gy + gz * gz):.4f}",
                        x, y, z])


def plot(samples, fs):
    import matplotlib.pyplot as plt

    n0 = samples[0][0]
    t = [(s[0] - n0) / fs for s in samples]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for i, name in ((1, "X"), (2, "Y"), (3, "Z")):
        ax.plot(t, [s[i] * G_PER_LSB for s in samples], label=name, linewidth=0.8)
    ax.set_xlabel("Tiempo [s]")
    ax.set_ylabel("Aceleracion [g]")
    ax.set_title(f"VibraScan etapa 2 - fs = {fs:.0f} Hz")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="Puerto serie, ej: COM5")
    parser.add_argument("--seconds", type=float, default=10.0, help="Duracion de la captura (default: 10)")
    parser.add_argument("--csv", default=None, help="CSV de salida (default: capturas/etapa2_<fecha>.csv)")
    parser.add_argument("--plot", action="store_true", help="Grafica la captura al terminar")
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

    all_passed = True
    for odr_req in args.odr:
        passed = run_one(args, odr_req)
        all_passed = all_passed and passed
    if len(args.odr) > 1:
        print()
        print("RESULTADO GENERAL: " + ("ETAPA 2 APROBADA" if all_passed else "ETAPA 2 CON FALLAS"))
    return 0 if all_passed else 1


def run_one(args, odr_req):
    """Captura y evalua con un ODR. Devuelve True si pasaron todos los chequeos."""
    print()
    print(f"Capturando {args.seconds:.0f} s de {args.port}"
          + (f" con ODR = {odr_req} Hz" if odr_req else "") + ". Dejar el sensor quieto...")
    try:
        samples, statuses, bad = capture(args.port, args.seconds, odr_req)
    except serial.SerialException as e:
        print(f"No se pudo leer {args.port}: {e}")
        return False

    checks, rate = evaluate(samples, statuses, bad)
    print()
    for name, ok, detail in checks:
        print(f"  [{'OK   ' if ok else 'FALLA'}] {name}: {detail}")
    passed = all(ok for _, ok, _ in checks)
    print()
    print("ETAPA 2 APROBADA" if passed else "ETAPA 2 CON FALLAS")

    if samples:
        # El reloj del sensor no es exacto (ej. 99,3 Hz para 100 Hz nominal):
        # para el eje de tiempo se usa la tasa medida si la hay.
        odr = float(statuses[-1].get("odr", "0") or 0) if statuses else 0.0
        fs = rate or odr or 100.0
        suffix = f"_{odr:.0f}hz" if odr else ""
        path = args.csv or os.path.join(
            "capturas", f"etapa2_{time.strftime('%Y%m%d_%H%M%S')}{suffix}.csv")
        save_csv(path, samples, fs)
        print(f"Datos guardados en {path}")
        if args.plot:
            plot(samples, fs)
    return passed


if __name__ == "__main__":
    sys.exit(main())
