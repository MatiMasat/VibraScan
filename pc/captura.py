"""
VibraScan - captura a archivo, sin ventana (etapa 4).

Graba lo que manda la Black Pill en pc/capturas/captura_<fecha>.csv y .json
(formato en vibrascan_captura.py). Si el USB se desconecta, sigue intentando
y retoma sola en un segmento nuevo.

Uso:
    python captura.py --port COM8 --minutos 30 --nota "reposo sobre la mesa"
    python captura.py --port COM8 --segundos 20 --nota "5 golpes"
    python captura.py --port COM8 --minutos 5 --odr 400 --rango 16

Ctrl+C termina antes de tiempo (lo grabado queda bien cerrado).
Despues: python ver_captura.py capturas/captura_<fecha>
"""

import argparse
import os
import sys
import time

import serial

from vibrascan_captura import Captura

CAPTURES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "capturas")
PROGRESS_S = 5.0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Puerto serie, ej: COM8")
    dur = parser.add_mutually_exclusive_group(required=True)
    dur.add_argument("--minutos", type=float, help="Duracion en minutos")
    dur.add_argument("--segundos", type=float, help="Duracion en segundos")
    parser.add_argument("--nota", default="", help="Texto libre que queda en el JSON")
    parser.add_argument("--odr", type=int, choices=(100, 200, 400, 800), default=None)
    parser.add_argument("--rango", type=int, choices=(2, 4, 8, 16), default=None, help="Rango en g")
    parser.add_argument("--salida", default=None, help="Nombre base de los archivos (sin extension)")
    args = parser.parse_args()

    seconds = args.minutos * 60 if args.minutos is not None else args.segundos
    base = args.salida or os.path.join(CAPTURES_DIR, f"captura_{time.strftime('%Y%m%d_%H%M%S')}")
    try:
        cap = Captura(args.port, base, args.odr, args.rango, args.nota)
    except serial.SerialException as e:
        print(f"No se pudo abrir {args.port}: {e}")
        return 2

    print(f"Capturando {seconds:.0f} s en {base}.csv (Ctrl+C para terminar antes)")
    t_print = time.monotonic()
    try:
        while cap.t_first_block is None or cap.t_now() < seconds:
            _, events = cap.poll()
            for kind, text in events:
                if kind not in ("respuesta", "ovr"):
                    print(f"  [{cap.t_now():7.1f} s] {kind}: {text}")
            if cap.t_first_block is None and time.monotonic() - cap.t_pc0 > 5:
                print("No llega ningun bloque: revisar que el firmware sea el de la etapa 3 o posterior.")
                break
            if time.monotonic() - t_print >= PROGRESS_S:
                t_print = time.monotonic()
                t = cap.totals()
                lost = t["tramas_perdidas"] + t["descartadas_micro"] + t["huecos"] + t["crc_mal"]
                print(f"  {cap.t_now():6.0f} / {seconds:.0f} s  muestras={cap.samples} "
                      f"segmentos={cap.seg + 1} perdidas={lost} reconexiones={t['reconexiones']}"
                      + ("  (sin puerto, reintentando)" if cap.ser is None else ""))
    except KeyboardInterrupt:
        print("  interrumpida")
    finally:
        cap.close()

    m = cap.meta()
    p = m["perdidas"]
    print()
    print(f"Captura: {m['duracion_s']:.1f} s, {m['muestras']} muestras, {len(m['segmentos'])} segmento(s), "
          f"odr={m['odr_hz']} Hz, rango={m['rango_g']} g, periodo={m['periodo_us']} us")
    print(f"Perdidas: tramas={p['tramas_perdidas']} micro={p['descartadas_micro']} huecos={p['huecos']} "
          f"crc={p['crc_mal']} reconexiones={p['reconexiones']}")
    print(f"Archivos: {base}.csv y {base}.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
