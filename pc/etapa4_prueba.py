"""
VibraScan - prueba de la etapa 4 (captura a archivo y grafico en tiempo).

Analiza capturas hechas con captura.py; no captura. Asi se puede volver a
evaluar (por ejemplo, con otros umbrales) sin repetir la captura.

    1) Reposo estable: 30 minutos quieto a 800 Hz
         python captura.py --port COM8 --minutos 30 --nota "reposo"
         python etapa4_prueba.py reposo capturas/captura_<fecha>

    2) Golpe visible: 5 golpecitos en la mesa, separados unos 2 s, a +/-16 g
         python captura.py --port COM8 --segundos 20 --rango 16 --nota "5 golpes"
         python etapa4_prueba.py golpes capturas/captura_<fecha>
         python ver_captura.py capturas/captura_<fecha>      (mirar los 5 picos)
"""

import argparse
import sys

import numpy as np

import vibrascan_archivo as va
from vibrascan_captura import COUNTERS

# Umbrales propuestos: ajustar con los datos de la primera captura real
REPOSO_MINUTOS = 30
DERIVA_MAX_MG = 10.0        # la media por minuto de cada eje no se mueve mas que esto
RUIDO_TOL = 0.20            # rms de cada minuto dentro de +/-20 % de la mediana
GRAVEDAD_TOL = 0.10         # |a| medio a 1 g +/- 10 %
TASA_TOL = 0.05             # tasa a ODR +/- 5 %
GOLPE_DUR_MAX_MS = 100.0    # tiempo sobre el umbral de cada golpe
GOLPE_VUELTA_MAX_MS = 500.0 # tiempo hasta quedar firme en reposo


def check_perdidas(cap, checks):
    m = cap.meta
    p = m.get("perdidas", {})
    lost = {k: p.get(k, 0) for k in ("tramas_perdidas", "descartadas_micro", "huecos", "crc_mal", "reconexiones")}
    checks.append(("Sin perdidas", all(v == 0 for v in lost.values()) and len(cap.tramos()) == 1,
                   " ".join(f"{k}={v}" for k, v in lost.items()) + f", {len(cap.tramos())} segmento(s)"))
    ini, fin = (m.get("contadores_firmware", {}).get(k) or {} for k in ("inicio", "fin"))
    deltas = {k: int(fin.get(k) or 0) - int(ini.get(k) or 0) for k in COUNTERS if k in ini and k in fin}
    checks.append(("Contadores del firmware sin cambios", bool(deltas) and all(v == 0 for v in deltas.values()),
                   " ".join(f"{k}=+{v}" for k, v in deltas.items()) or "sin datos"))


def check_saturacion(cap, checks, consejo):
    sat = va.saturadas(cap)
    n_sat = int(sat.sum()) if sat is not None else 0
    checks.append(("Sin saturacion", sat is not None and n_sat == 0,
                   f"rango +/-{cap.meta.get('rango_g')} g, {n_sat} muestras en el tope"
                   + (f" -> {consejo}" if n_sat else "") if sat is not None else "rango desconocido"))


def reposo(cap, minutos):
    checks = []
    m = cap.meta
    dur = float(cap.t[-1] - cap.t[0]) if len(cap.t) else 0.0
    checks.append(("Captura completa", bool(m.get("completa")) and dur >= minutos * 60 * 0.98,
                   f"{dur / 60:.1f} min de {minutos} pedidos" + ("" if m.get("completa") else ", SIN CERRAR")))
    check_perdidas(cap, checks)
    check_saturacion(cap, checks, "algo golpeo el sensor durante el reposo")

    ws = va.por_ventana(cap)
    if len(ws) < 2:
        checks.append(("Estadisticas por minuto", False, "captura muy corta"))
        return checks
    means = np.array([w["media_g"] for w in ws]) * 1000      # mg, (minutos, 3)
    rms = np.array([w["rms_g"] for w in ws]) * 1000
    drift = means.max(axis=0) - means.min(axis=0)
    checks.append(("Deriva de la media", bool((drift <= DERIVA_MAX_MG).all()),
                   "x/y/z = " + "/".join(f"{d:.1f}" for d in drift) + f" mg (max {DERIVA_MAX_MG:.0f} mg) "
                   f"en {len(ws)} min"))
    ref = np.median(rms, axis=0)
    bad = [(w["ventana"], "xyz"[a], rms[k, a]) for k, w in enumerate(ws) for a in range(3)
           if abs(rms[k, a] - ref[a]) > RUIDO_TOL * ref[a]]
    checks.append(("Ruido estable", not bad,
                   "rms x/y/z = " + "/".join(f"{r:.1f}" for r in ref) + f" mg; minutos fuera de +/-{RUIDO_TOL:.0%}: "
                   + (", ".join(f"min {k} {a}={r:.1f}" for k, a, r in bad[:8]) or "ninguno")))
    mod = float(np.mean([w["modulo_g"] for w in ws]))
    checks.append(("1 g en reposo", abs(mod - 1) <= GRAVEDAD_TOL, f"|a| medio = {mod:.3f} g"))

    period, tmin, tmax = m.get("periodo_us"), *(m.get("periodo_firmware_us") or [None, None])
    odr = m.get("odr_hz") or 0
    ok = (period is not None and tmin is not None and tmin - 1 <= period <= tmax + 1
          and abs(1e6 / period - odr) <= TASA_TOL * odr)
    checks.append(("Base de tiempo", ok,
                   f"periodo por regresion {period} us, firmware {tmin}-{tmax} us, "
                   f"tasa {1e6 / period:.2f} Hz para odr={odr}" if period else "sin periodo"))
    return checks


def golpes(cap, esperados):
    checks = []
    check_perdidas(cap, checks)
    check_saturacion(cap, checks, "repetir con --rango 16")
    gs = va.golpes(cap)
    checks.append(("Cantidad de golpes", len(gs) == esperados, f"{len(gs)} detectados, {esperados} esperados"))
    if gs:
        durs = [g.dur_ms for g in gs]
        checks.append(("Golpes cortos", max(durs) <= GOLPE_DUR_MAX_MS,
                       "sobre el umbral: " + ", ".join(f"{d:.0f}" for d in durs) + f" ms (max {GOLPE_DUR_MAX_MS:.0f})"))
        vueltas = [g.vuelta_ms for g in gs]
        ok = all(v is not None and v <= GOLPE_VUELTA_MAX_MS for v in vueltas)
        checks.append(("Vuelve al reposo", ok, "en " + ", ".join(f"{v:.0f}" if v is not None else "nunca"
                                                                   for v in vueltas)
                       + f" ms (max {GOLPE_VUELTA_MAX_MS:.0f})"))
        seps = np.diff([g.t_s for g in gs])
        print("  golpes: " + ", ".join(f"t={g.t_s:.2f} s pico={g.pico_g:.2f} g" for g in gs))
        if len(seps):
            print("  separaciones: " + ", ".join(f"{s:.2f}" for s in seps) + " s")
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prueba", choices=("reposo", "golpes"))
    parser.add_argument("captura", help="Captura (.csv, .json o nombre base)")
    parser.add_argument("--minutos", type=float, default=REPOSO_MINUTOS, help="Duracion pedida para reposo")
    parser.add_argument("--golpes", type=int, default=5, help="Golpes esperados")
    args = parser.parse_args()

    cap = va.cargar(args.captura)
    m = cap.meta
    print(f"{args.prueba.upper()}: {cap.base}  ({m.get('inicio')}, nota: {m.get('nota') or '-'}, "
          f"odr={m.get('odr_hz')} Hz, rango=+/-{m.get('rango_g')} g)")
    checks = reposo(cap, args.minutos) if args.prueba == "reposo" else golpes(cap, args.golpes)
    print()
    for name, ok, detail in checks:
        print(f"  [{'OK   ' if ok else 'FALLA'}] {name}: {detail}")
    passed = all(ok for _, ok, _ in checks)
    print()
    print(f"ETAPA 4 - {args.prueba.upper()}: " + ("APROBADO" if passed else "CON FALLAS"))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
