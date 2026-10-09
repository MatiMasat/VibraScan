"""
VibraScan - prueba de la etapa 7 (filtro adaptativo NLMS en el micro).

Analiza capturas hechas con captura.py con el filtro activo (por defecto lo
esta). Dos pruebas:

    1) Equivalencia y tiempo real: lo que calculo el micro (float32) contra
       la referencia de Python (float64, vibrascan_filtro.py) sobre las mismas
       muestras crudas, reiniciando en los mismos puntos que el micro; y el
       CPU que uso el filtro.
         python captura.py --port COM8 --segundos 30 --odr 3200 --nota "equivalencia"
         python etapa7_prueba.py equivalencia capturas/captura_<fecha>

    2) Comportamiento con un tono (frente a distintos niveles de ruido, como
       pide el plan de la beca): mejora de la relacion senal/ruido del tono en
       la salida y, cuanto tono queda en el residuo e y cuanto tarda en
       converger.
         python captura.py --port COM8 --segundos 30 --odr 3200 --nota "tono 250 Hz volumen 50 %"
         python etapa7_prueba.py tono capturas/captura_<fecha> --frecuencia 250

Donde se ejecuta cada parte: ver la tabla en docs/filtro_adaptativo.md.
"""

import argparse
import sys

import numpy as np

import vibrascan_archivo as va
import vibrascan_espectro as ve  # noqa: F401 (ve.NPERSEG)
import vibrascan_filtro as vf
from etapa5_prueba import encendido

# Umbrales
DIF_MAX_G = 1e-4          # micro (float32) vs Python (float64): 0,1 mg, 39 veces menos que 1 cuenta
CPU_MAX = 0.10            # el filtro no puede usar mas del 10 % del CPU
MEJORA_MIN_DB = 6.0       # el tono tiene que salir al menos 6 dB mas limpio en y
SUPRESION_MIN_DB = 10.0   # y quedar al menos 10 dB mas bajo en el residuo e
CONVERGE_MAX_S = 2.0
ESPERA_S = 3.0            # se analiza despues de este tiempo (ya convergio)
G_PER_LSB_F32 = np.float32(0.0039)


def runs(cap):
    """Tramos de la salida del filtro entre reinicios: [(fila_inicio, fila_fin)]."""
    r = np.flatnonzero(cap.filtro["reinicio"])
    if len(r) == 0 or r[0] != 0:
        r = np.concatenate(([0], r))
    return list(zip(r.tolist(), np.concatenate((r[1:], [len(cap.filtro["n"])])).tolist()))


def entrada(cap, eje, n):
    """Muestras crudas del eje para los indices n, en g con la misma conversion
    float32 que el micro. Devuelve (x, ok); ok = False si falta alguna muestra."""
    idx = np.searchsorted(cap.n, n)
    ok = bool(len(idx)) and idx[-1] < len(cap.n) and np.array_equal(cap.n[np.minimum(idx, len(cap.n) - 1)], n)
    if not ok:
        return None, False
    return (cap.c[idx, eje].astype(np.float32) * G_PER_LSB_F32).astype(np.float64), True


def config(cap):
    cfg = cap.meta.get("filtro") or {}
    eje = "xyz".index(cfg.get("eje", "z"))
    return cfg, eje, int(cfg.get("taps", vf.TAPS)), int(cfg.get("delta", vf.DELTA)), int(cfg.get("mu", 10)) / 1000.0


def equivalencia(cap):
    checks = []
    cfg, eje, taps, delta, mu = config(cap)
    p = cap.meta.get("perdidas", {})
    if cap.filtro is None or cfg.get("filtro") != "nlms":
        checks.append(("Salida del filtro", False, "la captura no tiene <base>_filtro.csv con NLMS activo"))
        return checks
    checks.append(("Sin perdidas", p.get("tramas_perdidas", 0) == 0 and p.get("descartadas_micro", 0) == 0
                   and p.get("crc_mal", 0) == 0,
                   " ".join(f"{k}={p.get(k)}" for k in ("tramas_perdidas", "descartadas_micro", "crc_mal"))))

    fs_nominal = float(cap.meta.get("odr_hz") or 800)
    dif_y = dif_e = 0.0
    muestras = tramos = salteados = 0
    for i0, i1 in runs(cap):
        n = cap.filtro["n"][i0:i1]
        x, ok = entrada(cap, eje, n)
        if not ok or np.any(np.diff(n) != 1):
            salteados += 1                   # faltan muestras crudas: no se puede reproducir
            continue
        y, e = vf.Filtro(vf.NLMS, fs_nominal, taps, delta, mu).procesar_serie(x)
        dif_y = max(dif_y, float(np.abs(cap.filtro["y"][i0:i1] - y).max()))
        dif_e = max(dif_e, float(np.abs(cap.filtro["e"][i0:i1] - e).max()))
        muestras += i1 - i0
        tramos += 1
    checks.append(("Mismo resultado que Python", tramos > 0 and max(dif_y, dif_e) < DIF_MAX_G,
                   f"{muestras} muestras en {tramos} tramo(s): diferencia maxima y {dif_y * 1e6:.3f} ug, "
                   f"e {dif_e * 1e6:.3f} ug (max {DIF_MAX_G * 1e6:.0f} ug)"
                   + (f"; {salteados} tramo(s) sin datos crudos completos" if salteados else "")))

    cpu = cap.meta["filtro"].get("cpu_us_por_s_max", 0) / 1e6
    checks.append(("Tiempo real", 0 < cpu <= CPU_MAX,
                   f"el filtro uso hasta {cpu * 100:.2f} % del CPU (max {CPU_MAX * 100:.0f} %) a "
                   f"{fs_nominal:.0f} Hz, {taps} coeficientes"))
    return checks


def tono(cap, f_tono):
    checks = []
    cfg, eje, taps, delta, mu = config(cap)
    if cap.filtro is None:
        checks.append(("Salida del filtro", False, "la captura no tiene <base>_filtro.csv"))
        return checks
    fs = 1e6 / cap.meta["periodo_us"] if cap.meta.get("periodo_us") else float(cap.meta.get("odr_hz"))
    # El tono se prende despues del silencio inicial: se mide desde ahi (si no,
    # el silencio contaria como tiempo de convergencia)
    t_on = encendido(cap, f_tono)
    if t_on is None:
        checks.append(("Tono presente", False, f"no aparece en {f_tono:g} Hz en el espectrograma"))
        return checks
    n_on = cap.n[min(int(np.searchsorted(cap.t, t_on)), len(cap.n) - 1)]
    i0, i1 = next(((a, b) for a, b in runs(cap) if cap.filtro["n"][a] <= n_on < cap.filtro["n"][b - 1]),
                  max(runs(cap), key=lambda r: r[1] - r[0]))
    j_on = i0 + int(np.searchsorted(cap.filtro["n"][i0:i1], n_on))
    print(f"  el tono se prende en t = {t_on:.1f} s")
    k = j_on + int(ESPERA_S * fs)
    if k >= i1 - 4 * ve.NPERSEG:
        checks.append(("Duracion", False, "captura muy corta"))
        return checks
    n = cap.filtro["n"][k:i1]
    x, ok = entrada(cap, eje, n)
    y, e = cap.filtro["y"][k:i1], cap.filtro["e"][k:i1]
    x = x - x.mean()
    # SNR en toda la banda: la medida que mejora un ALE (ver vf.snr_banda_db)
    sx, ax = vf.snr_banda_db(x, fs, f_tono)
    sy, ay = vf.snr_banda_db(y, fs, f_tono)
    _, ae = vf.snr_banda_db(e, fs, f_tono)
    print(f"  tono {f_tono:g} Hz en el eje {'xyz'[eje]}: entrada {ax * 1000:.2f} mg, "
          f"y {ay * 1000:.2f} mg, e {ae * 1000:.2f} mg (rms)")
    checks.append(("Mejora de SNR en y (toda la banda)", sy - sx >= MEJORA_MIN_DB,
                   f"{sx:.1f} dB en la entrada -> {sy:.1f} dB en y ({sy - sx:+.1f} dB, min +{MEJORA_MIN_DB:.0f})"))
    sup = 20 * np.log10(ax / ae) if ae else float("inf")
    checks.append(("Tono fuera del residuo e", sup >= SUPRESION_MIN_DB,
                   f"el tono queda {sup:.1f} dB mas bajo en e que en la entrada (min {SUPRESION_MIN_DB:.0f})"))

    # Convergencia: amplitud de y en ventanas de 0,1 s desde el inicio del tramo
    yy = cap.filtro["y"][j_on:i1]          # desde que se prende el tono
    w = max(1, int(0.1 * fs))
    amp = np.array([np.sqrt(np.mean(yy[j:j + w] ** 2)) for j in range(0, len(yy) - w, w)])
    final = np.median(amp[int(ESPERA_S / 0.1):])
    t90 = next((j * 0.1 for j, a in enumerate(amp) if a >= 0.9 * final), None)
    checks.append(("Convergencia", t90 is not None and t90 <= CONVERGE_MAX_S,
                   f"y llega al 90 % de su amplitud final {t90:.2f} s despues de prenderse el tono "
                   f"(max {CONVERGE_MAX_S:.0f} s)"
                   if t90 is not None else "no converge"))
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prueba", choices=("equivalencia", "tono"))
    parser.add_argument("captura", help="Captura (.csv, .json o nombre base)")
    parser.add_argument("--frecuencia", type=float, default=None, help="Frecuencia del tono (prueba tono)")
    args = parser.parse_args()

    cap = va.cargar(args.captura)
    cfg = cap.meta.get("filtro") or {}
    print(f"{args.prueba.upper()}: {cap.base}  (nota: {cap.meta.get('nota') or '-'}, odr={cap.meta.get('odr_hz')} Hz, "
          f"filtro={cfg.get('filtro')} eje={cfg.get('eje')} taps={cfg.get('taps')} delta={cfg.get('delta')} "
          f"mu={cfg.get('mu')}/1000)")
    if args.prueba == "tono":
        if args.frecuencia is None:
            parser.error("la prueba tono necesita --frecuencia")
        checks = tono(cap, args.frecuencia)
    else:
        checks = equivalencia(cap)
    print()
    for name, ok, detail in checks:
        print(f"  [{'OK   ' if ok else 'FALLA'}] {name}: {detail}")
    passed = all(ok for _, ok, _ in checks)
    print()
    print(f"ETAPA 7 - {args.prueba.upper()}: " + ("APROBADO" if passed else "CON FALLAS"))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
