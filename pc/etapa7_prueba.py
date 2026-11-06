"""
VibraScan - prueba de la etapa 7 (filtro adaptativo NLMS en el micro).

Analiza capturas hechas con captura.py con el filtro activo (por defecto lo
esta). Tres pruebas:

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

       El segundo en que se prende el tono se busca solo en el espectrograma.
       Con un tono muy debil (cerca del piso de ruido) el detector no lo
       encuentra: se indica a mano con --inicio (mirarlo con
       espectro.py --espectrograma, o contar los segundos de silencio):
         python etapa7_prueba.py tono capturas/captura_<fecha> --frecuencia 250 --inicio 5

    3) Motor (sensor pegado a la carcasa, ver docs/prueba_motor.md): la linea
       electrica aparece solo con el motor prendido y la velocidad del rotor se
       puede seguir en el tiempo con las bandas laterales. La captura tiene que
       empezar con unos segundos de motor apagado. Deja <base>_motor.png.
         python captura.py --port COM8 --segundos 90 --odr 3200 --rango 16 --nota "motor"
         python etapa7_prueba.py motor capturas/captura_<fecha> --rpm-nominal 1350

       Variando la velocidad por escalones (se chequea que aparezcan todos, y no
       que este por encima de la nominal). Con variador de frecuencia, la linea
       se mueve con la velocidad: agregar --variador.
         python etapa7_prueba.py motor capturas/captura_<fecha> --rpm-nominal 1350 --escalones 4
         python etapa7_prueba.py motor capturas/captura_<fecha> --rpm-nominal 1350 --escalones 4 --variador

Donde se ejecuta cada parte: ver la tabla en docs/filtro_adaptativo.md.
"""

import argparse
import os
import sys

import numpy as np

import vibrascan_archivo as va
import vibrascan_espectro as ve  # noqa: F401 (ve.NPERSEG)
import vibrascan_filtro as vf
import vibrascan_motor as vm
from etapa5_prueba import encendido

# Umbrales
DIF_MAX_G = 1e-4          # micro (float32) vs Python (float64): 0,1 mg, 39 veces menos que 1 cuenta
CPU_MAX = 0.10            # el filtro no puede usar mas del 10 % del CPU
MEJORA_MIN_DB = 6.0       # el tono tiene que salir al menos 6 dB mas limpio en y
GANANCIA_TOL_DB = 1.5     # ganancia del tono en y: la de la teoria del ALE +/- esto
CONVERGE_MAX_S = 2.0
ESPERA_S = 3.0            # se analiza despues de este tiempo (ya convergio)
MARCHA_MIN_S = 10.0       # motor: tiempo prendido, en total
REPOSO_MIN_S = 3.0        # motor: tiempo apagado, en el tramo mas largo
SATURADAS_MAX = 0.001     # motor: fraccion de muestras saturadas con el motor en marcha
LINEA_ERROR_HZ = 0.2      # motor: la linea a +/- esto de 2 x red (la red varia ~+/-0,05 Hz)
LINEA_DB = 20.0           # motor: la linea sobre el piso, y prendido contra apagado
MEDIDAS_MIN = 0.5         # motor: fraccion de ventanas con la velocidad medida
NOMINAL_TOL = 0.02        # motor: tolerancia de la velocidad nominal (dato de un modelo similar)
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


def tono(cap, f_tono, inicio=None):
    checks = []
    cfg, eje, taps, delta, mu = config(cap)
    if cap.filtro is None:
        checks.append(("Salida del filtro", False, "la captura no tiene <base>_filtro.csv"))
        return checks
    fs = 1e6 / cap.meta["periodo_us"] if cap.meta.get("periodo_us") else float(cap.meta.get("odr_hz"))
    # El tono se prende despues del silencio inicial: se mide desde ahi (si no,
    # el silencio contaria como tiempo de convergencia)
    t_on = encendido(cap, f_tono) if inicio is None else inicio
    if t_on is None:
        checks.append(("Tono presente", False, f"no aparece en {f_tono:g} Hz en el espectrograma "
                       "(si es muy debil, indicar el encendido con --inicio)"))
        return checks
    n_on = cap.n[min(int(np.searchsorted(cap.t, t_on)), len(cap.n) - 1)]
    i0, i1 = next(((a, b) for a, b in runs(cap) if cap.filtro["n"][a] <= n_on < cap.filtro["n"][b - 1]),
                  max(runs(cap), key=lambda r: r[1] - r[0]))
    j_on = i0 + int(np.searchsorted(cap.filtro["n"][i0:i1], n_on))
    print(f"  el tono se prende en t = {t_on:.1f} s" + (" (indicado con --inicio)" if inicio is not None else ""))
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
    # Ganancia del tono: con poca senal el ALE no recupera todo el tono (parte
    # queda en e). Se compara con la solucion de Wiener (vf.ganancia_ale).
    g_teo = vf.ganancia_ale(sx, taps)
    g_med = ay / ax
    dif = 20 * np.log10(g_med / g_teo)
    checks.append(("Ganancia del tono como la teoria", abs(dif) <= GANANCIA_TOL_DB,
                   f"medida {20 * np.log10(g_med):+.1f} dB, teoria {20 * np.log10(g_teo):+.1f} dB "
                   f"(G = Mr/2 / (1 + Mr/2), M={taps}, r={10 ** (sx / 10):.3f}); diferencia {dif:+.1f} dB "
                   f"(max {GANANCIA_TOL_DB})"))
    print("  tono en el residuo e: " + (f"{20 * np.log10(ax / ae):.1f} dB por debajo de la entrada "
                                       f"(teoria {-20 * np.log10(1 - g_teo):.1f} dB)" if ae > 0
                                       else "por debajo del piso de ruido"))

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


def _mas_largo(tramos):
    return max(tramos, key=lambda tr: tr[1] - tr[0]) if tramos else None


def _dur(tr):
    return tr[1] - tr[0] if tr else 0.0


def motor(cap, red, rpm_nominal=None, polos=None, variador=False, n_escalones=None):
    """Prueba con el motor. Devuelve (chequeos, seguimiento, escalones, polos)."""
    checks = []
    if polos is None and rpm_nominal:
        polos = vm.polos_de_nominal(rpm_nominal, red)
    if polos is None:
        print("  AVISO: sin --rpm-nominal ni --polos la velocidad se busca hasta f_L / 2 y puede")
        print("         confundirse con otra familia de lineas (por ejemplo, la de 50 Hz).")
    p = cap.meta.get("perdidas", {})
    checks.append(("Sin perdidas", p.get("tramas_perdidas", 0) == 0 and p.get("descartadas_micro", 0) == 0
                   and p.get("crc_mal", 0) == 0,
                   " ".join(f"{k}={p.get(k)}" for k in ("tramas_perdidas", "descartadas_micro", "crc_mal"))))
    malas = int(va.corruptas(cap).sum())
    if malas or p.get("huecos"):
        print(f"  AVISO: {malas} muestra(s) corrupta(s) y {p.get('huecos', 0)} hueco(s): "
              "es lo esperable al prender o apagar el motor si estan en esos instantes")
        if malas:
            print("         corruptas en t = " + ", ".join(f"{v:.2f}" for v in cap.t[va.corruptas(cap)][:10]) + " s")

    tramos = vm.marchas(cap)
    apagado = vm.reposos(cap, tramos)
    t_marcha = sum(_dur(tr) for tr in tramos)
    r = _mas_largo(apagado)
    print("  motor prendido: " + (", ".join(f"{a:.1f}-{b:.1f} s" for a, b in tramos) or "nunca"))
    checks.append(("Motor prendido", t_marcha >= MARCHA_MIN_S,
                   f"{t_marcha:.1f} s en {len(tramos)} tramo(s) (min {MARCHA_MIN_S:.0f} s)"))
    checks.append(("Motor apagado", _dur(r) >= REPOSO_MIN_S,
                   f"{_dur(r):.1f} s en el tramo mas largo (min {REPOSO_MIN_S:.0f} s)"))
    if t_marcha < MARCHA_MIN_S:
        return checks, None, [], polos

    sat, total = vm.saturadas_en_marcha(cap, tramos)
    checks.append(("Sin saturacion en marcha", sat <= SATURADAS_MAX * total,
                   f"{sat} de {total} muestras (si hay muchas: --rango mas grande)"))

    seg = vm.seguir(cap, tramos, None if variador else 2.0 * red, polos)
    con_linea = np.isfinite(seg.f_linea)
    if not con_linea.any():
        checks.append(("Linea electrica", False, f"no aparece en ninguna de las {len(seg.t)} ventanas"))
        return checks, seg, [], polos
    f_l = float(np.median(seg.f_linea[con_linea]))
    db_l = float(np.median(seg.db_linea[con_linea]))
    if variador:
        checks.append(("Linea electrica", db_l >= LINEA_DB,
                       f"entre {np.min(seg.f_linea[con_linea]):.2f} y {np.max(seg.f_linea[con_linea]):.2f} Hz, "
                       f"{db_l:.1f} dB sobre el piso (min {LINEA_DB:.0f})"))
    else:
        checks.append(("Linea electrica = 2 x red", abs(f_l - 2 * red) <= LINEA_ERROR_HZ and db_l >= LINEA_DB,
                       f"{f_l:.2f} Hz (2 x {red:g} Hz +/- {LINEA_ERROR_HZ}), {db_l:.1f} dB sobre el piso "
                       f"(min {LINEA_DB:.0f})"))
    prendido = vm.linea_en_psd(cap, _mas_largo(tramos), f_l)
    quieto = vm.linea_en_psd(cap, r, f_l) if r else None
    if prendido and quieto:
        dif = 10 * np.log10(prendido / quieto)
        checks.append(("Linea solo con el motor prendido", dif >= LINEA_DB,
                       f"{dif:.1f} dB mas que con el motor apagado (min {LINEA_DB:.0f})"))
    else:
        checks.append(("Linea solo con el motor prendido", False, "no alcanza el tramo con el motor apagado"))

    medidas = np.isfinite(seg.f1)
    checks.append(("Velocidad medida", medidas.mean() >= MEDIDAS_MIN,
                   f"en {int(medidas.sum())} de {len(seg.t)} ventanas de {vm.VENTANA_S:.0f} s "
                   f"(min {MEDIDAS_MIN * 100:.0f} %), {np.median(seg.bandas):.0f} bandas laterales por ventana"))
    esc = vm.escalones(seg)
    if medidas.any():
        rpm = vm.rpm_de(seg.f1[medidas])
        p5, p50, p95 = np.percentile(rpm, [5, 50, 95])
        print(f"  velocidad: mediana {p50:.0f} rpm (5-95 %: {p5:.0f}-{p95:.0f} rpm)")
        if polos:
            # La sincronica sale de la linea de cada ventana: con variador se mueve
            desliz = 1 - seg.f1[medidas] * polos / seg.f_linea[medidas]
            lo, hi = np.percentile(desliz, [5, 95])
            checks.append((f"Por debajo de la sincronica ({polos} polos)", lo >= 0,
                           f"deslizamiento {lo * 100:.1f} a {hi * 100:.1f} %"))
        if rpm_nominal and n_escalones is None:
            checks.append(("No baja de la nominal", p5 >= rpm_nominal * (1 - NOMINAL_TOL),
                           f"{p5:.0f} rpm contra {rpm_nominal:g} rpm a plena carga "
                           f"(tolerancia {NOMINAL_TOL * 100:.0f} %)"))
    print(f"  tramos de velocidad estable ({len(esc)}):")
    for k, e in enumerate(esc, 1):
        sinc = e.f_linea / polos if polos else None
        print(f"    {k:2d}  {e.t0:6.1f}-{e.t1:6.1f} s  {vm.rpm_de(e.f1):6.0f} rpm ({e.f1:6.2f} Hz)"
              + (f"  deslizamiento {(1 - e.f1 / sinc) * 100:5.1f} %" if sinc else "")
              + f"  linea {e.f_linea:7.2f} Hz  vibracion {e.rms_g * 1000:5.0f} mg rms")
    if n_escalones is not None:
        checks.append(("Escalones de velocidad", len(esc) == n_escalones,
                       f"{len(esc)} encontrados, {n_escalones} esperados"))
    return checks, seg, esc, polos


def grafico_motor(cap, seg, esc, polos, rpm_nominal, path):
    """Velocidad, vibracion y linea electrica contra el tiempo: una medida por panel."""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    fig = Figure(figsize=(11, 8))
    FigureCanvasAgg(fig)
    axs = fig.subplots(3, 1, sharex=True, gridspec_kw={"height_ratios": (2, 1, 1)})
    # Sin unir ventanas a traves de un salto de tiempo (motor apagado)
    corte = np.flatnonzero(np.diff(seg.t) > 2.5 * vm.PASO_S)

    def serie(y):
        return np.insert(seg.t, corte + 1, np.nan), np.insert(y, corte + 1, np.nan)

    ax = axs[0]
    ax.plot(*serie(vm.rpm_de(seg.f1)), color="tab:blue", linewidth=1.5, marker="o", markersize=3)
    for e in esc:
        ax.hlines(vm.rpm_de(e.f1), e.t0, e.t1, color="black", linewidth=1.0, alpha=0.6)
    if polos and np.isfinite(seg.f_linea).any():
        sinc = 60.0 * np.nanmedian(seg.f_linea) / polos
        ax.axhline(sinc, color="gray", linestyle="--", linewidth=0.8)
        ax.annotate(f"sincronica {sinc:.0f} rpm", (seg.t[0], sinc), xytext=(0, 3),
                    textcoords="offset points", fontsize=8, color="dimgray")
    if rpm_nominal:
        ax.axhline(rpm_nominal, color="gray", linestyle=":", linewidth=0.8)
        ax.annotate(f"nominal {rpm_nominal:g} rpm", (seg.t[0], rpm_nominal), xytext=(0, 3),
                    textcoords="offset points", fontsize=8, color="dimgray")
    ax.set_ylabel("Velocidad [rpm]")
    ax.set_title(f"{os.path.basename(cap.base)} - {cap.meta.get('nota') or ''}\n"
                 f"Velocidad del rotor (bandas laterales de la linea, ventanas de {vm.VENTANA_S:.0f} s); "
                 "linea negra: tramos estables")
    axs[1].plot(*serie(seg.rms_g * 1000), color="tab:blue", linewidth=1.0)
    axs[1].set_ylabel("Vibracion [mg rms]")
    axs[1].set_ylim(bottom=0)
    axs[2].plot(*serie(seg.f_linea), color="tab:blue", linewidth=1.0)
    axs[2].set_ylabel("Linea electrica [Hz]")
    axs[2].set_xlabel("Tiempo [s]")
    axs[2].ticklabel_format(useOffset=False, axis="y")
    for ax in axs:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prueba", choices=("equivalencia", "tono", "motor"))
    parser.add_argument("captura", help="Captura (.csv, .json o nombre base)")
    parser.add_argument("--frecuencia", type=float, default=None, help="Frecuencia del tono (prueba tono)")
    parser.add_argument("--inicio", type=float, default=None,
                        help="Segundo en que se prende el tono, si el detector no lo encuentra (prueba tono)")
    parser.add_argument("--red", type=float, default=50.0, help="Frecuencia de la red en Hz (prueba motor)")
    parser.add_argument("--rpm-nominal", type=float, default=None,
                        help="Velocidad a plena carga, de la chapa o la hoja de datos (prueba motor)")
    parser.add_argument("--polos", type=int, default=None,
                        help="Polos del motor; si no se indica, sale de --rpm-nominal (prueba motor)")
    parser.add_argument("--variador", action="store_true",
                        help="La alimentacion cambia de frecuencia: la linea no esta en 2 x red (prueba motor)")
    parser.add_argument("--escalones", type=int, default=None,
                        help="Cantidad de velocidades distintas esperadas (prueba motor)")
    args = parser.parse_args()

    cap = va.cargar(args.captura)
    cfg = cap.meta.get("filtro") or {}
    print(f"{args.prueba.upper()}: {cap.base}  (nota: {cap.meta.get('nota') or '-'}, odr={cap.meta.get('odr_hz')} Hz, "
          f"filtro={cfg.get('filtro')} eje={cfg.get('eje')} taps={cfg.get('taps')} delta={cfg.get('delta')} "
          f"mu={cfg.get('mu')}/1000)")
    if args.prueba == "tono":
        if args.frecuencia is None:
            parser.error("la prueba tono necesita --frecuencia")
        checks = tono(cap, args.frecuencia, args.inicio)
    elif args.prueba == "motor":
        checks, seg, esc, polos = motor(cap, args.red, args.rpm_nominal, args.polos, args.variador, args.escalones)
        if seg is not None and len(seg.t):
            png = cap.base + "_motor.png"
            grafico_motor(cap, seg, esc, polos, args.rpm_nominal, png)
            print(f"  grafico: {png}")
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
