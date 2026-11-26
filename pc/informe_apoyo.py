"""
VibraScan - figuras de la serie del 2026-11-24: niveles del selector con el
apoyo del eje normal y desplazado ~3 mm hacia arriba (etapa 7).

Lee las capturas pc/capturas/serie1124_* (no estan en git) y escribe en
docs/img/niveles/ las figuras de docs/prueba_motor.md, seccion 8:

    psd_niveles.png             10 espectros (Welch) en escala comun, como la
                                Figura 1 del informe del microfono
    espectrogramas_niveles.png  10 espectrogramas en escala comun (Figura 2)
    superpuestas_niveles.png    normal y desplazado superpuestos, 0-150 Hz
    linea_2x_niveles.png        la linea de 2x por nivel
    nivel<k>_<normal|desplazado>.png   cada espectro por separado

y una tabla con los indicadores por consola.

Numeracion del selector: la del informe del microfono (Nivel 1 = maxima
potencia, Nivel 5 = minima); las capturas se llaman p<nivel>_<N|A>.

    python informe_apoyo.py
"""

import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import vibrascan_archivo as va
import vibrascan_espectro as ve
import vibrascan_motor as vm

CAPTURAS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "capturas")
SALIDA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "img", "niveles")

# (nivel, captura con el apoyo normal, con el apoyo desplazado), de minima a maxima potencia
NIVELES = [(5, "p5_N2", "p5_A"), (4, "p4_N2", "p4_A"), (3, "p3_N", "p3_A"), (2, "p2_N", "p2_A"),
           (1, "p1_N", "p1_A")]
REPETICION = (3, "f1_N", "f2_A")    # nivel 3 repetido al final de la serie
ANDANDO = ("p4_N2", "p4_A")         # el motor ya andaba al empezar a grabar
NPERSEG = 4096                      # 1,3 s a 3160 Hz: 0,77 Hz de resolucion
FMAX = 1000.0
COLOR = {"N": "tab:blue", "A": "tab:orange"}
ROTULO = {"N": "apoyo normal", "A": "apoyo desplazado ~3 mm"}


def tramo(cap, nombre):
    """Segundos de marcha estable que se analizan: sin el arranque ni el apagado."""
    if nombre in ANDANDO:
        return 1.0, 11.5
    a, c = max(vm.marchas(cap), key=lambda x: x[1] - x[0])
    return max(a + 4.0, c - 14.5), c - 2.5


def medir(nombre):
    cap = va.cargar(os.path.join(CAPTURAS, "serie1124_" + nombre))
    g, t = cap.g, cap.t
    f0, f1 = tramo(cap, nombre)
    i = (t >= f0) & (t < f1)
    e = vm.espectro_ventana(g[i], t[i], 1e6 / cap.meta["periodo_us"])
    df = e.f[1] - e.f[0]
    fl = ve.piso(e.p, df)

    def linea(f, ancho):
        """Amplitud rms de la linea (mg): energia sobre el piso en f +/- ancho."""
        return 1e3 * np.sqrt(max((e.p - fl)[np.abs(e.f - f) <= ancho].sum(), 0) * df)

    f1x = float(np.nanmedian(vm.seguir(cap, [(f0 - 1, f1 + 1)], 100.0, 4).f1))
    por_s = [np.sqrt(g[(t >= s) & (t < s + 1)].var(0).sum()) * 1e3 for s in np.arange(f0, f1 - 0.9, 1.0)]
    return dict(nombre=nombre, cap=cap, f0=f0, f1=f1, psd=ve.psd(cap, NPERSEG, f0, f1), f1x=f1x,
                l1=linea(f1x, 0.35), l2=linea(2 * f1x, 0.5), l100=linea(100.0, 0.5),
                total=np.sqrt(g[i].var(0).sum()) * 1e3, rms=g[i].std(0) * 1e3,
                variacion=(max(por_s) - min(por_s)) / np.mean(por_s) * 100)


def db(p):
    return 10 * np.log10(np.maximum(p, 1e-14))


def panel_psd(ax, r, k, nivel, ylim):
    f, p = r["psd"].f, r["psd"].psd[3]
    m = (f >= 20) & (f <= FMAX)
    ax.plot(f[m], db(p[m]), color=COLOR[k], lw=0.8)
    for h in range(1, 10):          # linea electrica y armonicas
        j = np.abs(f - 100 * h) <= 1
        ax.plot(100 * h, db(p[j].max()), "v", color="black", ms=4)
    j = np.abs(f - 2 * r["f1x"]) <= 1
    ax.plot(2 * r["f1x"], db(p[j].max()), "D", color="tab:purple", ms=5)
    ax.annotate(f"2x: {r['l2']:.1f} mg", (2 * r["f1x"], db(p[j].max())), xytext=(8, 4),
                textcoords="offset points", fontsize=8, color="tab:purple")
    ax.set_xlim(20, FMAX)
    ax.set_ylim(*ylim)
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Nivel {nivel} - {ROTULO[k]} ({r['nombre']}): total {r['total']:.0f} mg rms",
                 fontsize=9)


def main():
    os.makedirs(SALIDA, exist_ok=True)
    R = {(n, k): medir(c) for n, cn, ca in NIVELES for k, c in (("N", cn), ("A", ca))}
    rep = {k: medir(c) for k, c in (("N", REPETICION[1]), ("A", REPETICION[2]))}

    print("Nivel  apoyo        captura  tramo s      total mg  var %   x/y/z mg        1x Hz   1x mg  2x mg  100 Hz mg")
    for n, *_ in NIVELES:
        for k in ("N", "A"):
            r = R[(n, k)]
            print(f"{n:5}  {ROTULO[k][:12]:12} {r['nombre']:8} {r['f0']:5.1f}-{r['f1']:5.1f}  {r['total']:7.0f}  "
                  f"{r['variacion']:5.0f}  {'/'.join(f'{v:.0f}' for v in r['rms']):14}  {r['f1x']:6.2f}  "
                  f"{r['l1']:5.1f}  {r['l2']:5.1f}  {r['l100']:7.0f}")
    for k in ("N", "A"):
        r = rep[k]
        print(f"3 rep  {ROTULO[k][:12]:12} {r['nombre']:8} {r['f0']:5.1f}-{r['f1']:5.1f}  {r['total']:7.0f}  "
              f"{r['variacion']:5.0f}  {'/'.join(f'{v:.0f}' for v in r['rms']):14}  {r['f1x']:6.2f}  "
              f"{r['l1']:5.1f}  {r['l2']:5.1f}  {r['l100']:7.0f}")

    # Escala comun para los 10 espectros
    todos = np.concatenate([db(r["psd"].psd[3][(r["psd"].f >= 20) & (r["psd"].f <= FMAX)]) for r in R.values()])
    ylim = (np.floor(np.percentile(todos, 1) / 10) * 10, np.ceil(todos.max() / 10) * 10 + 5)

    fig, axs = plt.subplots(5, 2, figsize=(14, 17), sharex=True, sharey=True)
    for fila, (n, *_) in enumerate(NIVELES):
        for col, k in enumerate(("N", "A")):
            panel_psd(axs[fila, col], R[(n, k)], k, n, ylim)
        axs[fila, 0].set_ylabel("PSD [dB re 1 g²/Hz]")
    for ax in axs[-1]:
        ax.set_xlabel("Frecuencia [Hz]")
    fig.suptitle("Acelerómetro sobre la carcasa: espectro (Welch, suma de ejes) por nivel del selector, "
                 "escala común\nNivel 5 = mínima potencia (arriba), Nivel 1 = máxima (abajo). "
                 "Triángulos: 100 Hz y múltiplos. Rombo: 2x la rotación (~49 Hz)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(os.path.join(SALIDA, "psd_niveles.png"), dpi=100)
    plt.close(fig)

    for (n, k), r in R.items():
        fig, ax = plt.subplots(figsize=(10, 4))
        panel_psd(ax, r, k, n, ylim)
        ax.set_xlabel("Frecuencia [Hz]")
        ax.set_ylabel("PSD [dB re 1 g²/Hz]")
        fig.tight_layout()
        fig.savefig(os.path.join(SALIDA, f"nivel{n}_{'normal' if k == 'N' else 'desplazado'}.png"), dpi=100)
        plt.close(fig)

    # Espectrogramas, escala de color comun
    esp = {}
    for (n, k), r in R.items():
        t, f, P = ve.espectrograma(r["cap"], 3)
        esp[(n, k)] = (t, f, 10 * np.log10(np.where(np.isfinite(P) & (P > 0), P, np.nan)))
    vals = np.concatenate([d[f <= FMAX].ravel() for t, f, d in esp.values()])
    vmin, vmax = np.nanpercentile(vals, [5, 99.9])
    fig, axs = plt.subplots(5, 2, figsize=(14, 17), sharey=True)
    for fila, (n, *_) in enumerate(NIVELES):
        for col, k in enumerate(("N", "A")):
            t, f, d = esp[(n, k)]
            r = R[(n, k)]
            ax = axs[fila, col]
            im = ax.imshow(d, origin="lower", aspect="auto", extent=(t[0], t[-1], f[0], f[-1]),
                           vmin=vmin, vmax=vmax, cmap="viridis", interpolation="nearest")
            for x in (r["f0"], r["f1"]):
                ax.axvline(x, color="white", ls="--", lw=0.8)
            ax.set_ylim(0, FMAX)
            ax.set_title(f"Nivel {n} - {ROTULO[k]} ({r['nombre']})", fontsize=9)
            if fila == 4:
                ax.set_xlabel("Tiempo [s]")
        axs[fila, 0].set_ylabel("Frecuencia [Hz]")
    fig.suptitle("Espectrogramas (suma de ejes, tramos de 0,32 s), escala de color común. "
                 "Líneas blancas: tramo analizado en el espectro", fontsize=11)
    fig.tight_layout(rect=(0, 0, 0.93, 0.97))
    fig.colorbar(im, ax=axs, label="dB re 1 g²/Hz", fraction=0.02, pad=0.01)
    fig.savefig(os.path.join(SALIDA, "espectrogramas_niveles.png"), dpi=72)
    plt.close(fig)

    # Superpuestas 0-150 Hz
    fig, axs = plt.subplots(5, 1, figsize=(10, 16), sharex=True, sharey=True)
    for ax, (n, *_) in zip(axs, NIVELES):
        for k in ("N", "A"):
            r = R[(n, k)]
            m = r["psd"].f <= 150
            ax.plot(r["psd"].f[m], db(r["psd"].psd[3][m]), color=COLOR[k], lw=1.0,
                    label=f"{ROTULO[k]} ({r['nombre']})")
        ax.axvline(2 * R[(n, "A")]["f1x"], color="gray", ls=":", lw=0.9)
        ax.set_title(f"Nivel {n}: línea de 2x {R[(n, 'N')]['l2']:.1f} -> {R[(n, 'A')]['l2']:.1f} mg rms",
                     fontsize=10)
        ax.set_ylabel("PSD [dB re 1 g²/Hz]")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="upper left")
    axs[-1].set_xlabel("Frecuencia [Hz]")
    fig.suptitle("Apoyo normal y desplazado superpuestos (0-150 Hz). Punteada: 2x la rotación", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(SALIDA, "superpuestas_niveles.png"), dpi=100)
    plt.close(fig)

    # Linea de 2x por nivel
    fig, ax = plt.subplots(figsize=(9, 4.5))
    x = np.arange(len(NIVELES))
    for d, k in ((-0.19, "N"), (0.19, "A")):
        v = [R[(n, k)]["l2"] for n, *_ in NIVELES]
        ax.bar(x + d, v, 0.36, color=COLOR[k], label=ROTULO[k])
        for xx, vv in zip(x + d, v):
            ax.annotate(f"{vv:.1f}", (xx, vv), xytext=(0, 2), textcoords="offset points", ha="center", fontsize=8)
    ax.set_xticks(x, [f"Nivel {n}" + (" (golpeteo)" if n == 5 else "") for n, *_ in NIVELES])
    ax.set_ylabel("Línea de 2x [mg rms, suma de ejes]")
    ax.set_title("Línea de 2x la rotación (~49 Hz) por nivel del selector")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(SALIDA, "linea_2x_niveles.png"), dpi=110)
    plt.close(fig)
    print("Figuras en", os.path.normpath(SALIDA))
    return 0


if __name__ == "__main__":
    sys.exit(main())
