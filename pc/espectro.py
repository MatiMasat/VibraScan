"""
VibraScan - espectro de una captura (etapa 5).

Grafica la densidad espectral de potencia (PSD, metodo de Welch) de los tres
ejes y de su suma, marca los picos y los lista. Con --espectrograma agrega
frecuencia contra tiempo, para ver cuando aparece cada vibracion.

Los picos que coinciden con una armonica del pico principal se marcan; si la
armonica esta por encima de fs/2, aparece reflejada (aliasing) en una
frecuencia mas baja: no es una vibracion real en esa frecuencia.

Uso:
    python espectro.py capturas/captura_<fecha>
    python espectro.py capturas/captura_<fecha> --desde 10 --hasta 30
    python espectro.py capturas/captura_<fecha> --espectrograma --png figura.png
    python espectro.py capturas/captura_<fecha> --nperseg 1024 --fmax 200
"""

import argparse
import os
import sys

import numpy as np

import vibrascan_archivo as va
import vibrascan_espectro as ve

COLORS = ("tab:blue", "tab:orange", "tab:green", "black")
NOMBRES = ("x", "y", "z", "suma")
MAX_PICOS = 8


def tabla(esp, k, max_picos=MAX_PICOS):
    """Picos de la curva k con la explicacion de cada uno respecto del principal."""
    pks = ve.picos(esp.f, esp.psd[k])[:max_picos]
    filas = []
    for j, pk in enumerate(pks):
        nota = "principal" if j == 0 else ""
        if j > 0:
            arm = ve.armonica_de(pk, pks[0].f, esp.fs, esp.df)
            if arm:
                kk, refl = arm
                nota = (f"armonica {kk} de {pks[0].f:.2f} Hz"
                        + (f" ({kk * pks[0].f:.1f} Hz reflejada por aliasing)" if refl else ""))
        filas.append((pk, nota))
    return filas


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("captura", help="Captura (.csv, .json o nombre base)")
    parser.add_argument("--desde", type=float, default=None, help="Segundo inicial")
    parser.add_argument("--hasta", type=float, default=None, help="Segundo final")
    parser.add_argument("--nperseg", type=int, default=ve.NPERSEG,
                        help=f"Muestras por tramo de Welch (default {ve.NPERSEG}: ~0,19 Hz de resolucion)")
    parser.add_argument("--fmax", type=float, default=None, help="Frecuencia maxima a graficar")
    parser.add_argument("--espectrograma", action="store_true", help="Agrega frecuencia contra tiempo (suma de ejes)")
    parser.add_argument("--png", default=None, help="Guarda la imagen en este archivo y no abre ventana")
    args = parser.parse_args()

    if args.png:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cap = va.cargar(args.captura)
    try:
        esp = ve.psd(cap, args.nperseg, args.desde, args.hasta)
    except ValueError as e:
        print(f"No se puede calcular el espectro: {e}")
        return 1

    m = cap.meta
    t0 = args.desde if args.desde is not None else cap.t[0]
    t1 = args.hasta if args.hasta is not None else cap.t[-1]
    print(f"{os.path.basename(cap.base)}  nota: {m.get('nota') or '-'}")
    print(f"  tramo {t0:.1f}-{t1:.1f} s, segmentos {esp.segmentos}, fs medida {esp.fs:.2f} Hz "
          f"(odr {m.get('odr_hz')}), {esp.tramos} tramos de {esp.nperseg} muestras, resolucion {esp.df:.3f} Hz")
    print("  Parseval (integral de la PSD / varianza) x/y/z: " + "/".join(f"{v:.3f}" for v in esp.parseval())
          + "  (~1 si la senal es estacionaria; menos con golpes)")
    banda = (esp.f > 0.05 * esp.fs) & (esp.f < 0.48 * esp.fs)
    piso = np.median(esp.psd[:3, banda], axis=1)
    if esp.parseval().min() < 0.95:
        print("  AVISO: la senal no es estacionaria (golpes o transitorios). Los picos de la tabla pueden ser")
        print("         ondulaciones de una resonancia ancha y no tonos: mirar el --espectrograma.")
    print("  piso de ruido x/y/z: " + "/".join(f"{np.sqrt(v) * 1e6:.0f}" for v in piso) + " ug/raizHz")
    for k in (3, 0, 1, 2):
        filas = tabla(esp, k)
        if not filas:
            print(f"  {NOMBRES[k]:4}: sin picos (> {ve.PICO_MIN_DB:.0f} dB sobre el piso)")
            continue
        for j, (pk, nota) in enumerate(filas):
            print(f"  {NOMBRES[k] if j == 0 else '':4}  {pk.f:8.2f} Hz  {pk.db:5.1f} dB  {pk.amp_g * 1000:7.3f} mg rms  {nota}")

    nplots = 2 if args.espectrograma else 1
    fig, axs = plt.subplots(nplots, 1, figsize=(12, 4.5 * nplots), squeeze=False)
    ax = axs[0, 0]
    for k in range(4):
        ax.semilogy(esp.f, esp.psd[k], color=COLORS[k], linewidth=1.2 if k == 3 else 0.7,
                    alpha=1.0 if k == 3 else 0.8, label=NOMBRES[k])
    for pk, nota in tabla(esp, 3):
        ax.annotate(f"{pk.f:.2f} Hz" + ("\n(alias)" if "aliasing" in nota else ""),
                    xy=(pk.f, esp.psd[3][pk.i]), xytext=(0, 12), textcoords="offset points",
                    ha="center", fontsize=8, color="red", arrowprops={"arrowstyle": "-", "color": "red"})
    ax.set_xlim(0, args.fmax or esp.f[-1])
    lo, hi = ax.get_ylim()
    ax.set_ylim(lo, hi * 8)            # lugar arriba para las etiquetas de los picos
    ax.set_xlabel("Frecuencia [Hz]")
    ax.set_ylabel("PSD [g^2/Hz]")
    ax.grid(True, which="both", alpha=0.3)
    for line in ax.legend(loc="upper right").get_lines():
        line.set_linewidth(2.5)
    ax.set_title(f"{os.path.basename(cap.base)} - {m.get('nota') or ''}\n"
                 f"Welch: {esp.tramos} tramos de {esp.nperseg} (Hann, 50 %), fs medida {esp.fs:.2f} Hz, "
                 f"resolucion {esp.df:.3f} Hz, tramo {t0:.1f}-{t1:.1f} s")

    if args.espectrograma:
        ax2 = axs[1, 0]
        t, f, P = ve.espectrograma(cap, 3, desde=args.desde, hasta=args.hasta)
        db = 10 * np.log10(np.where(np.isfinite(P) & (P > 0), P, np.nan))
        vmin, vmax = np.nanpercentile(db, [5, 99.9])
        im = ax2.imshow(db, origin="lower", aspect="auto", extent=(t[0], t[-1], f[0], f[-1]),
                        vmin=vmin, vmax=vmax, cmap="viridis", interpolation="nearest")
        ax2.set_ylim(0, args.fmax or f[-1])
        ax2.set_xlabel("Tiempo [s]")
        ax2.set_ylabel("Frecuencia [Hz]")
        ax2.set_title(f"Espectrograma (suma de ejes): tramos de {ve.ESPECTROGRAMA_NPERSEG} muestras, "
                      f"resolucion {f[1] - f[0]:.1f} Hz; blanco = sin datos")
        fig.colorbar(im, ax=ax2, label="dB (g^2/Hz)")

    fig.tight_layout()
    if args.png:
        fig.savefig(args.png, dpi=110)
        print(f"Imagen guardada en {args.png}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
