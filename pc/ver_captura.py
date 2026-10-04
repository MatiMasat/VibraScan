"""
VibraScan - visor de capturas (etapa 4).

Grafica una captura de captura.py (o de monitor_vivo.py) contra el tiempo:
los tres ejes y |a|. Con la barra de matplotlib se hace zoom; el grafico se
redibuja con el detalle que corresponde.

- Si hay mas muestras que pixeles, cada pixel muestra el minimo y el maximo de
  sus muestras: ningun pico desaparece, por larga que sea la captura.
- Los huecos y reconexiones son franjas grises; la linea no los cruza.
- Las muestras en el tope del rango (saturadas) se marcan en rojo.
- Los golpes detectados se marcan sobre |a|.

Uso:
    python ver_captura.py capturas/captura_20261004_120000
    python ver_captura.py capturas/captura_20261004_120000.csv --desde 100 --hasta 160
    python ver_captura.py capturas/captura_20261004_120000 --png figura.png   (sin ventana)
"""

import argparse
import os
import sys

import numpy as np

import vibrascan_archivo as va

COLORS = ("tab:blue", "tab:orange", "tab:green", "black")
MAX_MARKS = 2000   # marcas de saturacion dibujadas a la vez


def resumen(cap, gs):
    m = cap.meta
    p = m.get("perdidas", {})
    print(f"{os.path.basename(cap.base)}  {m.get('inicio', '')}  nota: {m.get('nota') or '-'}")
    print(f"  {m.get('duracion_s', 0):.1f} s, {len(cap.t)} muestras, odr={m.get('odr_hz')} Hz, "
          f"rango={m.get('rango_g')} g, periodo={m.get('periodo_us')} us, "
          f"{len(m.get('segmentos', []))} segmento(s)" + ("" if m.get("completa") else "  (CAPTURA INCOMPLETA)"))
    print(f"  perdidas: tramas={p.get('tramas_perdidas')} micro={p.get('descartadas_micro')} "
          f"huecos={p.get('huecos')} crc={p.get('crc_mal')} reconexiones={p.get('reconexiones')}")
    for s in m.get("segmentos", [])[1:]:
        print(f"  segmento {s['seg']} desde {s['t_s']:.3f} s: {s['motivo']}")
    sat = va.saturadas(cap)
    if sat is not None and sat.any():
        print(f"  SATURACION: {int(sat.sum())} muestras en el tope de +/-{m.get('rango_g')} g "
              "(el pico real es mayor; usar un rango mas grande)")
    if len(cap.t) and cap.t[-1] - cap.t[0] >= 120:
        print("  por minuto (media / rms en mg):")
        for w in va.por_ventana(cap):
            mean = "/".join(f"{v * 1000:+7.1f}" for v in w["media_g"])
            rms = "/".join(f"{v * 1000:4.1f}" for v in w["rms_g"])
            print(f"    min {w['ventana']:3d}: x/y/z {mean}   rms {rms}")
    print(f"  golpes: {len(gs)}")
    for k, g in enumerate(gs, 1):
        vuelta = f"{g.vuelta_ms:.0f} ms" if g.vuelta_ms is not None else "no volvio"
        print(f"    {k:3d}  t={g.t_s:9.3f} s  pico |a|={g.pico_g:5.2f} g  {g.dur_ms:5.0f} ms sobre el umbral  "
              f"vuelta al reposo {vuelta}" + ("  SATURADO" if g.saturado else ""))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("captura", help="Archivo .csv/.json o nombre base de la captura")
    parser.add_argument("--desde", type=float, default=None, help="Segundo inicial a mostrar")
    parser.add_argument("--hasta", type=float, default=None, help="Segundo final a mostrar")
    parser.add_argument("--png", default=None, help="Guarda la imagen en este archivo y no abre ventana")
    args = parser.parse_args()

    if args.png:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cap = va.cargar(args.captura)
    if len(cap.t) == 0:
        print("La captura no tiene muestras.")
        return 1
    gs = va.golpes(cap)
    resumen(cap, gs)

    g = cap.g
    series = [g[:, 0], g[:, 1], g[:, 2], cap.modulo]
    tramos = cap.tramos()
    sat = va.saturadas(cap)
    lim = cap.limites()

    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(12, 8))
    lines = []
    for ax, y, color, name in zip(axs, series, COLORS, ("ax [g]", "ay [g]", "az [g]", "|a| [g]")):
        (ln,) = ax.plot([], [], color=color, linewidth=0.8)
        lines.append(ln)
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
        lo, hi = float(y.min()), float(y.max())
        pad = max(0.05, (hi - lo) * 0.08)
        ax.set_ylim(lo - pad, hi + pad)
        if lim is not None and name != "|a| [g]":
            for v in lim:   # tope del rango, si entra en la escala
                if lo - pad <= v * cap.escala <= hi + pad:
                    ax.axhline(v * cap.escala, color="red", linewidth=0.6, linestyle=":")
    (sat_marks,) = axs[3].plot([], [], "r.", markersize=3, label="saturada")

    # Huecos y reconexiones: entre el final de un segmento y el inicio del siguiente
    for (a0, a1), (b0, _) in zip(tramos, tramos[1:]):
        for ax in axs:
            ax.axvspan(cap.t[a1 - 1], cap.t[b0], color="0.6", alpha=0.35, linewidth=0)
    for gol in gs:
        axs[3].annotate("", xy=(gol.t_pico_s, gol.pico_g), xytext=(gol.t_pico_s, gol.pico_g + 0.3),
                        arrowprops={"arrowstyle": "->", "color": "red"})

    m = cap.meta
    p = m.get("perdidas", {})
    fig.suptitle(f"{os.path.basename(cap.base)} - {m.get('nota') or ''}\n"
                 f"odr={m.get('odr_hz')} Hz  rango=+/-{m.get('rango_g')} g  {m.get('duracion_s', 0):.0f} s  "
                 f"segmentos={len(tramos)}  huecos={p.get('huecos')}  reconexiones={p.get('reconexiones')}  "
                 f"golpes={len(gs)}" + (f"  SATURADAS={int(sat.sum())}" if sat is not None and sat.any() else ""))
    axs[-1].set_xlabel("Tiempo [s] (reloj del micro)")

    def redraw(_ax=None):
        x0, x1 = axs[-1].get_xlim()
        width = max(200, int(axs[-1].bbox.width))
        for ln, y in zip(lines, series):
            xs, ys = [], []
            for s0, s1 in tramos:
                i0 = max(s0, int(np.searchsorted(cap.t, x0)) - 1)
                i1 = min(s1, int(np.searchsorted(cap.t, x1)) + 1)
                if i1 - i0 < 1:
                    continue
                frac = (cap.t[i1 - 1] - cap.t[i0]) / max(x1 - x0, 1e-9)
                tx, ty = va.envolvente(cap.t[i0:i1], y[i0:i1], max(10, int(width * min(1.0, frac))))
                xs += [tx, [np.nan]]          # NaN: la linea no cruza de un segmento a otro
                ys += [ty, [np.nan]]
            ln.set_data(np.concatenate(xs) if xs else [], np.concatenate(ys) if ys else [])
        if sat is not None:
            i0, i1 = np.searchsorted(cap.t, [x0, x1])
            idx = np.flatnonzero(sat[i0:i1]) + i0
            idx = idx[:: max(1, len(idx) // MAX_MARKS)]
            sat_marks.set_data(cap.t[idx], series[3][idx])
        fig.canvas.draw_idle()

    t_lo = args.desde if args.desde is not None else cap.t[0]
    t_hi = args.hasta if args.hasta is not None else cap.t[-1]
    axs[-1].set_xlim(t_lo, t_hi)
    fig.tight_layout()
    redraw()
    axs[-1].callbacks.connect("xlim_changed", redraw)

    if args.png:
        fig.savefig(args.png, dpi=110)
        print(f"Imagen guardada en {args.png}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
