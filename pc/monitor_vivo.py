"""
VibraScan - visor en tiempo real de los tres ejes.

Grafica lo que manda la Black Pill por el puerto COM virtual, para ver como
responde el sensor al moverlo. No guarda nada: para capturar y evaluar la
etapa se usa etapa1_prueba.py.

Uso:
    python monitor_vivo.py --port COM8
    python monitor_vivo.py --port COM8 --window 5 --ylim 4

Cerrar la ventana para terminar. El puerto no puede estar abierto por otro
programa (monitor serie, etapa1_prueba.py) al mismo tiempo.
"""

import argparse
import collections
import math
import sys
import threading

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

from etapa1_prueba import G_PER_LSB, parse_sample, parse_status

AXES = (("X", "tab:blue"), ("Y", "tab:orange"), ("Z", "tab:green"))


class Reader(threading.Thread):
    """Lee el puerto en segundo plano y guarda las ultimas muestras."""

    def __init__(self, port, maxlen):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.samples = collections.deque(maxlen=maxlen)   # (n, x_g, y_g, z_g)
        self.status = {}
        self.gaps = 0
        self.lock = threading.Lock()
        self.running = True

    def run(self):
        pending = b""
        self.ser.reset_input_buffer()
        first_line = True
        last_n = None
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                break
            if not chunk:
                continue
            pending += chunk
            *lines, pending = pending.split(b"\n")
            for raw in lines:
                line = raw.decode("ascii", errors="replace").strip()
                if first_line:
                    first_line = False   # puede venir cortada
                    continue
                if line.startswith("#"):
                    with self.lock:
                        self.status = parse_status(line)
                    continue
                sample = parse_sample(line)
                if sample is None:
                    continue
                n, x, y, z = sample
                with self.lock:
                    if last_n is not None and n != last_n + 1:
                        self.gaps += 1
                    last_n = n
                    self.samples.append((n, x * G_PER_LSB, y * G_PER_LSB, z * G_PER_LSB))

    def snapshot(self):
        with self.lock:
            return list(self.samples), dict(self.status), self.gaps

    def close(self):
        self.running = False
        self.ser.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Puerto serie, ej: COM8")
    parser.add_argument("--window", type=float, default=10.0, help="Segundos visibles (default: 10)")
    parser.add_argument("--ylim", type=float, default=2.0, help="Escala vertical en g, +/- (default: 2)")
    parser.add_argument("--fs", type=float, default=100.0, help="Tasa de muestreo nominal (default: 100)")
    args = parser.parse_args()

    try:
        reader = Reader(args.port, int(args.window * args.fs))
    except serial.SerialException as e:
        print(f"No se pudo abrir {args.port}: {e}")
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
    axs[-1].set_xlabel("Tiempo [s] (0 = ahora)")
    title = fig.suptitle(f"VibraScan - {args.port} - esperando datos...")
    fig.tight_layout()

    def update(_frame):
        samples, status, gaps = reader.snapshot()
        if not samples:
            return
        fs = float(status.get("odr", 0) or 0) or args.fs
        n_last = samples[-1][0]
        t = [(s[0] - n_last) / fs for s in samples]
        for i, ln in enumerate(lines):
            ln.set_data(t, [s[i + 1] for s in samples])
        x, y, z = samples[-1][1:]
        for lbl, v in zip(labels, (x, y, z)):
            lbl.set_text(f"{v:+.3f} g")
        mod = math.sqrt(x * x + y * y + z * z)
        title.set_text(f"VibraScan - {args.port} - |a| = {mod:.3f} g - "
                       f"estado={status.get('estado', '?')} drop={status.get('drop', '?')} "
                       f"saltos={gaps}")

    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841
    plt.show()
    reader.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
