"""
VibraScan - filtro adaptativo, referencia en Python (etapa 7).

Hace lo mismo que el micro (Core/Src/filtro.c y Core/Src/nlms.c), con la
misma estructura y los mismos nombres, en doble precision: es la "matematica
ideal" contra la que se compara lo que calcula el micro en float32.
Justificacion de la eleccion: docs/filtro_adaptativo.md.

Por muestra (Filtro.procesar):
    1. Pasaaltos de 1er orden (FC_HZ): saca la gravedad y la deriva.
    2. NLMS como realzador de lineas (ALE), nlms():
         u = [x[n-delta], ..., x[n-delta-taps+1]]
         y = w . u                       parte periodica (predecible)
         e = x[n] - y                    parte de banda ancha
         w = w + mu * e * u / (EPS + u . u)
"""

import numpy as np

FC_HZ = 1.0          # corte del pasaaltos (igual que FILTRO_FC_HZ en filtro.h)
EPS = 1.0e-6         # g^2, igual que NLMS_EPS en nlms.h
TAPS, DELTA, MU = 32, 2, 0.010   # valores por defecto del firmware

OFF, NLMS = 0, 1     # filtro_tipo_t
NOMBRES = {OFF: "off", NLMS: "nlms"}


class Nlms:
    """NLMS como ALE (nlms.c). Una muestra por llamada, como en el micro."""

    def __init__(self, taps=TAPS, delta=DELTA, mu=MU):
        if not (1 <= taps <= 64 and 1 <= delta <= 32 and mu > 0):
            raise ValueError("parametros fuera de rango")
        self.taps, self.delta, self.mu = taps, delta, mu
        self.reiniciar()

    def reiniciar(self):
        self.w = np.zeros(self.taps)
        self.hist = np.zeros(self.taps + self.delta)      # hist[0] = x[n-1], ...

    def procesar(self, x):
        u = self.hist[self.delta - 1:self.delta - 1 + self.taps]
        y = float(self.w @ u)
        e = x - y
        self.w += (self.mu * e / (EPS + float(u @ u))) * u
        self.hist[1:] = self.hist[:-1]
        self.hist[0] = x
        return y, e


class Pasaaltos:
    """Pasaaltos RC de 1er orden; la primera muestra inicializa el estado."""

    def __init__(self, fs, fc=FC_HZ):
        rc = 1.0 / (2.0 * np.pi * fc)
        self.a = rc / (rc + 1.0 / fs)
        self.reiniciar()

    def reiniciar(self):
        self.listo = False
        self.x1 = self.y1 = 0.0

    def procesar(self, x):
        if not self.listo:
            self.x1, self.y1, self.listo = x, 0.0, True
            return 0.0
        y = self.a * (self.y1 + x - self.x1)
        self.x1, self.y1 = x, y
        return y


class Filtro:
    """Cadena de filtrado (filtro.c): pasaaltos + algoritmo adaptativo."""

    def __init__(self, tipo=NLMS, fs=3200.0, taps=TAPS, delta=DELTA, mu=MU):
        self.tipo = tipo
        self.pasaaltos = Pasaaltos(fs)
        self.nlms = Nlms(taps, delta, mu) if tipo == NLMS else None

    def reiniciar(self):
        self.pasaaltos.reiniciar()
        if self.nlms:
            self.nlms.reiniciar()

    def procesar(self, x):
        xp = self.pasaaltos.procesar(x)
        if self.tipo == NLMS:
            return self.nlms.procesar(xp)
        return xp, 0.0

    def procesar_serie(self, x):
        """Procesa un arreglo; devuelve (y, e) como arreglos."""
        y = np.empty(len(x))
        e = np.empty(len(x))
        for i, v in enumerate(x):
            y[i], e[i] = self.procesar(float(v))
        return y, e


def nlms(x, taps=TAPS, delta=DELTA, mu=MU):
    """NLMS (sin pasaaltos) sobre un arreglo: devuelve (y, e)."""
    f = Nlms(taps, delta, mu)
    out = np.array([f.procesar(float(v)) for v in x])
    return out[:, 0], out[:, 1]


def snr_banda_db(x, fs, f_tono):
    """Relacion senal/ruido de un tono en TODA la banda (0 a fs/2): potencia del
    tono contra la potencia de todo lo demas. Es la medida que mejora un
    realzador de lineas (ALE): deja pasar una banda angosta alrededor del tono
    (~fs/taps de ancho) y saca el ruido del resto de la banda; una medida local
    (pico sobre el piso de +/-20 Hz) no ve esa mejora. Devuelve (snr_db, amp_rms_g)."""
    import vibrascan_espectro as ve
    f, p, _ = ve.welch(x, fs)
    df = f[1] - f[0]
    i = int(round(f_tono / df))
    i = i - 3 + int(np.argmax(p[i - 3:i + 4]))          # el pico verdadero, cerca
    amp = ve.amplitud_tono(f, p, i, ve.piso(p, df)[i])
    total = float(p.sum() * df)
    ruido = max(total - amp ** 2, 1e-30)
    return 10 * np.log10(max(amp ** 2, 1e-30) / ruido), amp   # amp = 0: tono bajo el piso
