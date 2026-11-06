"""
VibraScan - velocidad de un motor a partir de su vibracion (etapa 7).

En el espectro de un motor de induccion con el sensor sobre la carcasa aparecen
(ver docs/prueba_motor.md):

    linea electrica  f_L = 2 x la frecuencia de alimentacion (100 Hz con la red
                     de 50 Hz) y sus armonicas: k f_L
    bandas laterales k f_L +/- m f_1, con f_1 la velocidad del rotor (1x)

La velocidad se mide con las bandas laterales, no con el pico de 1x, que suele
ser mucho mas debil: f_1 es la separacion del peine de bandas respecto de la
linea. Se calcula en ventanas cortas para seguirla en el tiempo, cada una con
su tasa de muestreo local (el reloj del sensor deriva con la temperatura).

La linea puede estar fija en 2 x red (variar la tension no la mueve) o moverse
con la velocidad (variador de frecuencia): en ese caso se busca en cada ventana.

Si en una ventana no hay bandas suficientes, su velocidad queda en NaN: no se
inventa un valor.
"""

from dataclasses import dataclass

import numpy as np

import vibrascan_archivo as va
import vibrascan_espectro as ve

VENTANA_S = 4.0         # ventana de cada estimacion: resolucion 0,25 Hz (Hann: ~0,5 Hz)
PASO_S = 2.0
RELLENO = 2             # relleno de ceros de la FFT, para interpolar mejor los picos
FMAX_HZ = 600.0         # el peine se busca hasta aca
ARMONICAS = 5           # armonicas de la linea que se usan
LINEA_TOL_HZ = 1.0      # linea fija: se busca a +/- esto de 2 x red
LINEA_MIN_HZ = 10.0     # linea libre (variador): se busca desde aca ...
LINEA_MAX_HZ = 200.0    # ... hasta aca
LINEA_MIN_DB = 15.0     # sobre el piso local, para aceptar la linea
BANDA_MIN_DB = 10.0     # sobre el piso local, para aceptar una banda lateral
BANDA_TOL_HZ = 0.4      # una banda se busca a +/- esto de donde deberia estar
BANDAS_MIN = 3          # bandas que tiene que haber para aceptar una velocidad
TOPE_DB = 30.0          # en el puntaje: una linea muy fuerte no vale mas que esto
F1_MIN_HZ = 3.0
F1_MIN_FRAC = 0.2       # velocidad minima que se busca, fraccion de la maxima
MARCHA_UMBRAL_G = 0.1   # rms (suma de ejes) por encima de esto = motor prendido
MARCHA_BLOQUE_S = 0.25
MARCHA_MIN_S = 5.0      # mas corto no es una marcha (un golpe, un toque)
MARCHA_UNIR_S = 1.0     # pausas mas cortas no la cortan
MARGEN_S = 1.0          # no se analiza este tiempo al principio y al final de una marcha
ESCALON_TOL_RPM = 20.0  # una ventana pertenece al escalon si esta a menos de esto de su mediana
ESCALON_MIN = 3         # ventanas seguidas para que sea un escalon
LINEA_NPERSEG = 2048    # para comparar la linea con el motor prendido y apagado


def rpm_de(f_hz):
    return f_hz * 60.0


def polos_de_nominal(rpm_nominal, red_hz):
    """Polos de un motor de induccion a partir de su velocidad nominal: la
    velocidad sincronica mas baja que queda por encima de ella."""
    for polos in (2, 4, 6, 8, 10, 12):
        if 120.0 * red_hz / polos < rpm_nominal:
            return polos - 2 if polos > 2 else None
    return 12


def marchas(cap):
    """Tramos con el motor prendido: [(t0, t1)]. Se mira el rms de cada bloque
    corto sin su media, asi que inclinar el sensor no cuenta."""
    if len(cap.t) < 2:
        return []
    g = cap.g
    fs = (len(cap.t) - 1) / (cap.t[-1] - cap.t[0])
    k = max(1, int(MARCHA_BLOQUE_S * fs))
    m = len(g) // k
    if m == 0:
        return []
    bloques = g[:m * k].reshape(m, k, 3)
    rms = np.sqrt(bloques.var(axis=1).sum(axis=1))
    t = cap.t[:m * k:k]
    prendido = rms > MARCHA_UMBRAL_G
    tramos = []
    for i in np.flatnonzero(prendido):
        t0, t1 = float(t[i]), float(t[i] + k / fs)
        if tramos and t0 - tramos[-1][1] <= MARCHA_UNIR_S:
            tramos[-1][1] = t1
        else:
            tramos.append([t0, t1])
    return [(a, b) for a, b in tramos if b - a >= MARCHA_MIN_S]


def reposos(cap, tramos):
    """Tramos con el motor apagado (lo que queda entre las marchas), con
    MARGEN_S de margen a cada lado."""
    if len(cap.t) == 0:
        return []
    bordes = [float(cap.t[0]) - MARGEN_S] + [v for tr in tramos for v in tr] + [float(cap.t[-1]) + MARGEN_S]
    pares = zip(bordes[::2], bordes[1::2])
    return [(a + MARGEN_S, b - MARGEN_S) for a, b in pares if b - a > 2 * MARGEN_S]


@dataclass
class Espectro:
    f: np.ndarray       # Hz, hasta FMAX_HZ
    p: np.ndarray       # suma de los tres ejes, g^2/Hz
    db: np.ndarray      # dB sobre el piso local
    fs: float           # tasa local

    def db_en(self, f):
        """dB sobre el piso en las frecuencias f, limitado a [0, TOPE_DB]. Fuera
        de la banda, 0."""
        return np.clip(np.interp(f, self.f, self.db, left=0.0, right=0.0), 0.0, TOPE_DB)

    def pico(self, f0, tol):
        """(frecuencia interpolada, dB sobre el piso) del maximo en f0 +/- tol, si
        es un maximo local de verdad y no el flanco de otra linea; si no, None."""
        i0, i1 = np.searchsorted(self.f, [f0 - tol, f0 + tol])
        i0, i1 = max(1, int(i0)), min(len(self.f) - 1, int(i1))
        if i1 - i0 < 1:
            return None
        i = i0 + int(np.argmax(self.p[i0:i1]))
        if self.p[i] < self.p[i - 1] or self.p[i] < self.p[i + 1]:
            return None
        return ve._f_interp(self.f, self.p, i), float(self.db[i])


def espectro_ventana(g, t, fs_ref):
    """Espectro de una ventana (g: N x 3, t: N), con su tasa local. Hann, sin media."""
    n = len(g)
    fs = float(ve.tasas_tramos(t, fs_ref, n)[0])
    w = ve.hann(n)
    x = (g - g.mean(axis=0)) * w[:, None]
    nfft = RELLENO * n
    f = np.fft.rfftfreq(nfft, 1 / fs)
    banda = f <= FMAX_HZ
    p = (np.abs(np.fft.rfft(x, nfft, axis=0)[banda]) ** 2).sum(axis=1) * 2 / (fs * (w * w).sum())
    fb = f[banda]
    p = np.maximum(p, 1e-30)
    return Espectro(fb, p, 10 * np.log10(p / np.maximum(ve.piso(p, fb[1] - fb[0]), 1e-30)), fs)


def linea(esp, f_fija=None):
    """(f_L, dB) de la linea electrica en esta ventana, o (NaN, NaN). Con f_fija la
    busca alrededor de ella; si no, la que mejor explica un peine de armonicas."""
    if f_fija is None:
        df = esp.f[1] - esp.f[0]
        cand = np.arange(LINEA_MIN_HZ, min(LINEA_MAX_HZ, FMAX_HZ / 2), df)
        puntaje = sum(esp.db_en(k * cand) for k in range(1, 5))
        f0, tol = float(cand[np.argmax(puntaje)]), 2 * df
    else:
        f0, tol = f_fija, LINEA_TOL_HZ
    pk = esp.pico(f0, tol)
    if pk is None or pk[1] < LINEA_MIN_DB:
        return float("nan"), float("nan")
    return pk


def velocidad(esp, f_l, f1_max):
    """(f_1, bandas encontradas) a partir del peine k f_L +/- m f_1, o (NaN, bandas).

    Primero se elige la separacion que mejor explica el espectro. Contar tambien
    las bandas a 2 f_1 (con la mitad del peso) evita que gane la mitad o el
    doble de la separacion real. Despues se afina con la posicion de cada banda."""
    if not np.isfinite(f_l):
        return float("nan"), 0
    f1_min = max(F1_MIN_HZ, F1_MIN_FRAC * f1_max)
    cand = np.arange(f1_min, f1_max, esp.f[1] - esp.f[0])
    if len(cand) == 0:
        return float("nan"), 0
    ks = [k for k in range(1, ARMONICAS + 1) if k * f_l < FMAX_HZ]
    puntaje = np.zeros(len(cand))
    for k in ks:
        for m, peso in ((1, 1.0), (2, 0.5)):
            for signo in (1, -1):
                puntaje += peso * esp.db_en(k * f_l + signo * m * cand)
    f1 = float(cand[np.argmax(puntaje)])

    medidas = []
    for k in ks:
        for m in (1, 2):
            for signo in (1, -1):
                pk = esp.pico(k * f_l + signo * m * f1, BANDA_TOL_HZ)
                if pk is not None and pk[1] >= BANDA_MIN_DB:
                    medidas.append((abs(pk[0] - k * f_l) / m, pk[1]))
    if len(medidas) < BANDAS_MIN:
        return float("nan"), len(medidas)
    v, db = np.array(medidas).T
    return float(np.average(v, weights=db)), len(medidas)


@dataclass
class Seguimiento:
    t: np.ndarray           # s, centro de cada ventana
    f_linea: np.ndarray     # Hz (NaN si no se encontro)
    db_linea: np.ndarray    # dB sobre el piso local
    f1: np.ndarray          # Hz, velocidad del rotor (NaN si no se pudo medir)
    bandas: np.ndarray      # bandas laterales encontradas
    rms_g: np.ndarray       # rms de la ventana, suma de los ejes
    fs: np.ndarray          # tasa local de cada ventana


def seguir(cap, tramos, f_linea_fija=None, polos=None):
    """Sigue la linea y la velocidad en las marchas, ventana por ventana. La
    velocidad se busca por debajo de la sincronica (f_L / polos) si se conocen
    los polos, con un margen para poder ver si la supera; si no, hasta f_L / 2."""
    fs_ref = 1e6 / cap.meta["periodo_us"] if cap.meta.get("periodo_us") else float(cap.meta.get("odr_hz") or 800)
    g = cap.g
    filas = []
    for a, b in tramos:
        for t0 in np.arange(a + MARGEN_S, b - MARGEN_S - VENTANA_S + 1e-9, PASO_S):
            i0, i1 = np.searchsorted(cap.t, [t0, t0 + VENTANA_S])
            if i1 - i0 < 16 or cap.seg[i0] != cap.seg[i1 - 1]:
                continue                # no se cruza un hueco
            esp = espectro_ventana(g[i0:i1], cap.t[i0:i1], fs_ref)
            f_l, db_l = linea(esp, f_linea_fija)
            f1, bandas = (float("nan"), 0) if not np.isfinite(f_l) else \
                velocidad(esp, f_l, 1.03 * f_l / polos if polos else f_l / 2)
            rms = float(np.sqrt(g[i0:i1].var(axis=0).sum()))
            filas.append((t0 + VENTANA_S / 2, f_l, db_l, f1, bandas, rms, esp.fs))
    a = np.array(filas, dtype=float).reshape(-1, 7)
    return Seguimiento(*a.T)


@dataclass
class Escalon:
    t0: float
    t1: float
    f1: float       # Hz, mediana
    f_linea: float  # Hz, mediana
    rms_g: float    # mediana
    ventanas: int


def escalones(seg):
    """Tramos de velocidad estable: ventanas seguidas (las que no se pudieron medir
    cortan el tramo) cerca de la mediana del tramo, al menos ESCALON_MIN."""
    tol = ESCALON_TOL_RPM / 60.0
    salto = 2.5 * PASO_S
    grupos, actual = [], []
    for i in range(len(seg.t)):
        if not np.isfinite(seg.f1[i]):
            continue
        junto = actual and seg.t[i] - seg.t[actual[-1]] <= salto
        if junto and abs(seg.f1[i] - np.median(seg.f1[actual])) <= tol:
            actual.append(i)
        else:
            grupos.append(actual)
            actual = [i]
    grupos.append(actual)
    return [Escalon(float(seg.t[g[0]] - VENTANA_S / 2), float(seg.t[g[-1]] + VENTANA_S / 2),
                    float(np.median(seg.f1[g])), float(np.nanmedian(seg.f_linea[g])),
                    float(np.median(seg.rms_g[g])), len(g))
            for g in grupos if len(g) >= ESCALON_MIN]


def linea_en_psd(cap, tramo, f0, tol=1.0):
    """PSD (suma de ejes) del tramo en su maximo a f0 +/- tol, en g^2/Hz. None si el
    tramo es demasiado corto."""
    try:
        esp = ve.psd(cap, LINEA_NPERSEG, tramo[0], tramo[1])
    except ValueError:
        return None
    i = np.abs(esp.f - f0) <= tol
    return float(esp.psd[3][i].max()) if i.any() else None


def fuera_de_transitorios(cap, tramos):
    """Mascara de las muestras de las marchas sin el arranque y el apagado."""
    m = np.zeros(len(cap.t), dtype=bool)
    for a, b in tramos:
        m |= (cap.t >= a + MARGEN_S) & (cap.t <= b - MARGEN_S)
    return m


def saturadas_en_marcha(cap, tramos):
    """(muestras saturadas, total) dentro de las marchas, sin arranque ni apagado."""
    m = fuera_de_transitorios(cap, tramos)
    sat = va.saturadas(cap)
    return (int((sat & m).sum()) if sat is not None else 0), int(m.sum())
