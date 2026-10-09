"""
VibraScan - espectro de una captura (etapa 5).

Densidad espectral de potencia (PSD) por el metodo de Welch, escrito con
numpy para que la etapa 7 (FFT en el micro) lo pueda replicar paso a paso.
Convenciones (las mismas que tendra que usar el micro):

    ventana     Hann periodica: w[k] = 0,5 - 0,5 cos(2 pi k / n), k = 0..n-1
    tramos      n = NPERSEG muestras, solapados al 50 %
    preproceso  a cada tramo se le resta su media (saca la gravedad y la deriva lenta)
    escala      densidad de una cara: P = 2 |FFT(x w)|^2 / (fs sum(w^2)), sin
                duplicar la componente continua ni la de Nyquist. Unidades g^2/Hz.
                Parseval: la integral de P es la varianza de la senal.

Se calcula dentro de cada segmento de la captura (nunca a traves de un hueco)
y con la tasa medida de cada segmento, no con el ODR nominal.
"""

from dataclasses import dataclass

import numpy as np

NPERSEG = 4096          # 5,1 s a ~796 Hz: resolucion 0,19 Hz
SOLAPE = 0.5
PICO_BINS = 3           # la amplitud de un tono se integra en +/-3 bins
PICO_MIN_DB = 10.0      # un pico tiene que superar al piso local en esto
PISO_VENTANA_HZ = 20.0  # el piso local es la mediana de la PSD en +/- esto
EJES = ("x", "y", "z")


def hann(n):
    """Ventana de Hann periodica (la de scipy por defecto y la del micro en la etapa 7)."""
    return 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(n) / n)


def welch(x, fs, nperseg=NPERSEG, solape=SOLAPE):
    """PSD de una senal equiespaciada. Devuelve (f, P, tramos) o None si es mas
    corta que un tramo. P es de una cara, en unidades^2/Hz."""
    x = np.asarray(x, dtype=float)
    if len(x) < nperseg:
        return None
    step = nperseg - int(nperseg * solape)
    frames = np.lib.stride_tricks.sliding_window_view(x, nperseg)[::step]
    frames = frames - frames.mean(axis=1, keepdims=True)
    w = hann(nperseg)
    spec = np.abs(np.fft.rfft(frames * w, axis=1)) ** 2
    p = spec.mean(axis=0) / (fs * (w * w).sum())
    if nperseg % 2 == 0:
        p[1:-1] *= 2        # Nyquist (ultimo bin) aparece una sola vez
    else:
        p[1:] *= 2
    return np.fft.rfftfreq(nperseg, 1 / fs), p, len(frames)


@dataclass
class Espectro:
    f: np.ndarray           # Hz
    psd: np.ndarray         # (4, F): x, y, z y la suma, en g^2/Hz
    fs: float               # tasa de la grilla de frecuencias (segmento mas largo)
    nperseg: int
    tramos: int             # tramos de Welch promediados
    segmentos: list         # numeros de segmento usados
    varianza: np.ndarray    # (3,) varianza de cada eje (tramos sin media), para Parseval

    @property
    def df(self):
        return self.f[1] - self.f[0]

    def parseval(self):
        """Integral de la PSD / varianza, por eje (deberia dar ~1)."""
        return self.psd[:3].sum(axis=1) * self.df / self.varianza


def _fs_segmento(cap, seg, i0, i1):
    for s in cap.meta.get("segmentos", []):
        if s.get("seg") == seg and s.get("periodo_us"):
            return 1e6 / s["periodo_us"]
    if cap.meta.get("periodo_us"):
        return 1e6 / cap.meta["periodo_us"]
    return (i1 - i0 - 1) / (cap.t[i1 - 1] - cap.t[i0])      # ultimo recurso: el reloj de t_s


def psd(cap, nperseg=NPERSEG, desde=None, hasta=None):
    """PSD de una captura (vibrascan_archivo.Captura), por segmento y promediada."""
    t0 = cap.t[0] if desde is None else desde
    t1 = cap.t[-1] if hasta is None else hasta
    partes = []
    for i0, i1 in cap.tramos():
        seg = int(cap.seg[i0])
        fs = _fs_segmento(cap, seg, i0, i1)
        j0, j1 = np.searchsorted(cap.t[i0:i1], [t0, t1 + 1e-9]) + i0
        if j1 - j0 < nperseg:
            continue
        odr = next((s.get("odr") for s in cap.meta.get("segmentos", []) if s.get("seg") == seg), None)
        g = cap.g[j0:j1]
        res = [welch(g[:, a], fs, nperseg) for a in range(3)]
        partes.append((seg, odr, fs, res, g))
    if not partes:
        raise ValueError(f"ningun segmento tiene {nperseg} muestras en el tramo pedido")
    odrs = {p[1] for p in partes if p[1] is not None}
    if len(odrs) > 1:
        raise ValueError(f"el tramo mezcla ODR {sorted(odrs)}: elegir un tramo con un solo ODR (desde/hasta)")

    # Grilla del segmento con mas tramos; los demas se interpolan sobre ella
    ref = max(partes, key=lambda p: p[3][0][2])
    f = ref[3][0][0]
    acc = np.zeros((3, len(f)))
    var = np.zeros(3)
    total = 0
    for _, _, _, res, g in partes:
        m = res[0][2]
        for a in range(3):
            acc[a] += m * np.interp(f, res[a][0], res[a][1])
        var += m * _varianza_tramos(g, nperseg)
        total += m
    p = acc / total
    return Espectro(f, np.vstack((p, p.sum(axis=0))), ref[2], nperseg, total,
                    [s for s, *_ in partes], var / total)


def _varianza_tramos(g, nperseg):
    """Varianza media de los tramos de Welch (cada uno sin su media), por eje."""
    step = nperseg - int(nperseg * SOLAPE)
    out = np.zeros(3)
    for a in range(3):
        fr = np.lib.stride_tricks.sliding_window_view(g[:, a], nperseg)[::step]
        out[a] = fr.var(axis=1).mean()
    return out


def piso(p, df, ventana_hz=PISO_VENTANA_HZ):
    """Piso de ruido local: mediana de la PSD en +/- ventana_hz alrededor de cada bin."""
    h = max(1, int(round(ventana_hz / df)))
    pad = np.pad(p, h, mode="edge")
    return np.median(np.lib.stride_tricks.sliding_window_view(pad, 2 * h + 1), axis=1)


@dataclass
class Pico:
    f: float        # Hz, interpolado entre bins
    db: float       # sobre el piso local
    amp_g: float    # amplitud rms del tono, en g
    i: int          # bin del maximo


def picos(f, p, min_db=PICO_MIN_DB, fmin=1.0):
    """Picos de una PSD que superan al piso local en min_db, del mas fuerte al mas debil."""
    df = f[1] - f[0]
    fl = piso(p, df)
    ratio = p / np.maximum(fl, 1e-30)
    cand = np.flatnonzero((p[1:-1] > p[:-2]) & (p[1:-1] >= p[2:])) + 1
    cand = cand[(f[cand] >= fmin) & (ratio[cand] >= 10 ** (min_db / 10))]
    out = []
    for i in sorted(cand, key=lambda k: -p[k]):
        if any(abs(i - q.i) <= PICO_BINS for q in out):
            continue        # mismo pico (lobulo de la ventana)
        out.append(Pico(_f_interp(f, p, i), float(10 * np.log10(ratio[i])), amplitud_tono(f, p, i, fl[i]), int(i)))
    return out


def _f_interp(f, p, i):
    """Frecuencia del maximo por interpolacion parabolica del log de la PSD en 3 bins."""
    if i <= 0 or i >= len(p) - 1:
        return float(f[i])
    a, b, c = np.log(p[i - 1:i + 2])
    den = a - 2 * b + c
    d = 0.5 * (a - c) / den if den != 0 else 0.0
    return float(f[i] + d * (f[1] - f[0]))


def amplitud_tono(f, p, i, piso_i=0.0):
    """Amplitud rms (g) de un tono en el bin i: raiz de la integral de la PSD en
    +/-PICO_BINS, descontando el piso. No depende de la ventana ni de si el tono
    cae justo en un bin."""
    lo, hi = max(0, i - PICO_BINS), min(len(p), i + PICO_BINS + 1)
    energia = p[lo:hi].sum() - piso_i * (hi - lo)
    return float(np.sqrt(max(energia, 0.0) * (f[1] - f[0])))


def plegar(f, fs):
    """Frecuencia en la que aparece f muestreando a fs (aliasing): entre 0 y fs/2."""
    r = f % fs
    return min(r, fs - r)


def armonica_de(pk, f0, fs, df, kmax=6):
    """Si pk coincide (a menos de 2 bins) con la armonica k de f0, directa o
    reflejada por aliasing, devuelve (k, reflejada); si no, None."""
    for k in range(2, kmax + 1):
        fk = k * f0
        if abs(plegar(fk, fs) - pk.f) <= 2 * df:
            return k, fk > fs / 2
    return None


ESPECTROGRAMA_TRAMO_S = 0.32    # duracion de cada tramo: ~3,1 Hz de resolucion en cualquier ODR
                                # (256 muestras a 800 Hz, 1024 a 3200 Hz)
ESPECTROGRAMA_COLUMNAS = 4000   # en capturas largas el paso crece para no pasarse


def espectrograma_nperseg(fs):
    """Muestras por tramo del espectrograma: potencia de 2 que mas se acerca a
    ESPECTROGRAMA_TRAMO_S (misma resolucion en tiempo y frecuencia a cualquier ODR)."""
    return int(2 ** round(np.log2(fs * ESPECTROGRAMA_TRAMO_S)))


def espectrograma(cap, eje=3, nperseg=None, desde=None, hasta=None):
    """PSD en funcion del tiempo. eje 0..2 = x, y, z; 3 = suma. Devuelve
    (t, f, P) con P de forma (F, T) en g^2/Hz; las columnas sin datos (huecos
    entre segmentos) quedan en NaN."""
    t0 = cap.t[0] if desde is None else desde
    t1 = cap.t[-1] if hasta is None else hasta
    fs_ref = 1e6 / cap.meta["periodo_us"] if cap.meta.get("periodo_us") else 800.0
    nperseg = nperseg or espectrograma_nperseg(fs_ref)
    # Paso de un cuarto de tramo; en capturas largas crece para no pasar de
    # ESPECTROGRAMA_COLUMNAS columnas
    paso = max(nperseg // 4, int(np.ceil((t1 - t0) * fs_ref / ESPECTROGRAMA_COLUMNAS)))
    dt = paso / fs_ref
    cols = int((t1 - t0) / dt) + 1
    f = np.fft.rfftfreq(nperseg, 1 / fs_ref)
    out = np.full((len(f), cols), np.nan)
    w = hann(nperseg)
    ejes = range(3) if eje == 3 else [eje]
    for i0, i1 in cap.tramos():
        j0, j1 = np.searchsorted(cap.t[i0:i1], [t0, t1 + 1e-9]) + i0
        if j1 - j0 < nperseg:
            continue
        fs = _fs_segmento(cap, int(cap.seg[i0]), i0, i1)
        g = cap.g[j0:j1]
        starts = np.arange(0, j1 - j0 - nperseg + 1, paso)
        acc = 0.0
        for a in ejes:
            fr = np.lib.stride_tricks.sliding_window_view(g[:, a], nperseg)[starts]
            fr = fr - fr.mean(axis=1, keepdims=True)
            p = np.abs(np.fft.rfft(fr * w, axis=1)) ** 2 / (fs * (w * w).sum())
            p[:, 1:-1] *= 2
            acc = acc + p
        f_seg = np.fft.rfftfreq(nperseg, 1 / fs)
        tc = cap.t[j0 + starts + nperseg // 2]
        k = np.clip(((tc - t0) / dt).round().astype(int), 0, cols - 1)
        for col, row in zip(k, acc):
            out[:, col] = np.interp(f, f_seg, row)
    return t0 + np.arange(cols) * dt, f, out
