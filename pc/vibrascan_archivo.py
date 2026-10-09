"""
VibraScan - lectura y analisis de capturas (etapa 4).

Lee los archivos que escribe vibrascan_captura.py (<base>.csv y <base>.json)
y calcula lo que usan ver_captura.py y etapa4_prueba.py: estadisticas por
ventana de tiempo, golpes y muestras saturadas. Una sola implementacion para
los dos: lo que se ve en el grafico es lo mismo que se evalua.
"""

import json
import os
from dataclasses import dataclass

import numpy as np

from vibrascan_captura import G_PER_LSB, RANGOS, limites_cuentas

GOLPE_UMBRAL_G = 0.5      # |a| se aparta del reposo mas que esto = golpe
GOLPE_SEPARACION_S = 0.2  # cruces del umbral mas cercanos que esto son el mismo golpe
REPOSO_TOL_G = 0.1        # "volvio al reposo": |a| dentro de esto ...
REPOSO_FIRME_S = 0.05     # ... durante este tiempo seguido


def base_de(path):
    """'capturas/x.csv', 'capturas/x.json' o 'capturas/x' -> 'capturas/x'."""
    root, ext = os.path.splitext(path)
    return root if ext.lower() in (".csv", ".json") else path


@dataclass
class Captura:
    base: str
    meta: dict
    seg: np.ndarray     # int, segmento de cada muestra
    n: np.ndarray       # int, indice del firmware
    t: np.ndarray       # float, s
    c: np.ndarray       # int (N, 3), cuentas x, y, z

    @property
    def escala(self):
        return self.meta.get("escala_g_por_cuenta", G_PER_LSB)

    @property
    def g(self):
        return self.c * self.escala

    @property
    def modulo(self):
        g = self.g
        return np.sqrt((g * g).sum(axis=1))

    def tramos(self):
        """[(inicio, fin)] de indices de cada segmento (fin excluido)."""
        if len(self.seg) == 0:
            return []
        cuts = np.flatnonzero(np.diff(self.seg)) + 1
        starts = np.concatenate(([0], cuts))
        ends = np.concatenate((cuts, [len(self.seg)]))
        return list(zip(starts.tolist(), ends.tolist()))

    def limites(self):
        r = self.meta.get("rango_g")
        return limites_cuentas(r) if r in RANGOS else None


def cargar(path):
    base = base_de(path)
    with open(base + ".json", encoding="utf-8") as f:
        meta = json.load(f)
    data = np.loadtxt(base + ".csv", delimiter=",", skiprows=1, ndmin=2)
    if data.size == 0:
        data = np.zeros((0, 6))
    cap = Captura(base, meta, data[:, 0].astype(np.int32), data[:, 1].astype(np.int64),
                  data[:, 2], data[:, 3:6].astype(np.int32))
    # Salida del filtro adaptativo del micro (etapa 7), si la hay
    cap.filtro = None
    if os.path.exists(base + "_filtro.csv"):
        f = np.loadtxt(base + "_filtro.csv", delimiter=",", skiprows=1, ndmin=2)
        if f.size:
            cap.filtro = {"n": f[:, 0].astype(np.int64), "y": f[:, 1], "e": f[:, 2],
                          "reinicio": f[:, 3].astype(bool)}
    return cap


def saturadas(cap):
    """Mascara de muestras con algun eje en el tope del rango (None si no se conoce el rango)."""
    lim = cap.limites()
    if lim is None:
        return None
    return ((cap.c <= lim[0]) | (cap.c >= lim[1])).any(axis=1)


def por_ventana(cap, ventana_s=60.0):
    """Media y rms (desvio) por eje en g, por ventana de tiempo completa.
    Devuelve una lista de dicts; la ultima ventana incompleta se descarta."""
    out = []
    if len(cap.t) == 0:
        return out
    g, mod = cap.g, cap.modulo
    edges = np.arange(cap.t[0], cap.t[-1] - ventana_s + 1e-9, ventana_s)
    for k, t0 in enumerate(edges):
        i0, i1 = np.searchsorted(cap.t, [t0, t0 + ventana_s])
        if i1 - i0 < 2:
            continue
        w = g[i0:i1]
        out.append({"ventana": k, "t0_s": float(t0), "muestras": int(i1 - i0),
                    "media_g": w.mean(axis=0).tolist(), "rms_g": w.std(axis=0).tolist(),
                    "modulo_g": float(mod[i0:i1].mean())})
    return out


@dataclass
class Golpe:
    t_s: float          # primer cruce del umbral
    pico_g: float       # |a| mas alejado del reposo
    dur_ms: float       # del primer al ultimo cruce del umbral
    vuelta_ms: float    # del ultimo cruce hasta quedar firme en reposo (None si no volvio)
    saturado: bool
    t_pico_s: float = None  # instante del pico


def golpes(cap, umbral_g=GOLPE_UMBRAL_G):
    """Golpes: tramos donde |a| se aparta del reposo (mediana de |a|) mas que el umbral."""
    if len(cap.t) == 0:
        return []
    mod = cap.modulo
    reposo = float(np.median(mod))
    dev = np.abs(mod - reposo)
    idx = np.flatnonzero(dev > umbral_g)
    if len(idx) == 0:
        return []
    sat = saturadas(cap)
    # Cruces separados por mas de GOLPE_SEPARACION_S son golpes distintos
    splits = np.flatnonzero(np.diff(cap.t[idx]) > GOLPE_SEPARACION_S) + 1
    out = []
    for grupo in np.split(idx, splits):
        i0, i1 = int(grupo[0]), int(grupo[-1])
        k = i0 + int(np.argmax(dev[i0:i1 + 1]))
        out.append(Golpe(float(cap.t[i0]), float(mod[k]), (cap.t[i1] - cap.t[i0]) * 1000,
                         _vuelta_ms(cap.t, dev, i1),
                         bool(sat[i0:i1 + 1].any()) if sat is not None else False,
                         float(cap.t[k])))
    return out


def _vuelta_ms(t, dev, i1, busqueda_s=2.0):
    """ms desde el ultimo cruce del umbral (i1) hasta el primer instante desde
    el que |a| queda REPOSO_FIRME_S seguidos dentro de REPOSO_TOL_G del reposo.
    None si no pasa dentro de busqueda_s."""
    j_end = int(np.searchsorted(t, t[i1] + busqueda_s))
    quiet = (dev[i1 + 1:j_end] <= REPOSO_TOL_G).astype(np.int32)
    dt = np.median(np.diff(t[max(0, i1 - 100):i1 + 1])) if i1 > 0 else 0.00125
    L = max(1, int(round(REPOSO_FIRME_S / dt)))
    if len(quiet) < L:
        return None
    firm = np.flatnonzero(np.convolve(quiet, np.ones(L, dtype=np.int32), mode="valid") == L)
    return float((t[i1 + 1 + firm[0]] - t[i1]) * 1000) if len(firm) else None


def envolvente(t, y, ancho_px):
    """Puntos para dibujar y(t) en ancho_px columnas sin perder picos: si hay mas
    muestras que columnas, cada grupo se dibuja como su minimo y su maximo."""
    k = len(t)
    if k <= 2 * ancho_px:
        return t, y
    m = int(np.ceil(k / ancho_px))
    usable = (k // m) * m
    tt = t[:usable].reshape(-1, m)
    yy = y[:usable].reshape(-1, m)
    tc = np.repeat(tt[:, m // 2], 2)
    yc = np.column_stack((yy.min(axis=1), yy.max(axis=1))).ravel()
    if usable < k:   # resto que no completa un grupo
        tc = np.concatenate((tc, [t[usable], t[-1]]))
        yc = np.concatenate((yc, [y[usable:].min(), y[usable:].max()]))
    return tc, yc
