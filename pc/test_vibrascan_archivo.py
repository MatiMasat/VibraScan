"""
Pruebas de lectura y analisis de capturas (etapa 4), sin placa.

    python -m unittest test_vibrascan_archivo -v
"""

import json
import os
import tempfile
import time
import unittest

import numpy as np

import vibrascan_archivo as va
from vibrascan_captura import limites_cuentas

FS = 1e6 / 1258   # tasa real a 800 Hz nominales


def escribir(base, t, c, seg=None, rango=4, extra=None):
    """Escribe una captura sintetica con el mismo formato que vibrascan_captura."""
    seg = np.zeros(len(t), dtype=int) if seg is None else seg
    data = np.column_stack((seg, np.arange(len(t)), t, c))
    np.savetxt(base + ".csv", data, fmt=["%d", "%d", "%.6f", "%d", "%d", "%d"],
               delimiter=",", header="seg,n,t_s,x,y,z", comments="")
    meta = {"rango_g": rango, "escala_g_por_cuenta": 0.0039, "odr_hz": 800,
            "duracion_s": float(t[-1]) if len(t) else 0.0, "completa": True}
    meta.update(extra or {})
    with open(base + ".json", "w") as f:
        json.dump(meta, f)


def reposo(n, rng, ruido=1.3):
    """n muestras quietas con Z hacia abajo (~ -1 g), ruido en cuentas."""
    c = np.column_stack((3 + rng.normal(0, ruido, n), 13 + rng.normal(0, ruido, n),
                         -262 + rng.normal(0, ruido, n)))
    return np.round(c).astype(int)


class TestLimites(unittest.TestCase):
    def test_bits_por_rango(self):
        self.assertEqual(limites_cuentas(2), (-512, 511))
        self.assertEqual(limites_cuentas(4), (-1024, 1023))
        self.assertEqual(limites_cuentas(16), (-4096, 4095))


class TestEnvolvente(unittest.TestCase):
    def test_no_pierde_un_pico_de_una_muestra(self):
        t = np.arange(1_000_000) / FS
        y = np.zeros(len(t))
        y[537_123] = 3.0
        y[812_001] = -2.0
        tx, ty = va.envolvente(t, y, 1500)
        self.assertLessEqual(len(tx), 2 * 1500 + 2)
        self.assertEqual(ty.max(), 3.0)
        self.assertEqual(ty.min(), -2.0)

    def test_pocas_muestras_sin_tocar(self):
        t = np.arange(100.0)
        tx, ty = va.envolvente(t, np.sin(t), 1500)
        self.assertIs(tx, t)


class TestCaptura(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.base = os.path.join(self.dir.name, "cap")
        self.rng = np.random.default_rng(7)

    def tearDown(self):
        self.dir.cleanup()

    def test_30_minutos_carga_y_pico(self):
        n = int(30 * 60 * FS)
        t = np.arange(n) / FS
        c = reposo(n, self.rng)
        c[700_000, 2] = 300          # un solo valor fuera de lugar en 30 min
        escribir(self.base, t, c)
        t0 = time.monotonic()
        cap = va.cargar(self.base + ".csv")
        dt = time.monotonic() - t0
        print(f"\n  carga de {n} filas: {dt:.1f} s")
        self.assertEqual(len(cap.t), n)
        _, ty = va.envolvente(cap.t, cap.g[:, 2], 1500)
        self.assertAlmostEqual(ty.max(), 300 * 0.0039, places=6)
        ventanas = va.por_ventana(cap)
        self.assertEqual(len(ventanas), 29)   # la ultima ventana queda incompleta por el redondeo
        con_pico = int(cap.t[700_000] // 60)
        for w in ventanas:
            self.assertAlmostEqual(w["media_g"][2], -262 * 0.0039, delta=0.001)
            if w["ventana"] == con_pico:
                # Una sola muestra fuera de lugar en 48 000 duplica el rms del minuto:
                # el criterio de reposo la va a marcar, y en que minuto
                self.assertGreater(w["rms_g"][2], 2 * 1.3 * 0.0039)
            else:
                self.assertAlmostEqual(w["rms_g"][2], 1.3 * 0.0039, delta=0.0005)

    def test_golpes(self):
        n = int(20 * FS)
        t = np.arange(n) / FS
        c = reposo(n, self.rng)
        for k, t_g in enumerate((2.0, 4.1, 6.0, 8.2, 10.0)):
            i = int(t_g * FS)
            dt = np.arange(40) / FS
            c[i:i + 40, 2] += np.round(600 * np.exp(-dt / 0.008) * np.sin(2 * np.pi * 120 * dt + 1.2)).astype(int)
        escribir(self.base, t, c)
        gs = va.golpes(va.cargar(self.base))
        self.assertEqual(len(gs), 5)
        for g, t_g in zip(gs, (2.0, 4.1, 6.0, 8.2, 10.0)):
            self.assertAlmostEqual(g.t_s, t_g, delta=0.01)
            self.assertLess(g.dur_ms, 50)
            self.assertIsNotNone(g.vuelta_ms)
            self.assertLess(g.vuelta_ms, 100)
            self.assertFalse(g.saturado)

    def test_saturacion(self):
        n = 2000
        t = np.arange(n) / FS
        c = reposo(n, self.rng)
        c[1000:1003, 2] = -512        # tope de +/-2 g
        escribir(self.base, t, c, rango=2)
        cap = va.cargar(self.base)
        self.assertEqual(int(va.saturadas(cap).sum()), 3)

    def test_tramos_por_segmento(self):
        n = 3000
        seg = np.repeat([0, 1, 2], 1000)
        escribir(self.base, np.arange(n) / FS, reposo(n, self.rng), seg=seg)
        self.assertEqual(va.cargar(self.base).tramos(), [(0, 1000), (1000, 2000), (2000, 3000)])

    def test_nombre_base(self):
        self.assertEqual(va.base_de("capturas/x.csv"), "capturas/x")
        self.assertEqual(va.base_de("capturas/x.json"), "capturas/x")
        self.assertEqual(va.base_de("capturas/x"), "capturas/x")


if __name__ == "__main__":
    unittest.main()
