"""
Pruebas de lectura y analisis de capturas (etapa 4), sin placa.

    python -m unittest test_vibrascan_archivo -v
"""

import json
import os
import tempfile
import time
import unittest
from unittest import mock

import numpy as np

import vibrascan_archivo as va
import vibrascan_proto as vp
from vibrascan_captura import Captura, limites_cuentas, muestra_imposible

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


class PuertoFalso:
    """Lo minimo de serial.Serial para el motor de captura: entrega los bytes dados."""

    def __init__(self, data):
        self.data = data

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, k):
        out, self.data = self.data[:k], self.data[k:]
        return out

    def write(self, _data):
        pass

    def close(self):
        pass


class TestLimites(unittest.TestCase):
    def test_bits_por_rango(self):
        self.assertEqual(limites_cuentas(2), (-512, 511))
        self.assertEqual(limites_cuentas(4), (-1024, 1023))
        self.assertEqual(limites_cuentas(16), (-4096, 4095))

    def test_muestra_imposible(self):
        self.assertTrue(muestra_imposible((4100, 0, 238), 16))      # capturas reales con el motor
        self.assertTrue(muestra_imposible((0, 16384, -7680), 16))
        self.assertFalse(muestra_imposible((4095, -4096, 0), 16))   # el tope es posible
        self.assertTrue(muestra_imposible((512, 0, 0), 2))
        self.assertFalse(muestra_imposible((30000, 0, 0), None))    # rango todavia desconocido


class TestCapturaEnVivo(unittest.TestCase):
    def test_corrupta_se_informa_y_el_csv_la_conserva(self):
        muestras = [(1, 2, 3)] * vp.BLOCK_SAMPLES
        muestras[5] = (0, 16384, -7680)
        flujo = (vp.build_frame(vp.TYPE_STATUS, 0, b"estado=OK etapa=7 odr=3200 rango=16")
                 + vp.build_frame(vp.TYPE_SAMPLES, 1, vp.build_block_payload(100, 0, 3200, vp.FLAG_FIRST, muestras)))
        with tempfile.TemporaryDirectory() as d, \
                mock.patch("vibrascan_captura.vp.open_stream", return_value=PuertoFalso(flujo)):
            base = os.path.join(d, "cap")
            cap = Captura("COMx", base)
            eventos = []
            while cap.ser.in_waiting:
                eventos += cap.poll()[1]
            cap.close()
            self.assertEqual(cap.totals()["muestras_corruptas"], 1)
            texto = [t for k, t in eventos if k == "corrupta"]
            self.assertEqual(len(texto), 1)
            self.assertIn("n=105", texto[0])
            leida = va.cargar(base)
            self.assertEqual(leida.c[5].tolist(), [0, 16384, -7680])
            self.assertEqual(leida.meta["perdidas"]["muestras_corruptas"], 1)
            self.assertTrue(va.corruptas(leida)[5])


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


class TestReposo(unittest.TestCase):
    def eventos(self, tramos):
        """tramos: [(media en cuentas, sacudida en cuentas)] de ~2 s cada uno a 100 Hz."""
        rng = np.random.default_rng(2)
        flujo, seq, n = vp.build_frame(vp.TYPE_STATUS, 0, b"estado=OK etapa=7 odr=100 rango=16"), 1, 0
        for media, ruido in tramos:
            for _ in range(8):
                m = [tuple(int(v) for v in np.round(np.array(media) + rng.normal(0, ruido, 3))) for _ in range(32)]
                flujo += vp.build_frame(vp.TYPE_SAMPLES, seq, vp.build_block_payload(
                    n, n * 10000, 100, vp.FLAG_FIRST if n == 0 else 0, m))
                seq, n = seq + 1, n + 32
        with mock.patch("vibrascan_captura.vp.open_stream", return_value=PuertoFalso(flujo)) as abrir:
            cap = Captura("COMx", eje="y")
            ev = []
            while cap.ser.in_waiting:
                ev += cap.poll()[1]
        self.assertEqual(abrir.call_args[0][3], "y")
        return [t for k, t in ev if k == "reposo"]

    def test_normal_no_avisa(self):
        self.assertEqual(self.eventos([((-13, 258, -23), 2), ((-13, 258, -23), 100), ((-13, 258, -23), 2)]), [])

    def test_offset_corrido_desde_el_principio(self):
        ev = self.eventos([((282, 457, -25), 2)])          # caso real: |a| = 2,1 g
        self.assertEqual(len(ev), 1)
        self.assertIn("REVISAR", ev[0])

    def test_offset_que_cambia_y_vuelve(self):
        # Caso real: x corrido 0,5 g deja |a| en 1,1 g; lo delata el cambio
        ev = self.eventos([((-13, 258, -23), 2), ((115, 258, -23), 2), ((-13, 258, -23), 2)])
        self.assertEqual(len(ev), 2)
        self.assertIn("REVISAR", ev[0])
        self.assertIn("volvio", ev[1])


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

    def test_corrupta_no_es_saturada_ni_golpe(self):
        # Los valores reales de la prueba del motor (+/-16 g): y = 16384 no existe
        n = 4000
        c = reposo(n, self.rng)
        c[2000] = (0, 16384, -7680)
        c[2001] = (520, 16384, 513)
        c[3000, 2] = 4095           # este si es el tope del rango
        escribir(self.base, np.arange(n) / FS, c, rango=16)
        cap = va.cargar(self.base)
        self.assertEqual(np.flatnonzero(va.corruptas(cap)).tolist(), [2000, 2001])
        self.assertEqual(np.flatnonzero(va.saturadas(cap)).tolist(), [3000])
        self.assertEqual(cap.c[2000, 1], 16384)                 # lo crudo no se toca
        self.assertLess(abs(cap.g[2000, 1]), 0.2)               # lo que se analiza, si
        self.assertLess(abs(cap.g[2001, 2] + 262 * 0.0039), 0.1)
        self.assertEqual(len(va.golpes(cap)), 1)                # solo el tope: no hay golpe de 64 g
        self.assertLess(va.golpes(cap)[0].pico_g, 17)

    def test_corrupta_usa_el_rango_mas_grande(self):
        # 2000 cuentas son imposibles a 2 g pero validas a 16 g: si el rango cambio
        # en la captura, no se pueden marcar
        n = 1000
        c = reposo(n, self.rng)
        c[10, 2] = 2000
        escribir(self.base, np.arange(n) / FS, c, rango=2, extra={"segmentos": [{"seg": 0, "rango": 16}]})
        self.assertFalse(va.corruptas(va.cargar(self.base)).any())

    def test_sin_rango_no_marca_corruptas(self):
        n = 100
        c = reposo(n, self.rng)
        c[10, 2] = 30000
        escribir(self.base, np.arange(n) / FS, c, rango=None)
        self.assertFalse(va.corruptas(va.cargar(self.base)).any())

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
