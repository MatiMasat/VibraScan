"""
Pruebas del espectro (etapa 5), sin placa. Usan senales sinteticas cuyas
propiedades se conocen de antemano (Parseval, tonos de frecuencia y amplitud
dadas), no el propio codigo como referencia.

    python -m unittest test_vibrascan_espectro -v
"""

import os
import tempfile
import unittest

import numpy as np

import vibrascan_archivo as va
import vibrascan_espectro as ve
from test_vibrascan_archivo import escribir

FS = 795.01       # tasa real medida en la etapa 5 (no 800)


def tono(n, f, amp_rms, fs=FS, fase=0.3):
    return amp_rms * np.sqrt(2) * np.sin(2 * np.pi * f * np.arange(n) / fs + fase)


class TestWelch(unittest.TestCase):
    def test_hann_periodica(self):
        w = ve.hann(8)
        self.assertEqual(w[0], 0.0)
        self.assertAlmostEqual(w[4], 1.0)
        self.assertNotEqual(w[-1], 0.0)          # la simetrica terminaria en 0

    def test_parseval_ruido_blanco(self):
        rng = np.random.default_rng(1)
        x = rng.normal(0, 0.006, 60 * 800)      # ~6 mg rms, como el reposo real
        f, p, m = ve.welch(x, FS)
        self.assertAlmostEqual(p.sum() * (f[1] - f[0]) / x.var(), 1.0, delta=0.02)

    def test_ruido_blanco_es_plano(self):
        rng = np.random.default_rng(2)
        sigma = 0.006
        f, p, _ = ve.welch(rng.normal(0, sigma, 120 * 800), FS)
        esperado = 2 * sigma ** 2 / FS             # densidad de una cara del ruido blanco
        self.assertAlmostEqual(np.median(p[10:-10]) / esperado, 1.0, delta=0.1)

    def test_tono_frecuencia_y_amplitud(self):
        rng = np.random.default_rng(3)
        n = 30 * 800
        for f_t in (37.0, 123.4, 249.98, 350.0):
            for desplazamiento in (0.0, 0.5):       # justo en un bin y entre dos bins
                df = FS / ve.NPERSEG
                f_real = (round(f_t / df) + desplazamiento) * df
                x = tono(n, f_real, 0.005) + rng.normal(0, 0.006, n)
                f, p, _ = ve.welch(x, FS)
                pk = ve.picos(f, p)[0]
                self.assertAlmostEqual(pk.f, f_real, delta=0.03, msg=f"{f_real:.3f} Hz")
                self.assertAlmostEqual(pk.amp_g, 0.005, delta=0.005 * 0.05, msg=f"{f_real:.3f} Hz")
                self.assertGreater(pk.db, 20)

    def test_amplitud_exacta_sin_ruido(self):
        # Sin ruido el unico error es el del metodo: tono justo entre dos bins
        # (el peor caso). Integrando +/-3 bins el error es ~0,004 %; con +/-1
        # seria 1 %: esta prueba distingue las dos cosas, la de arriba (con
        # ruido y 5 % de tolerancia) no.
        df = FS / ve.NPERSEG
        f_t = (round(200 / df) + 0.5) * df
        f, p, _ = ve.welch(tono(30 * 800, f_t, 0.005), FS)
        pk = ve.picos(f, p)[0]
        self.assertAlmostEqual(pk.amp_g / 0.005, 1.0, delta=0.002)

    def test_cantidad_de_tramos(self):
        # Convencion que la etapa 7 tiene que copiar: tramos de NPERSEG solapados al 50 %
        n = ve.NPERSEG
        self.assertEqual(ve.welch(np.zeros(n), FS)[2], 1)
        self.assertEqual(ve.welch(np.zeros(n + n // 2 - 1), FS)[2], 1)
        self.assertEqual(ve.welch(np.zeros(n + n // 2), FS)[2], 2)
        self.assertEqual(ve.welch(np.zeros(3 * n), FS)[2], 5)

    def test_dos_tonos_a_1_hz(self):
        n = 60 * 800
        x = tono(n, 200.0, 0.004) + tono(n, 201.0, 0.004, fase=1.1)
        f, p, _ = ve.welch(x, FS)
        fs_ = sorted(pk.f for pk in ve.picos(f, p)[:2])
        self.assertAlmostEqual(fs_[0], 200.0, delta=0.05)
        self.assertAlmostEqual(fs_[1], 201.0, delta=0.05)

    def test_tasa_nominal_corre_el_pico(self):
        # Con 800 Hz nominales en vez de la tasa medida el pico se corre ~1,6 Hz
        n = 30 * 800
        x = tono(n, 250.0, 0.01)
        f, p, _ = ve.welch(x, 800.0)
        self.assertAlmostEqual(ve.picos(f, p)[0].f, 250.0 * 800.0 / FS, delta=0.05)

    def test_corto(self):
        self.assertIsNone(ve.welch(np.zeros(100), FS))

    def test_contra_scipy(self):
        try:
            from scipy import signal
        except ImportError:
            self.skipTest("scipy no esta instalado")
        rng = np.random.default_rng(4)
        x = tono(20000, 123.4, 0.005) + rng.normal(0, 0.006, 20000)
        f, p, _ = ve.welch(x, FS)
        fs_, ps = signal.welch(x, FS, window="hann", nperseg=ve.NPERSEG, noverlap=ve.NPERSEG // 2,
                               detrend="constant", scaling="density")
        np.testing.assert_allclose(p, ps, rtol=1e-10, atol=1e-18)


class TestAliasing(unittest.TestCase):
    def test_plegar(self):
        # Los casos medidos con el parlante del celular (fs = 795,01 Hz)
        self.assertAlmostEqual(ve.plegar(250.0, FS), 250.0)
        self.assertAlmostEqual(ve.plegar(500.0, FS), 295.01)
        self.assertAlmostEqual(ve.plegar(750.0, FS), 45.01)
        self.assertAlmostEqual(ve.plegar(1000.0, FS), 204.99)

    def test_armonica_reflejada(self):
        n = 30 * 800
        # Tono de 250 Hz con armonicas 2 y 3 por encima de fs/2: el sensor las ve reflejadas
        x = tono(n, 250.0, 0.011) + tono(n, ve.plegar(500.0, FS), 0.0015) + tono(n, ve.plegar(750.0, FS), 0.004)
        f, p, _ = ve.welch(x, FS)
        pks = ve.picos(f, p)
        self.assertAlmostEqual(pks[0].f, 250.0, delta=0.03)
        notas = {round(pk.f): ve.armonica_de(pk, pks[0].f, FS, f[1] - f[0]) for pk in pks[1:3]}
        self.assertEqual(notas, {295: (2, True), 45: (3, True)})

    def test_no_inventa_armonicas(self):
        pk = ve.Pico(69.76, 11.0, 0.001, 0)
        self.assertIsNone(ve.armonica_de(pk, 249.98, FS, FS / ve.NPERSEG))


class TestCaptura(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.base = os.path.join(self.dir.name, "cap")
        self.rng = np.random.default_rng(5)

    def tearDown(self):
        self.dir.cleanup()

    def escribir(self, x_g, seg=None, segmentos=None):
        n = len(x_g)
        c = np.column_stack((np.round(self.rng.normal(3, 1.3, n)), np.round(self.rng.normal(13, 1.3, n)),
                             np.round(x_g / 0.0039))).astype(int)
        t = np.arange(n) / FS
        escribir(self.base, t, c, seg=seg, extra={"periodo_us": 1e6 / FS, "segmentos": segmentos or
                                                  [{"seg": 0, "odr": 800, "periodo_us": 1e6 / FS}]})
        return va.cargar(self.base)

    def test_gravedad_no_tapa_el_tono(self):
        n = 30 * 800
        cap = self.escribir(-1.02 + tono(n, 250.0, 0.011) + self.rng.normal(0, 0.006, n))
        esp = ve.psd(cap)
        pk = ve.picos(esp.f, esp.psd[2])[0]
        self.assertAlmostEqual(pk.f, 250.0, delta=0.03)
        self.assertAlmostEqual(pk.amp_g, 0.011, delta=0.011 * 0.06)   # cuantizado a 3,9 mg/cuenta
        np.testing.assert_allclose(esp.parseval(), 1.0, atol=0.02)

    def test_no_cruza_segmentos(self):
        # Dos segmentos de 5000 muestras: 1 tramo cada uno. Si cruzara el hueco serian 3.
        n = 10000
        seg = np.repeat([0, 1], n // 2)
        segs = [{"seg": 0, "odr": 800, "periodo_us": 1e6 / FS}, {"seg": 1, "odr": 800, "periodo_us": 1e6 / FS}]
        esp = ve.psd(self.escribir(-1.0 + tono(n, 100.0, 0.01), seg=seg, segmentos=segs))
        self.assertEqual(esp.tramos, 2)
        self.assertEqual(esp.segmentos, [0, 1])

    def test_tasa_por_segmento(self):
        # Segundo segmento con el sensor 0,2 % mas rapido: el pico no se duplica ni se corre
        n = 2 * 30 * 800
        h = n // 2
        fs2 = FS * 1.002
        x = np.concatenate((tono(h, 300.0, 0.01), tono(h, 300.0, 0.01, fs=fs2))) - 1.0
        segs = [{"seg": 0, "odr": 800, "periodo_us": 1e6 / FS}, {"seg": 1, "odr": 800, "periodo_us": 1e6 / fs2}]
        esp = ve.psd(self.escribir(x, seg=np.repeat([0, 1], h), segmentos=segs))
        pks = ve.picos(esp.f, esp.psd[2])
        self.assertAlmostEqual(pks[0].f, 300.0, delta=0.05)
        self.assertTrue(all(abs(p.f - 300.0) < 0.3 or p.db < pks[0].db - 20 for p in pks[1:]))

    def test_espectrograma(self):
        # Tono solo en la segunda mitad; hueco entre segmentos en el medio
        n = 20 * 800
        h = n // 2
        x = np.concatenate((np.zeros(h), tono(h, 200.0, 0.01))) - 1.0
        seg = np.repeat([0, 1], h)
        segs = [{"seg": 0, "odr": 800, "periodo_us": 1e6 / FS}, {"seg": 1, "odr": 800, "periodo_us": 1e6 / FS}]
        cap = self.escribir(x, seg=seg, segmentos=segs)
        cap.t[h:] += 3.0                       # 3 s de hueco
        t, f, P = ve.espectrograma(cap)
        k = np.argmin(abs(f - 200))
        hueco = (t > h / FS + 0.5) & (t < h / FS + 2.5)
        self.assertTrue(np.isnan(P[:, hueco]).all())
        antes = np.nanmedian(P[k, t < h / FS - 0.5])
        despues = np.nanmedian(P[k, t > h / FS + 3.5])
        # Esperado ~22 dB: ruido x+y (5,1 mg rms c/u) ~1,3e-7 g^2/Hz; tono de
        # 10 mg rms en el ancho efectivo de Hann (1,5 bins x 3,1 Hz) ~2,1e-5 g^2/Hz
        self.assertGreater(10 * np.log10(despues / antes), 15)

    def test_odr_mezclados(self):
        n = 10000
        segs = [{"seg": 0, "odr": 800, "periodo_us": 1258}, {"seg": 1, "odr": 400, "periodo_us": 2516}]
        cap = self.escribir(np.zeros(n) - 1, seg=np.repeat([0, 1], n // 2), segmentos=segs)
        with self.assertRaises(ValueError):
            ve.psd(cap)
        self.assertEqual(ve.psd(cap, desde=0, hasta=(n // 2 - 1) / FS).segmentos, [0])

    def test_tramo_de_tiempo(self):
        n = 40 * 800
        x = np.where(np.arange(n) < n // 2, 0.0, tono(n, 150.0, 0.01)) - 1.0
        cap = self.escribir(x)
        sin = ve.picos(*(lambda e: (e.f, e.psd[2]))(ve.psd(cap, hasta=19.0)))
        con = ve.picos(*(lambda e: (e.f, e.psd[2]))(ve.psd(cap, desde=21.0)))
        self.assertFalse(any(abs(p.f - 150) < 1 for p in sin))
        self.assertAlmostEqual(con[0].f, 150.0, delta=0.05)


if __name__ == "__main__":
    unittest.main()
