"""
Pruebas del filtro adaptativo (etapa 7), sin placa, con senales sinteticas:
se sabe exactamente que tienen adentro (un tono de frecuencia y amplitud
dadas mas ruido), asi se puede comprobar que el filtro hace lo correcto.

    python -m unittest test_vibrascan_filtro -v

La ultima prueba compila el filtro del micro (Core/Src/filtro.c y nlms.c) en
la PC con gcc y compara su salida con la de Python (si no hay gcc, se saltea).
"""

import os
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

import vibrascan_espectro as ve
import vibrascan_filtro as vf

FS = 3178.0          # tasa real medida a 3200 Hz nominales
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def senal(segundos, f_tono=250.0, amp_rms=0.010, ruido_rms=0.006, gravedad=-1.0, seed=1):
    """Tono + ruido blanco + gravedad, en g (como el eje Z quieto con el celular)."""
    rng = np.random.default_rng(seed)
    n = int(segundos * FS)
    t = np.arange(n) / FS
    tono = amp_rms * np.sqrt(2) * np.sin(2 * np.pi * f_tono * t)
    return gravedad + tono + rng.normal(0, ruido_rms, n), tono


def snr_db(x, f_tono):
    """Pico del tono sobre el piso local, en la PSD."""
    f, p, _ = ve.welch(x, FS)
    i = int(round(f_tono / (f[1] - f[0])))
    return 10 * np.log10(p[i - 2:i + 3].max() / np.median(p[max(0, i - 200):i + 200]))


def lms(x, taps, delta, mu):
    """LMS basico (sin normalizar), solo para comparar con NLMS en las pruebas."""
    w = np.zeros(taps)
    hist = np.zeros(taps + delta)
    y = np.empty(len(x))
    for i, v in enumerate(x):
        u = hist[delta - 1:delta - 1 + taps]
        y[i] = w @ u
        w += mu * (v - y[i]) * u
        hist[1:] = hist[:-1]
        hist[0] = v
    return y


class TestPasaaltos(unittest.TestCase):
    def test_saca_la_gravedad_sin_escalon(self):
        x, _ = senal(5)
        hp = vf.Pasaaltos(FS)
        y = np.array([hp.procesar(v) for v in x])
        self.assertEqual(y[0], 0.0)                     # la primera muestra inicializa
        self.assertLess(abs(y[:200]).max(), 0.05)       # sin el escalon de 1 g
        self.assertLess(abs(y[int(FS):].mean()), 1e-3)  # media ~0 tras 1 s


class TestNlms(unittest.TestCase):
    def test_realza_el_tono(self):
        # ALE: la salida y conserva el tono y deja afuera la mayor parte del ruido
        x, tono = senal(20)
        y, e = vf.Filtro(vf.NLMS, FS).procesar_serie(x)
        k = int(5 * FS)                                  # ya convergio
        mejora = snr_db(y[k:], 250) - snr_db(x[k:] - x[k:].mean(), 250)
        print(f"\n  mejora de SNR en y: {mejora:.1f} dB")
        self.assertGreater(mejora, 6.0)
        # y se parece al tono verdadero mucho mas que la entrada
        err_y = np.sqrt(np.mean((y[k:] - tono[k:]) ** 2))
        err_x = np.sqrt(np.mean((x[k:] + 1.0 - tono[k:]) ** 2))
        self.assertLess(err_y, 0.5 * err_x)
        # y el residuo e casi no tiene tono
        self.assertLess(snr_db(e[k:], 250), 6.0)

    def test_converge_en_menos_de_2_s(self):
        x, tono = senal(6)
        y, _ = vf.Filtro(vf.NLMS, FS).procesar_serie(x)
        seg = int(0.25 * FS)
        amp = [np.sqrt(np.mean(y[i:i + seg] ** 2)) for i in range(0, len(y) - seg, seg)]
        final = np.mean(amp[-6:])
        t90 = next(k for k, a in enumerate(amp) if a >= 0.9 * final) * 0.25
        print(f"\n  llega al 90 % de la amplitud final en {t90:.2f} s")
        self.assertLess(t90, 2.0)

    def test_nlms_no_depende_del_nivel_y_lms_si(self):
        # Por que NLMS: el mismo mu sirve para una senal 100 veces mas grande
        # (un golpe, otro rango). El LMS con el mismo paso diverge.
        # (No es exactamente 100 veces: EPS no escala con la senal y pesa un poco
        # al principio, con la historia casi vacia. Medido: 0,5 % rms.)
        x, _ = senal(3, gravedad=0.0)
        y1, _ = vf.nlms(x)
        y100, _ = vf.nlms(100 * x)
        dif = np.sqrt(np.mean((y100 / 100 - y1) ** 2)) / np.sqrt(np.mean(y1 ** 2))
        self.assertLess(dif, 0.02)
        mu_lms = 0.5                                    # anda bien con la senal original...
        self.assertTrue(np.isfinite(lms(x, 32, 2, mu_lms)).all())
        with np.errstate(over="ignore", invalid="ignore"):
            grande = lms(100 * x, 32, 2, mu_lms)        # ...y diverge con la grande
        self.assertFalse(np.isfinite(grande).all() and abs(grande).max() < 1e3)

    def test_reiniciar(self):
        f = vf.Filtro(vf.NLMS, FS)
        x, _ = senal(1)
        a = f.procesar_serie(x)
        f.reiniciar()
        b = f.procesar_serie(x)
        np.testing.assert_array_equal(a[0], b[0])

    def test_parametros(self):
        with self.assertRaises(ValueError):
            vf.Nlms(taps=0)
        with self.assertRaises(ValueError):
            vf.Nlms(delta=40)
        with self.assertRaises(ValueError):
            vf.Nlms(mu=0)


class TestMicroContraPython(unittest.TestCase):
    """El C del micro compilado en la PC contra la referencia en float64."""

    def test_mismo_resultado(self):
        gcc = shutil.which("gcc") or (r"C:\MinGW\bin\gcc.exe" if os.path.exists(r"C:\MinGW\bin\gcc.exe") else None)
        if not gcc:
            self.skipTest("no hay gcc")
        with tempfile.TemporaryDirectory() as d:
            exe = os.path.join(d, "filtro_pc.exe")
            src = [os.path.join(REPO, p) for p in ("test_fw/filtro_pc.c", "Core/Src/filtro.c", "Core/Src/nlms.c")]
            subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-I",
                            os.path.join(REPO, "Core/Inc"), *src, "-o", exe], check=True)
            for f_tono, amp, ruido in ((250.0, 0.010, 0.006), (1000.0, 0.050, 0.013), (37.0, 0.002, 0.006)):
                x, _ = senal(10, f_tono, amp, ruido)
                x32 = x.astype(np.float32)
                x32.tofile(os.path.join(d, "x.f32"))
                subprocess.run([exe, os.path.join(d, "x.f32"), os.path.join(d, "y.f32"),
                                str(FS), str(vf.TAPS), str(vf.DELTA), str(vf.MU)], check=True)
                ye = np.fromfile(os.path.join(d, "y.f32"), dtype=np.float32).reshape(-1, 2)
                y, e = vf.Filtro(vf.NLMS, FS).procesar_serie(x32.astype(np.float64))
                dif = max(abs(ye[:, 0] - y).max(), abs(ye[:, 1] - e).max())
                print(f"\n  {f_tono:6.0f} Hz: diferencia maxima micro (float32) - Python (float64) = {dif * 1e6:.2f} ug")
                self.assertLess(dif, 1e-4)       # 0,1 mg: 39 veces menos que 1 cuenta del sensor


if __name__ == "__main__":
    unittest.main()
