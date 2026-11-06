"""
Pruebas de la medicion de velocidad de un motor (etapa 7), sin placa. Usan un
motor sintetico cuya velocidad se conoce: linea y armonicas moduladas por la
rotacion, como en las capturas reales (ver docs/prueba_motor.md).

    python -m unittest test_vibrascan_motor -v
"""

import contextlib
import io
import os
import tempfile
import unittest

import numpy as np

import etapa7_prueba as e7
import vibrascan_archivo as va
import vibrascan_motor as vm
from test_vibrascan_archivo import escribir

FS = 3172.0


def motor_sintetico(tramos, fs_ini=FS, fs_fin=FS, ruido=0.01, bandas=0.3, seed=1, reposo_s=6.0):
    """Reposo, marcha por tramos (duracion, f_linea, f_1) y reposo. Devuelve (t, cuentas)."""
    rng = np.random.default_rng(seed)
    dur = sum(d for d, *_ in tramos) + 2 * reposo_s
    n = int(dur * (fs_ini + fs_fin) / 2)
    fs_t = fs_ini + (fs_fin - fs_ini) * np.arange(n) / n
    t = np.concatenate(([0.0], np.cumsum(1 / fs_t[:-1])))      # reloj del sensor que deriva
    f_l, f_1, on = np.zeros(n), np.zeros(n), np.zeros(n)
    t0 = reposo_s
    for d, fl, f1 in tramos:
        i = (t >= t0) & (t < t0 + d)
        f_l[i], f_1[i], on[i] = fl, f1, 1.0
        t0 += d
    fase_l = 2 * np.pi * np.cumsum(f_l / fs_t)
    fase_1 = 2 * np.pi * np.cumsum(f_1 / fs_t)
    x = np.zeros(n)
    for k, amp in ((1, 0.5), (2, 0.25), (3, 0.3), (4, 0.15), (5, 0.2)):
        x += amp * np.sin(k * fase_l) * (1 + bandas * np.cos(fase_1) + bandas / 2 * np.cos(2 * fase_1 + 0.4))
    x = x * on
    c = np.column_stack((x, 0.6 * x, 1.0 + 0.8 * x)) + rng.normal(0, ruido, (n, 3))
    return t, np.round(c / 0.0039).astype(int)


class Base(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.base = os.path.join(self.dir.name, "cap")

    def tearDown(self):
        self.dir.cleanup()

    def cargar(self, t, c, huecos=0):
        periodo = 1e6 * t[-1] / (len(t) - 1)
        escribir(self.base, t, c, rango=16, extra={
            "periodo_us": periodo, "odr_hz": 3200,
            "perdidas": {"tramas_perdidas": 0, "descartadas_micro": 0, "crc_mal": 0, "huecos": huecos},
            "segmentos": [{"seg": 0, "odr": 3200, "rango": 16, "periodo_us": periodo}]})
        return va.cargar(self.base)

    def motor(self, *args, **kw):
        """Corre la prueba de etapa7_prueba sin ensuciar la salida."""
        with contextlib.redirect_stdout(io.StringIO()):
            checks, seg, esc, polos = e7.motor(*args, **kw)
        return {nombre: ok for nombre, ok, _ in checks}, seg, esc, polos


class TestVelocidad(Base):
    def test_velocidad_constante(self):
        cap = self.cargar(*motor_sintetico([(30.0, 100.0, 24.17)]))
        tramos = vm.marchas(cap)
        self.assertEqual(len(tramos), 1)
        self.assertAlmostEqual(tramos[0][0], 6.0, delta=0.3)
        self.assertAlmostEqual(tramos[0][1], 36.0, delta=0.3)
        seg = vm.seguir(cap, tramos, 100.0, 4)
        self.assertTrue(np.isfinite(seg.f1).all())
        self.assertLess(np.abs(seg.f1 - 24.17).max(), 0.03)
        self.assertLess(np.abs(seg.f_linea - 100.0).max(), 0.03)

    def test_reloj_que_deriva(self):
        # Reloj como el de la captura real, exagerado: la velocidad no se corre
        cap = self.cargar(*motor_sintetico([(60.0, 100.0, 24.17)], fs_ini=3175.0, fs_fin=3145.0))
        seg = vm.seguir(cap, vm.marchas(cap), 100.0, 4)
        self.assertLess(np.abs(seg.f1 - 24.17).max(), 0.03)
        self.assertLess(np.abs(seg.f_linea - 100.0).max(), 0.03)

    def test_escalones_con_linea_fija(self):
        # Variar la tension: la linea sigue en 100 Hz y la velocidad baja
        velocidades = (24.2, 22.0, 19.0, 15.0)
        cap = self.cargar(*motor_sintetico([(25.0, 100.0, v) for v in velocidades]))
        seg = vm.seguir(cap, vm.marchas(cap), 100.0, 4)
        esc = vm.escalones(seg)
        self.assertEqual(len(esc), 4)
        for e, v in zip(esc, velocidades):
            self.assertAlmostEqual(e.f1, v, delta=0.03)
            self.assertGreaterEqual(e.ventanas, 8)

    def test_escalones_con_variador(self):
        # Variador: la linea baja junto con la velocidad, siempre ~3 % bajo la sincronica
        lineas = (100.0, 80.0, 60.0, 40.0)
        cap = self.cargar(*motor_sintetico([(25.0, fl, 0.97 * fl / 4) for fl in lineas]))
        seg = vm.seguir(cap, vm.marchas(cap), None, 4)
        esc = vm.escalones(seg)
        self.assertEqual(len(esc), 4)
        for e, fl in zip(esc, lineas):
            self.assertAlmostEqual(e.f_linea, fl, delta=0.05)
            self.assertAlmostEqual(e.f1, 0.97 * fl / 4, delta=0.05)

    def test_sin_bandas_no_inventa_velocidad(self):
        cap = self.cargar(*motor_sintetico([(30.0, 100.0, 24.17)], bandas=0.0))
        seg = vm.seguir(cap, vm.marchas(cap), 100.0, 4)
        self.assertTrue(np.isfinite(seg.f_linea).all())
        self.assertFalse(np.isfinite(seg.f1).any())
        self.assertEqual(vm.escalones(seg), [])

    def test_muestras_corruptas_no_molestan(self):
        t, c = motor_sintetico([(30.0, 100.0, 24.17)])
        c[int(15 * FS)] = (0, 16384, -7680)
        c[int(15 * FS) + 1] = (520, 16384, 513)
        cap = self.cargar(t, c)
        seg = vm.seguir(cap, vm.marchas(cap), 100.0, 4)
        self.assertLess(np.abs(seg.f1 - 24.17).max(), 0.03)


class TestMarchas(Base):
    def test_golpe_e_inclinacion_no_son_marcha(self):
        rng = np.random.default_rng(3)
        n = int(30 * FS)
        t = np.arange(n) / FS
        z = np.ones(n)
        z[int(10 * FS):int(10 * FS) + 40] += 3.0            # golpe
        y = 0.5 * np.clip((t - 18) / 2, 0, 1)               # el sensor se inclina de a poco
        c = np.column_stack((np.zeros(n), y, z)) + rng.normal(0, 0.01, (n, 3))
        self.assertEqual(vm.marchas(self.cargar(t, np.round(c / 0.0039).astype(int))), [])

    def test_reposos(self):
        cap = self.cargar(*motor_sintetico([(20.0, 100.0, 24.0)]))
        tramos = vm.marchas(cap)
        rep = vm.reposos(cap, tramos)
        self.assertEqual(len(rep), 2)
        self.assertLess(rep[0][1], tramos[0][0])
        self.assertGreater(rep[1][0], tramos[0][1])


class TestPolos(unittest.TestCase):
    def test_polos_de_nominal(self):
        self.assertEqual(vm.polos_de_nominal(1350, 50), 4)
        self.assertEqual(vm.polos_de_nominal(1450, 50), 4)
        self.assertEqual(vm.polos_de_nominal(2850, 50), 2)
        self.assertEqual(vm.polos_de_nominal(950, 50), 6)
        self.assertEqual(vm.polos_de_nominal(1750, 60), 4)
        self.assertIsNone(vm.polos_de_nominal(10000, 50))     # universal: no es de induccion


class TestPruebaMotor(Base):
    def test_aprueba(self):
        checks, seg, esc, polos = self.motor(self.cargar(*motor_sintetico([(30.0, 100.0, 24.17)])),
                                             50.0, rpm_nominal=1350)
        self.assertEqual(polos, 4)
        self.assertTrue(all(checks.values()), checks)
        self.assertEqual(len(esc), 1)
        self.assertAlmostEqual(vm.rpm_de(esc[0].f1), 1450, delta=2)

    def test_escalones_esperados(self):
        cap = self.cargar(*motor_sintetico([(25.0, 100.0, v) for v in (24.2, 21.0, 18.0)]))
        checks, *_ = self.motor(cap, 50.0, rpm_nominal=1350, n_escalones=3)
        self.assertTrue(all(checks.values()), checks)         # por debajo de la nominal: esta bien aca
        checks, *_ = self.motor(cap, 50.0, rpm_nominal=1350, n_escalones=4)
        self.assertFalse(checks["Escalones de velocidad"])
        checks, *_ = self.motor(cap, 50.0, rpm_nominal=1350)
        self.assertFalse(checks["No baja de la nominal"])

    def test_variador(self):
        cap = self.cargar(*motor_sintetico([(25.0, fl, 0.97 * fl / 4) for fl in (100.0, 70.0)]))
        checks, *_ = self.motor(cap, 50.0, polos=4, variador=True, n_escalones=2)
        self.assertTrue(all(checks.values()), checks)
        checks, *_ = self.motor(cap, 50.0, polos=4, n_escalones=2)   # sin --variador no cierra
        self.assertFalse(all(checks.values()))

    def test_sin_motor_apagado_falla(self):
        cap = self.cargar(*motor_sintetico([(30.0, 100.0, 24.17)], reposo_s=0.5))
        checks, *_ = self.motor(cap, 50.0, rpm_nominal=1350)
        self.assertFalse(checks["Motor apagado"])

    def test_reposo_falla_sin_romperse(self):
        rng = np.random.default_rng(4)
        n = int(20 * FS)
        c = np.round((np.array([0, 0, 1.0]) + rng.normal(0, 0.01, (n, 3))) / 0.0039).astype(int)
        checks, seg, *_ = self.motor(self.cargar(np.arange(n) / FS, c), 50.0, rpm_nominal=1350)
        self.assertFalse(checks["Motor prendido"])
        self.assertIsNone(seg)

    def test_grafico(self):
        cap = self.cargar(*motor_sintetico([(25.0, 100.0, v) for v in (24.2, 21.0)]))
        _, seg, esc, polos = self.motor(cap, 50.0, rpm_nominal=1350)
        png = self.base + "_motor.png"
        e7.grafico_motor(cap, seg, esc, polos, 1350, png)
        self.assertGreater(os.path.getsize(png), 10000)


if __name__ == "__main__":
    unittest.main()
