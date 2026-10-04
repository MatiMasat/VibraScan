"""
Pruebas del protocolo de la etapa 3, sin placa.

    python -m unittest test_vibrascan_proto -v
"""

import random
import unittest

import vibrascan_proto as p

PERIOD_US = 1258  # periodo real medido a 800 Hz nominales


def make_stream(n_blocks, seq0=0, n_start=0, t_start=0, first=True, odr=800):
    """Tramas de un flujo sin perdidas: estado + n_blocks bloques de 32 muestras."""
    frames = [p.build_frame(p.TYPE_STATUS, seq0, b"estado=OK etapa=3 odr=800")]
    n, t = n_start, t_start
    for k in range(n_blocks):
        samples = [((n + i) % 100, -(n + i) % 50, -256) for i in range(p.BLOCK_SAMPLES)]
        flags = p.FLAG_FIRST if (first and k == 0) else 0
        frames.append(p.build_frame(p.TYPE_SAMPLES, seq0 + 1 + k,
                                    p.build_block_payload(n, t, odr, flags, samples)))
        n += p.BLOCK_SAMPLES
        t += p.BLOCK_SAMPLES * PERIOD_US
    return frames


def decode_all(data, chunk=None):
    dec, chk = p.Decoder(), p.Checker()
    frames = []
    step = chunk or len(data) or 1
    for i in range(0, len(data), step):
        for fr in dec.feed(data[i:i + step]):
            chk.frame(fr)
            frames.append(fr)
    return dec, chk, frames


class TestCrc(unittest.TestCase):
    def test_valor_de_referencia(self):
        # Valor de control publicado de CRC-16/CCITT-FALSE
        self.assertEqual(p.crc16(b"123456789"), 0x29B1)


class TestTrama(unittest.TestCase):
    def test_tamano_de_bloque_completo(self):
        payload = p.build_block_payload(0, 0, 800, 0, [(0, 0, 0)] * 32)
        self.assertEqual(len(payload), 204)
        self.assertEqual(len(p.build_frame(p.TYPE_SAMPLES, 0, payload)), 213)

    def test_ida_y_vuelta(self):
        samples = [(1, -2, 3), (-32768, 32767, 0)]
        fr = p.build_frame(p.TYPE_SAMPLES, 7, p.build_block_payload(404780, 12345678, 800,
                                                                     p.FLAG_OVR, samples))
        _, _, frames = decode_all(fr)
        blk = p.parse_block(frames[0])
        self.assertEqual((blk.seq, blk.n0, blk.t0, blk.odr, blk.flags), (7, 404780, 12345678, 800, p.FLAG_OVR))
        self.assertEqual(blk.samples, samples)

    def test_contenido_incoherente(self):
        payload = p.build_block_payload(0, 0, 800, 0, [(0, 0, 0)] * 4)[:-1]  # falta un byte
        self.assertIsNone(p.parse_block(p.Frame(p.TYPE_SAMPLES, 0, payload)))


class TestDecoder(unittest.TestCase):
    def test_flujo_limpio_en_pedazos(self):
        data = b"".join(make_stream(50))
        for chunk in (1, 7, 64, 213, 512):
            dec, chk, frames = decode_all(data, chunk)
            self.assertEqual(len(frames), 51, f"chunk={chunk}")
            self.assertEqual((dec.crc_errors, dec.skipped), (0, 0))
            self.assertEqual((chk.lost_frames, chk.lost_in_micro, chk.gaps), (0, 0, 0))
            self.assertEqual(chk.samples, 50 * 32)

    def test_basura_al_principio(self):
        # Por ejemplo, el final de una trama de una sesion anterior
        junk = bytes([0xA5, 0x00, 0x5A, 0xA5, 0x5A, 0xFF, 0x13])
        dec, chk, frames = decode_all(junk + b"".join(make_stream(5)), chunk=3)
        self.assertEqual(len(frames), 6)
        self.assertEqual(chk.lost_frames, 0)

    def test_cada_byte_corrupto_se_detecta(self):
        """Se corrompe cada byte de una trama, de a uno: siempre se pierde
        solo esa trama, nunca se acepta una trama falsa."""
        stream = make_stream(3)
        target = 2                                    # segundo bloque
        start = sum(len(f) for f in stream[:target])
        for pos in range(len(stream[target])):
            data = bytearray(b"".join(stream))
            data[start + pos] ^= 0x10
            _, chk, frames = decode_all(bytes(data), chunk=50)
            seqs = [f.seq for f in frames]
            self.assertEqual(seqs, [0, 1, 3], f"byte {pos}")
            self.assertEqual(chk.lost_frames, 1, f"byte {pos}")

    def test_bytes_perdidos(self):
        # Se pierden 20 bytes en medio de una trama: se pierde esa trama sola
        stream = make_stream(4)
        data = bytearray(b"".join(stream))
        cut = len(stream[0]) + len(stream[1]) + 50
        del data[cut:cut + 20]
        dec, chk, frames = decode_all(bytes(data), chunk=64)
        self.assertEqual([f.seq for f in frames], [0, 1, 3, 4])
        self.assertEqual(chk.lost_frames, 1)

    def test_ruido_aleatorio_no_genera_tramas(self):
        rng = random.Random(1)
        noise = bytes(rng.getrandbits(8) for _ in range(200_000))
        noise = noise.replace(b"\xA5\x5A", b"\xA5\x5B")[:100_000] + b"\xA5\x5A" * 500
        dec, _, frames = decode_all(noise, chunk=1000)
        self.assertEqual(frames, [])

    def test_largo_imposible(self):
        bad = p.SYNC + p.HEADER.pack(1, 60000, 0)
        dec, _, frames = decode_all(bad + b"".join(make_stream(2)))
        self.assertEqual(len(frames), 3)
        self.assertEqual(dec.bad_length, 1)


class TestChecker(unittest.TestCase):
    def test_trama_perdida_por_usb(self):
        stream = make_stream(6)
        del stream[3]
        _, chk, _ = decode_all(b"".join(stream))
        self.assertEqual(chk.lost_frames, 1)
        self.assertEqual(chk.lost_in_transit, 32)                  # las 32 muestras de esa trama
        self.assertEqual((chk.lost_in_micro, chk.gaps), (0, 0))   # no se confunde con otra cosa

    def test_buffer_lleno_en_el_micro(self):
        # El micro descarta 100 muestras: n0 y t0 saltan juntos, seq sigue
        a = make_stream(3)
        b = make_stream(3, seq0=3, n_start=3 * 32 + 100, t_start=(3 * 32 + 100) * PERIOD_US, first=False)
        _, chk, _ = decode_all(b"".join(a + b[1:]))
        self.assertEqual(chk.lost_in_micro, 100)
        self.assertEqual((chk.lost_frames, chk.lost_in_transit, chk.gaps), (0, 0, 0))

    def test_hueco_de_int1(self):
        # INT1 suelto 3 s: n0 sigue contiguo, t0 salta 3 s y el bloque trae FLAG_GAP
        a = make_stream(40)
        n = 40 * 32
        t = n * PERIOD_US + 3_000_000
        samples = [(0, 0, -256)] * 32
        gap = p.build_frame(p.TYPE_SAMPLES, 41, p.build_block_payload(n, t, 800, p.FLAG_GAP, samples))
        _, chk, _ = decode_all(b"".join(a) + gap)
        self.assertEqual(chk.gaps, 1)
        self.assertAlmostEqual(chk.gap_samples, 3_000_000 / PERIOD_US, delta=10)
        self.assertEqual((chk.lost_frames, chk.lost_in_micro), (0, 0))

    def test_una_muestra_sin_flanco(self):
        # Falta un solo flanco dentro de la sesion: se detecta igual
        a = make_stream(10)
        n = 10 * 32
        blk = p.build_block_payload(n, (n + 1) * PERIOD_US, 800, 0, [(0, 0, -256)] * 32)
        _, chk, _ = decode_all(b"".join(a) + p.build_frame(p.TYPE_SAMPLES, 11, blk))
        self.assertEqual((chk.gaps, chk.gap_samples), (1, 1))

    def test_periodo_medido(self):
        # Arranca con 1250 us (800 Hz nominal) y converge al real
        _, chk, _ = decode_all(b"".join(make_stream(200)))
        self.assertAlmostEqual(chk.period_us, PERIOD_US, delta=1)
        self.assertEqual(chk.gaps, 0)

    def test_vuelta_de_t0_y_seq(self):
        # t0 da la vuelta a los 2^32 us y seq a los 65536: no son perdidas
        stream = make_stream(10, seq0=65530, n_start=2**32 - 100, t_start=2**32 - 5 * 32 * PERIOD_US)
        _, chk, _ = decode_all(b"".join(stream))
        self.assertEqual((chk.lost_frames, chk.lost_in_micro, chk.gaps), (0, 0, 0))
        self.assertEqual(chk.t_us, 2**32 - 5 * 32 * PERIOD_US + 9 * 32 * PERIOD_US)

    def test_start_reinicia_seq(self):
        # Segundo start a mitad de la sesion: seq vuelve a 0, no es perdida
        _, chk, _ = decode_all(b"".join(make_stream(5) + make_stream(5, n_start=999_000)))
        self.assertEqual((chk.lost_frames, chk.lost_in_micro, chk.gaps), (0, 0, 0))

    def test_cambio_de_odr(self):
        a = make_stream(3)
        n = 3 * 32
        blk = p.build_block_payload(n, n * PERIOD_US + 50_000, 400, 0, [(0, 0, -256)] * 32)
        _, chk, _ = decode_all(b"".join(a) + p.build_frame(p.TYPE_SAMPLES, 4, blk))
        self.assertEqual(chk.gaps, 0)
        self.assertEqual([e.kind for e in chk.events], ["inicio", "odr"])


if __name__ == "__main__":
    unittest.main()
