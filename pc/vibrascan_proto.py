"""
VibraScan - protocolo binario de la etapa 3 (lado PC).

El firmware manda tramas; la PC manda comandos de texto ("start", "stop",
"odr=<Hz>", una linea cada uno).

Trama (little-endian):
    sync   2 bytes  A5 5A
    tipo   1 byte   1 = muestras, 2 = estado (texto), 3 = respuesta (texto)
    largo  2 bytes  bytes del contenido
    seq    2 bytes  contador de tramas (vuelve a 0 tras 65535)
    contenido
    crc    2 bytes  CRC-16/CCITT-FALSE de tipo + largo + seq + contenido

Contenido de una trama de muestras:
    n0     uint32   indice de la primera muestra del bloque
    t0     uint32   us del micro en el flanco de INT1 de la primera muestra
    odr    uint16   Hz
    flags  uint8    FLAG_OVR | FLAG_GAP | FLAG_FIRST
    cant   uint8    muestras del bloque (1 a 32)
    cant x (x, y, z) int16, cuentas crudas (3.9 mg/LSB)

Las muestras de un bloque son siempre contiguas: la muestra i se tomo en
t0 + i * periodo.

Uso como herramienta (muestra las tramas como texto):
    python vibrascan_proto.py --port COM8 --dump
    python vibrascan_proto.py --port COM8 --dump --seconds 5
"""

import argparse
import binascii
import struct
import sys
import time
from dataclasses import dataclass, field

SYNC = b"\xA5\x5A"
HEADER = struct.Struct("<BHH")          # tipo, largo, seq (despues del sync)
BLOCK_HEADER = struct.Struct("<IIHBB")  # n0, t0, odr, flags, cant
SAMPLE = struct.Struct("<hhh")

TYPE_SAMPLES = 1
TYPE_STATUS = 2
TYPE_REPLY = 3

FLAG_OVR = 0x01     # el sensor piso al menos una muestra de este bloque
FLAG_GAP = 0x02     # antes de este bloque hubo un hueco (faltaron flancos de INT1)
FLAG_FIRST = 0x04   # primer bloque desde "start"

BLOCK_SAMPLES = 32
MAX_PAYLOAD = 1024  # una trama mas larga es un sync falso
SEQ_MOD = 1 << 16
T_MOD = 1 << 32
GAP_PERIODS = 1.5   # mas de 1,5 periodos entre muestras = hueco
CLOCK_TOLERANCE = 0.02  # error admitido del periodo estimado, sobre el tiempo esperado


def crc16(data):
    """CRC-16/CCITT-FALSE: polinomio 0x1021, valor inicial 0xFFFF."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


# ---- Armado (pruebas y simulacion) ------------------------------------------

def build_frame(ftype, seq, payload):
    body = HEADER.pack(ftype, len(payload), seq % SEQ_MOD) + bytes(payload)
    return SYNC + body + struct.pack("<H", crc16(body))


def build_block_payload(n0, t0, odr, flags, samples):
    return (BLOCK_HEADER.pack(n0 % T_MOD, t0 % T_MOD, odr, flags, len(samples))
            + b"".join(SAMPLE.pack(*s) for s in samples))


# ---- Decodificacion ----------------------------------------------------------

@dataclass
class Frame:
    ftype: int
    seq: int
    payload: bytes


@dataclass
class Block:
    seq: int
    n0: int
    t0: int
    odr: int
    flags: int
    samples: list   # [(x, y, z), ...] en cuentas


def parse_block(frame):
    """Trama de muestras -> Block, o None si el contenido no es coherente."""
    p = frame.payload
    if len(p) < BLOCK_HEADER.size:
        return None
    n0, t0, odr, flags, cant = BLOCK_HEADER.unpack_from(p)
    if len(p) != BLOCK_HEADER.size + cant * SAMPLE.size:
        return None
    samples = [SAMPLE.unpack_from(p, BLOCK_HEADER.size + i * SAMPLE.size) for i in range(cant)]
    return Block(frame.seq, n0, t0, odr, flags, samples)


def parse_fields(text):
    """'estado=OK odr=800 ...' -> {'estado': 'OK', 'odr': '800', ...}"""
    fields = {}
    for token in text.split():
        key, sep, value = token.partition("=")
        if sep:
            fields[key] = value
    return fields


class Decoder:
    """Separa las tramas de un flujo de bytes que llega en pedazos.

    Busca el sync, controla que el largo sea razonable y verifica el CRC. Si
    algo falla avanza un byte y vuelve a buscar: asi se recupera de bytes
    perdidos o corruptos sin perder mas que la trama afectada.
    """

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.crc_errors = 0     # candidatos con sync y largo validos pero CRC mal
        self.bad_length = 0     # candidatos con largo imposible
        self.skipped = 0        # bytes descartados buscando el sync

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # Puede quedar medio sync al final: se guarda el ultimo byte
                keep = 1 if self.buf[-1:] == SYNC[:1] else 0
                self.skipped += len(self.buf) - keep
                del self.buf[:len(self.buf) - keep]
                return out
            if i > 0:
                self.skipped += i
                del self.buf[:i]
            if len(self.buf) < 2 + HEADER.size:
                return out
            ftype, length, seq = HEADER.unpack_from(self.buf, 2)
            if length > MAX_PAYLOAD:
                self.bad_length += 1
                self.skipped += 1
                del self.buf[:1]
                continue
            total = 2 + HEADER.size + length + 2
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2:total - 2])
            (crc,) = struct.unpack_from("<H", self.buf, total - 2)
            if crc != crc16(body):
                self.crc_errors += 1
                self.skipped += 1
                del self.buf[:1]
                continue
            out.append(Frame(ftype, seq, body[HEADER.size:]))
            self.frames += 1
            del self.buf[:total]


@dataclass
class Event:
    kind: str       # "inicio", "seq", "n", "hueco", "ovr", "odr", "contenido"
    text: str
    block: object = None
    count: int = 0  # tramas o muestras perdidas, segun el tipo


@dataclass
class Checker:
    """Controla la continuidad de las tramas y dice donde se perdio algo.

    seq salta                 -> se perdieron tramas entre el micro y la PC
    n0 salta, seq contiguo    -> el micro descarto muestras (buffer lleno)
    n0 salta, seq salto       -> las muestras iban en las tramas perdidas
    t0 salta con n0 contiguo  -> no llegaron flancos de INT1 (hueco)
    FLAG_OVR                  -> el sensor piso una muestra
    """

    events: list = field(default_factory=list)
    lost_frames: int = 0
    lost_in_micro: int = 0
    lost_in_transit: int = 0    # muestras que iban en tramas perdidas
    gaps: int = 0
    gap_samples: int = 0
    overruns: int = 0
    samples: int = 0
    blocks: int = 0
    bad_blocks: int = 0
    _seq: int = None
    _blk: Block = None
    _t_ext: int = 0         # t0 sin vueltas, en us
    _period: float = None   # us entre muestras, medido
    _seq_lost: bool = False # se perdieron tramas desde el ultimo bloque

    def _event(self, kind, text, block=None, count=0):
        ev = Event(kind, text, block, count)
        self.events.append(ev)
        return ev

    def frame(self, frame):
        """Registra una trama. Devuelve los eventos que genero."""
        n_ev = len(self.events)
        if self._seq is not None:
            lost = (frame.seq - self._seq - 1) % SEQ_MOD
            if lost and frame.seq != 0:   # seq = 0: el firmware recibio start
                self.lost_frames += lost
                self._seq_lost = True
                self._event("seq", f"se perdieron {lost} tramas entre el micro y la PC "
                                   f"(seq {self._seq} -> {frame.seq})", count=lost)
        self._seq = frame.seq
        if frame.ftype == TYPE_SAMPLES:
            blk = parse_block(frame)
            if blk is None:
                self.bad_blocks += 1
                self._event("contenido", f"trama de muestras con contenido incoherente (seq {frame.seq})")
            else:
                self._block(blk)
        return self.events[n_ev:]

    def _block(self, blk):
        prev = self._blk
        self.blocks += 1
        self.samples += len(blk.samples)
        if blk.flags & FLAG_OVR:
            self.overruns += 1
            self._event("ovr", f"el sensor piso muestras en el bloque n0={blk.n0}", blk)

        if prev is None or blk.flags & FLAG_FIRST:
            self._seq_lost = False
            self._t_ext = blk.t0
            self._period = 1e6 / blk.odr if blk.odr else None
            if blk.flags & FLAG_FIRST:
                self._event("inicio", f"primer bloque desde start (n0={blk.n0})", blk)
            self._blk = blk
            return
        if blk.odr != prev.odr:
            self._event("odr", f"odr {prev.odr} -> {blk.odr} Hz", blk)
            self._t_ext += (blk.t0 - prev.t0) % T_MOD
            self._period = 1e6 / blk.odr if blk.odr else None
            self._blk = blk
            return

        dn = (blk.n0 - prev.n0) % T_MOD - len(prev.samples)   # muestras que el micro no mando
        dt = (blk.t0 - prev.t0) % T_MOD
        self._t_ext += dt
        seq_lost, self._seq_lost = self._seq_lost, False
        if dn and seq_lost:
            self.lost_in_transit += dn
            self._event("n", f"{dn} muestras iban en las tramas perdidas", blk, dn)
        elif dn:
            self.lost_in_micro += dn
            self._event("n", f"el micro descarto {dn} muestras (n {prev.n0 + len(prev.samples)} "
                             f"-> {blk.n0}): buffer lleno", blk, dn)
        steps = len(prev.samples) + dn      # periodos esperados entre t0 y t0
        if self._period and steps > 0:
            expected = steps * self._period
            extra = dt - expected
            limit = max((GAP_PERIODS - 1) * self._period, CLOCK_TOLERANCE * expected)
            if extra > limit or blk.flags & FLAG_GAP:
                missing = max(0, round(extra / self._period))
                self.gaps += 1
                self.gap_samples += missing
                self._event("hueco", f"hueco de {max(extra, 0) / 1000:.1f} ms antes de n0={blk.n0} "
                                     f"(~{missing} muestras sin flanco de INT1)", blk, missing)
            elif not dn:
                # Periodo medido (el reloj del sensor no es exacto): promedio lento
                measured = dt / steps
                self._period += (measured - self._period) * 0.1
        self._blk = blk

    @property
    def period_us(self):
        return self._period

    @property
    def t_us(self):
        """t0 del ultimo bloque, sin vueltas."""
        return self._t_ext


# ---- Herramienta de linea de comandos ---------------------------------------

def open_stream(port, odr=None):
    """Abre el puerto, detiene un envio anterior, descarta lo viejo y manda start."""
    import serial

    ser = serial.Serial(port, 115200, timeout=0.1)
    ser.write(b"stop\n")
    time.sleep(0.2)
    ser.reset_input_buffer()
    if odr:
        ser.write(f"odr={odr}\n".encode("ascii"))
    ser.write(b"start\n")
    return ser


def describe(frame, blk=None):
    if frame.ftype == TYPE_SAMPLES and blk is not None:
        flags = "".join(c for f, c in ((FLAG_OVR, "O"), (FLAG_GAP, "H"), (FLAG_FIRST, "I"))
                        if blk.flags & f) or "-"
        x, y, z = blk.samples[0] if blk.samples else (0, 0, 0)
        return (f"seq={frame.seq} n0={blk.n0} t0={blk.t0 / 1e6:.6f} s odr={blk.odr} "
                f"cant={len(blk.samples)} flags={flags} primera=({x},{y},{z})")
    text = frame.payload.decode("ascii", errors="replace")
    kind = {TYPE_STATUS: "estado", TYPE_REPLY: "respuesta"}.get(frame.ftype, f"tipo {frame.ftype}")
    return f"seq={frame.seq} {kind}: {text}"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Puerto serie, ej: COM8")
    parser.add_argument("--dump", action="store_true", help="Muestra cada trama como texto")
    parser.add_argument("--seconds", type=float, default=None, help="Duracion (default: hasta Ctrl+C)")
    parser.add_argument("--odr", type=int, default=None, help="Cambia el ODR al arrancar")
    args = parser.parse_args()

    ser = open_stream(args.port, args.odr)
    dec, chk = Decoder(), Checker()
    t_end = time.monotonic() + args.seconds if args.seconds else None
    try:
        while t_end is None or time.monotonic() < t_end:
            for fr in dec.feed(ser.read(ser.in_waiting or 1)):
                events = chk.frame(fr)
                if args.dump:
                    print(describe(fr, parse_block(fr) if fr.ftype == TYPE_SAMPLES else None))
                for ev in events:
                    print(f"  ** {ev.kind}: {ev.text}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"stop\n")
        ser.close()
    print(f"tramas={dec.frames} crc_err={dec.crc_errors} bytes_descartados={dec.skipped} "
          f"tramas_perdidas={chk.lost_frames} ({chk.lost_in_transit} muestras) descartadas_micro={chk.lost_in_micro} "
          f"huecos={chk.gaps} ovr={chk.overruns} muestras={chk.samples}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
