"""
VibraScan - captura a archivo (etapa 4).

Lee las tramas del puerto (protocolo de vibrascan_proto.py), se reconecta solo
si el puerto se pierde y escribe dos archivos:

    <base>.csv   una fila por muestra: seg,n,t_s,x,y,z
    <base>.json  metadatos, segmentos, eventos y resumen

Columnas del CSV:
    seg   segmento: tramo de muestras contiguas con ODR y rango fijos. Cambia
          en cada hueco de INT1, perdida, reconexion o cambio de ODR/rango:
          un analisis (FFT) no debe cruzar de un segmento a otro.
    n     indice de la muestra que da el firmware (vuelve a 0 si la placa se
          reinicia)
    t_s   tiempo de la muestra en s desde el inicio de la captura. Dentro de
          una conexion es el reloj del micro (t0 del bloque + i x periodo);
          entre conexiones se alinea con el reloj de la PC (~10 ms).
    x,y,z cuentas crudas del sensor (escala en el JSON: 0,0039 g/cuenta)

El CSV se escribe a medida que llegan los datos y el JSON se actualiza cada
META_S segundos: si el programa se corta, lo grabado queda en disco.
"""

import datetime
import json
import os
import time

import serial

import vibrascan_proto as vp

G_PER_LSB = 0.0039          # FULL_RES: 3,9 mg/cuenta en cualquier rango
RANGOS = (2, 4, 8, 16)
RECONNECT_S = 1.0           # espera entre intentos de reabrir el puerto
FLUSH_S = 1.0               # cada cuanto se vuelca el CSV a disco
META_S = 10.0               # cada cuanto se reescribe el JSON
MAX_EVENTS = 2000           # eventos guardados en el JSON
DISCONTINUIDADES = ("seq", "n", "hueco", "odr")   # eventos de vp.Checker que abren segmento
COUNTERS = ("ovr", "i2c_err", "drop", "stall", "bfull", "hueco")


def limites_cuentas(rango):
    """Cuentas minima y maxima del sensor en FULL_RES para un rango en g:
    10 bits a 2 g, un bit mas por cada duplicacion del rango."""
    bits = 10 + RANGOS.index(rango)
    return -(1 << (bits - 1)), (1 << (bits - 1)) - 1


class _Regresion:
    """Recta t = a + periodo * n por minimos cuadrados, con los (n0, t0) de los bloques."""

    def __init__(self):
        self.k = 0
        self.n_ref = self.t_ref = None
        self.sx = self.sy = self.sxx = self.sxy = 0.0

    def add(self, n, t):
        if self.n_ref is None:
            self.n_ref, self.t_ref = n, t      # se resta para no perder precision
        x, y = n - self.n_ref, t - self.t_ref
        self.k += 1
        self.sx += x
        self.sy += y
        self.sxx += x * x
        self.sxy += x * y

    def periodo(self):
        den = self.k * self.sxx - self.sx * self.sx
        return (self.k * self.sxy - self.sx * self.sy) / den if self.k >= 2 and den > 0 else None


class Captura:
    """Captura desde un puerto. Llamar poll() seguido y close() al terminar.

    poll() devuelve (tramas, eventos): las tramas recibidas (para mostrarlas
    en vivo) y los eventos nuevos como (tipo, texto).
    """

    def __init__(self, port, base=None, odr=None, rango=None, nota=""):
        self.port, self.base, self.odr_req, self.rango_req, self.nota = port, base, odr, rango, nota
        self.inicio = datetime.datetime.now().isoformat(timespec="seconds")
        self.t_pc0 = time.monotonic()
        self.ser = None
        self.dec = self.chk = None
        self.conn = 0
        self.lost_since = None
        self.last_try = 0.0
        self.finished_conns = []        # (Decoder, Checker) de conexiones anteriores

        self.seg = -1
        self.segments = []
        self.new_seg = "inicio"         # motivo del proximo segmento, o None
        self.anchor = None              # (t0 sin vueltas, t_s) del primer bloque de la conexion
        self.t_first_block = None       # time.monotonic() del primer bloque de la captura
        self.reg = None
        self.period_us = None

        self.status = {}
        self.first_status = None
        self.counters_first = None
        self.odr = self.rango = None
        self.tmin = self.tmax = None
        self.samples = 0
        self.reconexiones = 0
        self.events = []
        self._pending = []
        self._crc_seen = 0
        self.closed = False

        self.csv = None
        if base:
            os.makedirs(os.path.dirname(base) or ".", exist_ok=True)
            self.csv = open(base + ".csv", "w", encoding="ascii", newline="")
            self.csv.write("seg,n,t_s,x,y,z\n")
        self.t_flush = self.t_meta = time.monotonic()
        self._open(first=True)

    # ---- conexion -----------------------------------------------------------

    def _open(self, first=False):
        try:
            self.ser = vp.open_stream(self.port, self.odr_req, self.rango_req)
        except (serial.SerialException, OSError):
            if first:
                raise
            return False
        self.dec, self.chk = vp.Decoder(), vp.Checker()
        self._crc_seen = 0
        self.conn += 1
        self.anchor = None
        if not first:
            self.reconexiones += 1
            self.new_seg = "reconexion"
            self._event("reconexion", f"puerto {self.port} abierto de nuevo (conexion {self.conn})")
        return True

    def _lost(self, err):
        self._event("puerto_perdido", f"se perdio el puerto: {err}")
        try:
            self.ser.close()
        except (serial.SerialException, OSError):
            pass
        self.ser = None
        self.finished_conns.append((self.dec, self.chk))
        self.dec = self.chk = None
        self.lost_since = self.last_try = time.monotonic()

    # ---- lectura ------------------------------------------------------------

    def poll(self):
        frames = []
        if self.ser is None:
            if time.monotonic() - self.last_try >= RECONNECT_S:
                self.last_try = time.monotonic()
                self._open()
            else:
                time.sleep(0.05)
        else:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError) as e:
                self._lost(e)
                data = b""
            if data:
                frames = self.dec.feed(data)
                for fr in frames:
                    self._frame(fr)
                if self.dec.crc_errors != self._crc_seen:
                    self._event("crc", f"{self.dec.crc_errors - self._crc_seen} trama(s) con CRC mal, descartadas")
                    self._crc_seen = self.dec.crc_errors
                    self.new_seg = self.new_seg or "crc"
        self._periodic()
        events, self._pending = self._pending, []
        return frames, events

    def _event(self, kind, text):
        ev = {"t_s": round(self.t_now(), 3), "tipo": kind, "texto": text}
        if len(self.events) < MAX_EVENTS:
            self.events.append(ev)
        self._pending.append((kind, text))

    def t_now(self):
        """Segundos desde el primer bloque (o desde el inicio, si todavia no llego)."""
        return time.monotonic() - (self.t_first_block or self.t_pc0)

    def _frame(self, fr):
        first_block_of_conn = self.anchor is None
        for ev in self.chk.frame(fr):
            self._event(ev.kind, ev.text)
            if ev.kind in DISCONTINUIDADES and not first_block_of_conn:
                self.new_seg = self.new_seg or ev.kind

        if fr.ftype == vp.TYPE_STATUS:
            self._status(vp.parse_fields(fr.payload.decode("ascii", errors="replace")))
        elif fr.ftype == vp.TYPE_REPLY:
            self._event("respuesta", fr.payload.decode("ascii", errors="replace"))
        elif fr.ftype == vp.TYPE_SAMPLES:
            blk = vp.parse_block(fr)
            if blk is not None:
                self._block(blk)

    def _status(self, st):
        self.status = st
        if st.get("estado") != "OK":
            self._event("estado", " ".join(f"{k}={v}" for k, v in st.items()))
            return
        if self.first_status is None:
            self.first_status = dict(st)
            self.counters_first = {k: st.get(k) for k in COUNTERS}
        rango = int(st["rango"]) if st.get("rango", "").isdigit() else None
        if rango and self.rango and rango != self.rango:
            self._event("rango", f"rango {self.rango} -> {rango} g")
            self.new_seg = self.new_seg or "rango"
        if rango:
            self.rango = rango
        tmin, tmax = int(st.get("tmin", 0) or 0), int(st.get("tmax", 0) or 0)
        if tmin > 0 and int(st.get("sps", 0) or 0) > 0:
            self.tmin = tmin if self.tmin is None else min(self.tmin, tmin)
            self.tmax = tmax if self.tmax is None else max(self.tmax, tmax)

    def _block(self, blk):
        now = time.monotonic()
        t_ext = self.chk.t_us
        if self.t_first_block is None:
            self.t_first_block = now
        if self.anchor is None:
            # Conexion nueva: la placa pudo reiniciarse (su reloj vuelve a 0),
            # asi que se alinea con el reloj de la PC
            self.anchor = (t_ext, now - self.t_first_block)
        if self.new_seg:
            self._close_regression()
            self.seg += 1
            t_s = self.anchor[1] + (t_ext - self.anchor[0]) / 1e6
            self.segments.append({"seg": self.seg, "motivo": self.new_seg, "conexion": self.conn,
                                  "n0": blk.n0, "t_s": round(t_s, 6), "odr": blk.odr,
                                  "rango": self.rango, "muestras": 0, "periodo_us": None})
            self.new_seg = None
            self.reg = _Regresion()
        self.odr = blk.odr

        period = self.chk.period_us or 1e6 / blk.odr
        t_base = self.anchor[1] + (t_ext - self.anchor[0]) / 1e6
        if self.csv:
            self.csv.write("".join(f"{self.seg},{blk.n0 + i},{t_base + i * period / 1e6:.6f},{x},{y},{z}\n"
                                   for i, (x, y, z) in enumerate(blk.samples)))
        self.reg.add(blk.n0, t_ext)
        self.segments[-1]["muestras"] += len(blk.samples)
        self.samples += len(blk.samples)

    def _close_regression(self):
        if self.reg is not None and self.segments:
            p = self.reg.periodo()
            self.segments[-1]["periodo_us"] = round(p, 4) if p else None

    def _periodic(self):
        now = time.monotonic()
        if self.csv and now - self.t_flush >= FLUSH_S:
            self.t_flush = now
            self.csv.flush()
        if self.base and now - self.t_meta >= META_S:
            self.t_meta = now
            self.write_meta()

    # ---- resumen ------------------------------------------------------------

    def totals(self):
        """Perdidas sumadas de todas las conexiones."""
        conns = self.finished_conns + ([(self.dec, self.chk)] if self.dec else [])
        t = {"tramas": 0, "crc_mal": 0, "bytes_descartados": 0, "tramas_perdidas": 0,
             "muestras_en_tramas_perdidas": 0, "descartadas_micro": 0, "huecos": 0,
             "muestras_en_huecos": 0, "bloques_con_overrun": 0}
        for dec, chk in conns:
            t["tramas"] += dec.frames
            t["crc_mal"] += dec.crc_errors
            t["bytes_descartados"] += dec.skipped
            t["tramas_perdidas"] += chk.lost_frames
            t["muestras_en_tramas_perdidas"] += chk.lost_in_transit
            t["descartadas_micro"] += chk.lost_in_micro
            t["huecos"] += chk.gaps
            t["muestras_en_huecos"] += chk.gap_samples
            t["bloques_con_overrun"] += chk.overruns
        t["reconexiones"] = self.reconexiones
        return t

    def period(self):
        """Periodo medio (us), promedio de las regresiones por segmento pesado por muestras."""
        self._close_regression()
        segs = [s for s in self.segments if s["periodo_us"]]
        w = sum(s["muestras"] for s in segs)
        return sum(s["periodo_us"] * s["muestras"] for s in segs) / w if w else None

    def meta(self):
        period = self.period()
        fs = self.first_status or {}
        return {
            "formato": "VibraScan captura v1",
            "columnas": {
                "seg": "segmento: muestras contiguas con ODR y rango fijos",
                "n": "indice de la muestra del firmware",
                "t_s": "s desde el inicio; reloj del micro dentro de una conexion, "
                       "reloj de la PC entre conexiones (~10 ms)",
                "x,y,z": "cuentas crudas; g = cuentas x escala_g_por_cuenta",
            },
            "inicio": self.inicio,
            "completa": self.closed,
            "puerto": self.port,
            "nota": self.nota,
            "odr_hz": self.odr,
            "rango_g": self.rango,
            "escala_g_por_cuenta": G_PER_LSB,
            "limites_cuentas": list(limites_cuentas(self.rango)) if self.rango in RANGOS else None,
            "firmware": {k: fs.get(k) for k in ("etapa", "addr", "devid")},
            "periodo_us": round(period, 4) if period else None,
            "tasa_hz": round(1e6 / period, 4) if period else None,
            "periodo_firmware_us": [self.tmin, self.tmax],
            "duracion_s": round(self.t_now(), 3) if self.t_first_block else 0.0,
            "muestras": self.samples,
            "perdidas": self.totals(),
            "contadores_firmware": {"inicio": self.counters_first,
                                    "fin": {k: self.status.get(k) for k in COUNTERS}},
            "segmentos": self.segments,
            "eventos": self.events,
        }

    def write_meta(self):
        if not self.base:
            return
        tmp = self.base + ".json.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.meta(), f, ensure_ascii=False, indent=1)
        os.replace(tmp, self.base + ".json")   # nunca queda un JSON a medio escribir

    def close(self):
        if self.ser is not None:
            try:
                self.ser.write(b"stop\n")
                self.ser.close()
            except (serial.SerialException, OSError) as e:
                self._event("puerto_perdido", f"al cerrar el puerto: {e}")
        self.closed = True
        if self.csv:
            self.csv.close()
        self.write_meta()
