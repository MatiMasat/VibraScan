"""
VibraScan - prueba de la etapa 5 (preproceso y FFT/PSD en Python).

Analiza una captura con un tono de frecuencia conocida y dice si el espectro
lo encuentra. La captura tiene que empezar con unos segundos SIN tono: asi se
comprueba que el pico aparece cuando se prende la fuente y no es del ambiente.

    python captura.py --port COM8 --segundos 30 --nota "tono 250 Hz"
        (esperar ~5 s y prender el tono hasta el final)
    python etapa5_prueba.py capturas/captura_<fecha> --frecuencia 250

Una captura por tono (por ejemplo 150, 250 y 350 Hz; por debajo de fs/2,
~398 Hz a 800 Hz de ODR).
"""

import argparse
import sys

import numpy as np

import vibrascan_archivo as va
import vibrascan_espectro as ve

# Umbrales
TOL_HZ = 0.5            # el pico a +/- esto de la frecuencia del tono
SNR_MIN_DB = 20.0       # el pico sobre el piso local
PARSEVAL_TOL = 0.02     # integral de la PSD / varianza a 1 +/- esto
SILENCIO_MIN_S = 2.0    # segundos sin tono al principio
BANDA_HZ = 3.0          # banda del tono en el espectrograma (resolucion 3,1 Hz)
ENCENDIDO_DB = 10.0     # en el espectrograma: el tono supera al piso en esto


def encendido(cap, f_tono):
    """Instante (s) en que el tono aparece en el espectrograma, o None."""
    t, f, P = ve.espectrograma(cap, 3)
    datos = np.isfinite(P).any(axis=0)          # sin las columnas vacias (huecos, final)
    t, P = t[datos], P[:, datos]
    # Bandas de al menos un bin: con otra resolucion nunca quedan vacias
    df = f[1] - f[0]
    ancho = max(BANDA_HZ, df)
    banda = np.abs(f - f_tono) <= ancho
    ref = (np.abs(f - f_tono) > 3 * ancho) & (np.abs(f - f_tono) < max(30.0, 12 * ancho))
    with np.errstate(invalid="ignore", divide="ignore"):
        snr = 10 * np.log10(np.nanmax(P[banda], axis=0) / np.nanmedian(P[ref], axis=0))
    on = np.flatnonzero(snr > ENCENDIDO_DB)
    # Encendido = primera columna desde la que el tono queda prendido (90 % de las columnas siguientes)
    for k in on:
        rest = snr[k:]
        if np.mean(rest[np.isfinite(rest)] > ENCENDIDO_DB) >= 0.9:
            return float(t[k])
    return None


def evaluar(cap, f_tono):
    checks = []
    p = cap.meta.get("perdidas", {})
    lost = {k: p.get(k, 0) for k in ("tramas_perdidas", "descartadas_micro", "huecos", "crc_mal")}
    checks.append(("Sin perdidas", all(v == 0 for v in lost.values()),
                   " ".join(f"{k}={v}" for k, v in lost.items())))

    fs_nyq = 1e6 / cap.meta["periodo_us"] / 2 if cap.meta.get("periodo_us") else 400.0
    if f_tono >= fs_nyq:
        print(f"  Atencion: {f_tono} Hz esta por encima de fs/2 = {fs_nyq:.1f} Hz; "
              f"el pico se espera reflejado en {ve.plegar(f_tono, 2 * fs_nyq):.2f} Hz")
    f_obj = ve.plegar(f_tono, 2 * fs_nyq)

    t_on = encendido(cap, f_obj)
    silencio = (t_on - cap.t[0]) if t_on is not None else 0.0
    checks.append(("El pico aparece al prender el tono", t_on is not None and silencio >= SILENCIO_MIN_S,
                   f"tono desde t = {t_on:.1f} s ({silencio:.1f} s de silencio antes, min {SILENCIO_MIN_S:.0f})"
                   if t_on is not None else f"no aparece en {f_obj:.1f} Hz"))
    desde = t_on + 1.0 if t_on is not None else None
    try:
        esp = ve.psd(cap, desde=desde)
    except ValueError as e:
        checks.append(("Espectro", False, str(e)))
        return checks, None

    pks = ve.picos(esp.f, esp.psd[3])
    if not pks:
        checks.append(("Frecuencia del pico", False, "no hay picos"))
        return checks, esp
    pk = pks[0]
    checks.append(("Frecuencia del pico", abs(pk.f - f_obj) <= TOL_HZ,
                   f"{pk.f:.2f} Hz, esperado {f_obj:.2f} Hz (error {pk.f - f_obj:+.2f} Hz, max {TOL_HZ})"))
    checks.append(("Relacion senal/ruido", pk.db >= SNR_MIN_DB,
                   f"{pk.db:.1f} dB sobre el piso (min {SNR_MIN_DB:.0f}), amplitud {pk.amp_g * 1000:.2f} mg rms"))
    pv = esp.parseval()
    checks.append(("Parseval", bool(np.all(np.abs(pv - 1) <= PARSEVAL_TOL)),
                   "x/y/z = " + "/".join(f"{v:.3f}" for v in pv) + f" (1 +/- {PARSEVAL_TOL})"))

    print(f"  tramo analizado desde {desde if desde is not None else cap.t[0]:.1f} s: {esp.tramos} tramos, "
          f"fs medida {esp.fs:.2f} Hz, resolucion {esp.df:.3f} Hz")
    print("  por eje: " + ", ".join(
        f"{n} {pe.f:.2f} Hz {pe.amp_g * 1000:.2f} mg" for n, pe in
        ((n, (ve.picos(esp.f, esp.psd[k]) or [None])[0]) for k, n in enumerate("xyz")) if pe))
    for q in pks[1:6]:
        arm = ve.armonica_de(q, pk.f, esp.fs, esp.df)
        print(f"  otro pico: {q.f:7.2f} Hz {q.db:5.1f} dB {q.amp_g * 1000:6.2f} mg"
              + (f"  armonica {arm[0]}" + (" reflejada por aliasing" if arm[1] else "") if arm else ""))
    return checks, esp


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("captura", help="Captura (.csv, .json o nombre base)")
    parser.add_argument("--frecuencia", type=float, required=True, help="Frecuencia del tono en Hz")
    args = parser.parse_args()

    cap = va.cargar(args.captura)
    print(f"TONO {args.frecuencia:g} Hz: {cap.base}  (nota: {cap.meta.get('nota') or '-'}, "
          f"odr={cap.meta.get('odr_hz')} Hz)")
    checks, _ = evaluar(cap, args.frecuencia)
    print()
    for name, ok, detail in checks:
        print(f"  [{'OK   ' if ok else 'FALLA'}] {name}: {detail}")
    passed = all(ok for _, ok, _ in checks)
    print()
    print(f"ETAPA 5 - TONO {args.frecuencia:g} Hz: " + ("APROBADO" if passed else "CON FALLAS"))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
