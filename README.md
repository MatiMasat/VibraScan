# VibraScan

Firmware y herramientas de PC para la beca Vibra-Scan: adquisición de vibraciones
con un ADXL345 sobre una Black Pill (STM32F411CEU6). Se construye por etapas;
cada una se prueba sola antes de sumar la siguiente. Cada etapa aprobada tiene
un tag en git (`etapa-1`, `etapa-2`, ...).

## Etapa 3 - buffer por bloques y trama binaria con checksum (en curso)

Sobre la etapa 2 (muestreo por interrupción DATA_READY, ver más abajo). Lo
nuevo:

- **Bloques.** Las muestras se juntan en bloques de hasta 32 muestras
  contiguas (40 ms a 800 Hz), con el índice `n0` de la primera, su hora `t0`
  en µs del micro (tomada en el flanco de INT1) y el ODR. Hay lugar para 32
  bloques (1,3 s a 800 Hz). Un bloque se cierra al llenarse, al detectarse un
  hueco o al cambiar el ODR: así la muestra `i` se tomó en `t0 + i × periodo`.
- **Tramas binarias con CRC.** Cada bloque sale en una trama; estado y
  respuestas también. A 800 Hz pasan ~5,5 kB/s (el texto de la etapa 2 usaba
  ~17 kB/s).
- **Huecos.** Si entre dos muestras pasan más de 1,5 periodos (faltaron
  flancos de INT1), el micro cuenta un `hueco` y marca el bloque siguiente; la
  PC calcula cuántas muestras faltaron con `t0`. En la etapa 2 un corte de
  INT1 no movía `n` y pasaba casi desapercibido.
- **Arranque limpio.** No se manda nada hasta que la PC envía `start`, que
  vacía el buffer y reinicia `seq`. Así no llegan datos viejos al abrir el
  puerto. Si el USB se desconecta deja de mandar hasta el próximo `start`.

### Trama

Little-endian, igual en `Core/Src/trama.c` y `pc/vibrascan_proto.py`:

| Campo | Bytes | Contenido |
|---|---|---|
| sync | 2 | `A5 5A` |
| tipo | 1 | 1 = muestras, 2 = estado (texto), 3 = respuesta (texto) |
| largo | 2 | bytes del contenido |
| seq | 2 | contador de tramas (0 tras `start`) |
| contenido | largo | |
| crc | 2 | CRC-16/CCITT-FALSE de tipo + largo + seq + contenido |

Contenido de una trama de muestras (204 bytes con 32 muestras; 213 la trama):
`n0` (uint32), `t0` (uint32, µs, da la vuelta cada 71 min), `odr` (uint16,
Hz), `flags` (uint8: 1 = overrun, 2 = hueco antes del bloque, 4 = primer
bloque tras `start`), `cant` (uint8) y `cant` × (x, y, z) en int16 (cuentas
crudas, 3,9 mg/LSB).

La PC distingue dónde se perdió algo:

| Qué se ve | Dónde se perdió |
|---|---|
| `seq` salta | entre el micro y la PC (si además sube `drop`, el micro no pudo enviar) |
| `seq` contiguo, `n0` salta, sube `bfull` | buffer del micro lleno (la PC tardó más de 1,3 s en leer) |
| `seq` y `n0` contiguos, `t0` salta, flag de hueco | no llegaron flancos de INT1 |
| flag de overrun, sube `ovr` | el sensor pisó una muestra antes de que el micro la leyera |
| CRC mal | bytes corruptos o desalineados |

### Comandos (texto, una línea; la respuesta llega en una trama tipo 3)

| Comando | Qué hace |
|---|---|
| `start` | vacía el buffer, `seq` = 0, manda respuesta y estado, y empieza a mandar bloques |
| `stop` | deja de mandar (el sensor sigue midiendo) |
| `odr=<Hz>` | cambia el ODR a 100, 200, 400 u 800 Hz (1600 y 3200 quedan para SPI, etapa 6) |

La trama de estado (una por segundo mientras se manda) es texto
`estado=OK etapa=3 addr=0x53 devid=0xE5 odr=800 n=... ovr= i2c_err= drop=
stall= bfull= hueco= int1= sps= tmin= tmax=`. Los contadores son acumulados
desde el arranque y deberían quedarse quietos; `int1`, `sps`, `tmin` y `tmax`
son del último segundo completo.

### Probar

    cd pc
    pip install -r requirements.txt
    python -m unittest test_vibrascan_proto -v                (sin placa)
    python etapa3_prueba.py --list
    python etapa3_prueba.py --port COMx --odr 100 400 800
    python etapa3_prueba.py --port COMx --odr 800 --seconds 600   (prueba larga)
    python vibrascan_proto.py --port COMx --dump --seconds 2      (ver las tramas)

Con el sensor quieto, la prueba tiene que terminar en `ETAPA 3 APROBADA`:

| Chequeo | Qué confirma |
|---|---|
| Sensor detectado | DEVID = 0xE5: cableado y dirección I2C correctos |
| Arranque limpio | `start ok` con seq 0, estado y primer bloque marcado: nada viejo en camino |
| Tramas íntegras (CRC) | ningún CRC mal ni byte suelto |
| Sin tramas perdidas | `seq` contiguo |
| Sin muestras descartadas en el micro | `n0` contiguo: el buffer nunca se llenó |
| Sin huecos de INT1 | `t0` avanza 32 periodos por bloque |
| Tasa de muestreo | muestras / tiempo entre `t0` ≈ ODR (reloj del sensor) |
| INT1 conectado, Periodo estable | ~ODR flancos/s y ningún periodo 1,5 veces lejos del medio |
| Contadores de error sin cambios | `ovr`, `i2c_err`, `drop`, `stall`, `bfull`, `hueco` |
| 1 g en reposo | escala y lectura de los tres ejes correctas |

El CSV (`pc/capturas/etapa3_<fecha>_<odr>hz.csv`) tiene las columnas
`muestra`, `tiempo_s` (reloj del micro: `t0` + i × periodo medido), `ax_g`,
`ay_g`, `az_g`, `modulo_g` y las cuentas crudas.

Para ver los ejes en tiempo real:

    python monitor_vivo.py --port COMx
    python monitor_vivo.py --port COMx --odr 400

Cada sesión del monitor queda en `pc/capturas/`: `monitor_<fecha>.log` (una
línea por segundo; eventos TRAMA PERDIDA / DESCARTE / HUECO / CRC / CORTE /
GOLPE / CARA / CONTADOR; al cerrar, un resumen con el chequeo de las pruebas
manuales), `.csv` con todas las muestras crudas y `_N.png` con el gráfico
(tecla `g` y al cerrar). A 800 Hz el gráfico dibuja 1 de cada k muestras si
la ventana tiene más de 4000; los datos no se pierden.

Pruebas manuales con el monitor: reposo (curva de 10 s completa, `sps` ≈ ODR),
±1 g en cada cara, golpe visible sin cortes, soltar INT1 unos segundos
(aparece `HUECO` con su duración y se recupera solo) y sin pérdidas.

LED de la placa: parpadeo lento = midiendo; rápido = el sensor no responde;
muy rápido = falló la inicialización de un periférico.

## Etapa 2 - muestreo por interrupción DATA_READY a 800 Hz (aprobada, tag `etapa-2`)

Cada flanco de INT1 (DATA_READY del ADXL345 -> PB0, EXTI0) dispara una
lectura I2C a 400 kHz por interrupción (sin bloquear) de 8 registros desde
INT_SOURCE: la muestra y su bit de overrun en una sola transferencia. El
ritmo lo marca el sensor; el micro mide con su contador de ciclos (DWT) el
periodo mínimo y máximo entre flancos. Una red de seguridad (`stall`) relanza
la lectura si INT1 queda en alto sin flanco. Protocolo de texto: `n,x,y,z`
por muestra, `# estado=...` por segundo, `> ...` como respuesta.

Aprobada el 2026-10-02 (`etapa2_prueba.py` a 100/400/800 Hz y 60 s a 800 Hz:
47 686 muestras, 0 saltos, contadores en 0, periodo 1257–1258 µs) y el
2026-10-03 con el monitor (reposo, ±1 g en cuatro caras, 26 golpes de hasta
2,2 g, recuperación sola al soltar INT1). Esa prueba mostró que con INT1
suelto `n` no avanza y el hueco solo queda marcado por `ovr`: lo resuelve la
etapa 3.

Si la placa se reinicia con el USB conectado (por ejemplo al grabar), el
firmware mantiene D+ en bajo 20 ms al arrancar para que la PC la vuelva a
reconocer.

## Conexiones

| ADXL345 | Black Pill |
|---------|------------|
| VCC     | 3V3        |
| GND     | GND        |
| SDA     | PB7        |
| SCL     | PB6        |
| CS      | 3V3 (modo I2C) |
| SDO     | GND (dirección 0x53; a 3V3 es 0x1D, el firmware prueba las dos) |
| INT1    | PB0 (desde la etapa 2) |

La PC se conecta al USB propio de la Black Pill. El ST-Link solo se usa para
grabar, conectado con SWDIO, SWCLK y GND (sin su 3,3 V: la placa se alimenta
por su USB, y unir los dos 3,3 V hizo que uno de los dos USB fallara).

## Compilar y grabar

STM32CubeIDE: File > Import > General > Existing Projects into Workspace,
elegir esta carpeta (sin copiar) y compilar. Para grabar, Run o Debug con
"VibraScan_v2 Debug" (el proyecto de Eclipse conserva ese nombre).

## Estructura

    Core/Src/main.c        generado por CubeMX; solo llama a app_init() y app_loop()
    Core/Src/app.c         lógica de la etapa (lo que cambia de etapa a etapa)
    Core/Src/adxl345.c     driver del sensor, independiente del bus (I2C hoy, SPI después)
    Core/Src/trama.c       tramas binarias con CRC (no conoce el transporte)
    Core/Src/usb_link.c    envío a la PC por USB CDC
    USB_DEVICE/, Middlewares/, Drivers/   pila USB y HAL de ST
    pc/vibrascan_proto.py  protocolo del lado PC: decodificador y verificador de continuidad
    pc/test_vibrascan_proto.py  pruebas del protocolo sin placa
    pc/etapa3_prueba.py    prueba de aceptación de la etapa 3
    pc/monitor_vivo.py     gráfico en tiempo real y registro de la sesión
    pc/etapa2_prueba.py    prueba de la etapa 2 (no sirve con el firmware de la 3;
                           la 3 usa sus constantes y funciones)

## Próximas etapas

| # | Etapa | Se aprueba cuando |
|---|---|---|
| 2 | Muestreo por interrupción DATA_READY (INT1 a un pin EXTI) y ODR más alto | muestras/s = ODR, sin saltos (aprobada) |
| 3 | Buffer por bloques y trama binaria con checksum | minutos sin pérdidas |
| 4 | Captura a archivo y gráfico en tiempo | reposo estable, golpe visible |
| 5 | Preproceso y FFT/PSD en Python | pico en la frecuencia de una fuente conocida |
| 6 | SPI a 3200 Hz (con el FIFO del ADXL345) | igual que 2 y 3 |
| 7 | FFT y filtro adaptativo (LMS) en el micro | mismo resultado que en Python |

## Sobre VibraScan_v2.ioc

Refleja la configuración (I2C1 a 400 kHz, INT1 en PB0 con EXTI0, USB CDC, LED
en PC13, 96 MHz), pero fue editado a mano y no se probó regenerando desde
CubeMX. Antes de usar "Generate Code", hacer un commit: los archivos de
USB_DEVICE no tienen bloques USER CODE y se sobrescriben (se recuperan con
`git checkout -- USB_DEVICE`).
