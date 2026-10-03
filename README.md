# VibraScan_v2

Firmware y herramientas de PC para la beca Vibra-Scan: adquisición de vibraciones
con un ADXL345 sobre una Black Pill (STM32F411CEU6). Se construye por etapas;
cada una se prueba sola antes de sumar la siguiente.

## Etapa 2 - muestreo por interrupción DATA_READY a 800 Hz (aprobada)

Copia de la etapa 1 (proyecto `VibraScan`, que queda como referencia).

Aprobada el 2026-10-02 en la placa: `etapa2_prueba.py` sin fallas a 100, 400
y 800 Hz (20 s cada uno) y 60 s a 800 Hz (47 686 muestras, 0 saltos, todos los
contadores en 0, periodo entre flancos de 1257 a 1258 µs). Verificado a mano
con `monitor_vivo.py`: reposo, ±1 g por cara, golpe visible, y recuperación
sola al soltar y reconectar el cable INT1 (sube `stall`).

El micro busca el ADXL345 por I2C (400 kHz), verifica el DEVID y lo configura
(800 Hz, ±4 g, FULL_RES) con DATA_READY saliendo por INT1 -> PB0. Cada flanco
de INT1 dispara una lectura I2C por interrupción (sin bloquear) de 8 registros
desde INT_SOURCE: la muestra y su bit de overrun en una sola transferencia.
Las muestras pasan por un buffer circular y el lazo principal las manda por el
puerto COM virtual USB, varias líneas por envío:

    n,x,y,z                              una línea por muestra (cuentas crudas, 3,9 mg/LSB)
    # estado=OK etapa=2 addr=0x53 ...    una línea de estado por segundo
    > odr=400 ok                         respuesta a un comando de la PC

El ritmo lo marca el sensor; el micro mide con su propio reloj (contador de
ciclos DWT) cuántas muestras llegan por segundo (`sps`) y el periodo mínimo y
máximo entre flancos de INT1 (`tmin`, `tmax`, en µs).

Comando de la PC (una línea de texto): `odr=<Hz>` cambia el ODR a 100, 200,
400 u 800 Hz sin regrabar. 1600 y 3200 Hz quedan para SPI (etapa 6).

Si la placa se reinicia con el USB conectado (por ejemplo al grabar), el
firmware mantiene D+ en bajo 20 ms al arrancar para que la PC la vuelva a
reconocer.

### Conexiones

| ADXL345 | Black Pill |
|---------|------------|
| VCC     | 3V3        |
| GND     | GND        |
| SDA     | PB7        |
| SCL     | PB6        |
| CS      | 3V3 (modo I2C) |
| SDO     | GND (dirección 0x53; a 3V3 es 0x1D, el firmware prueba las dos) |
| INT1    | PB0 (nuevo en la etapa 2) |

La PC se conecta al USB propio de la Black Pill. El ST-Link solo se usa para
grabar, conectado con SWDIO, SWCLK y GND (sin su 3,3 V: la placa se alimenta
por su USB, y unir los dos 3,3 V hizo que uno de los dos USB fallara).

### Compilar y grabar

STM32CubeIDE: File > Import > General > Existing Projects into Workspace,
elegir esta carpeta (sin copiar) y compilar. Para grabar, Run o Debug con
"VibraScan_v2 Debug".

### Probar

    cd pc
    pip install -r requirements.txt
    python etapa2_prueba.py --list
    python etapa2_prueba.py --port COMx --plot
    python etapa2_prueba.py --port COMx --odr 100 400 800 --seconds 20
    python etapa2_prueba.py --port COMx --odr 800 --seconds 60     (prueba larga)

Con el sensor quieto, la prueba tiene que terminar en `ETAPA 2 APROBADA`:

| Chequeo | Qué confirma |
|---|---|
| Sensor detectado | DEVID = 0xE5: cableado y dirección I2C correctos |
| Tasa de muestreo | n crece ~ODR por segundo: se lee cada muestra del sensor |
| INT1 conectado | ~ODR flancos/s en PB0: el cable INT1 -> PB0 funciona |
| Periodo estable | ningún periodo entre flancos se aleja más de 1,5 veces del medio (no se perdió ningún flanco) |
| Secuencia sin saltos | no se pierde ninguna línea entre el micro y la PC |
| Contadores de error en 0 | `ovr`, `i2c_err`, `drop`, `stall` y `bfull` no crecen durante la captura |
| 1 g en reposo | escala y lectura de los tres ejes correctas |

El CSV que guarda tiene una fila por muestra con las columnas `muestra`,
`tiempo_s` (con la tasa medida), `ax_g`, `ay_g`, `az_g`, `modulo_g` y los
valores crudos `ax_cuentas`, `ay_cuentas`, `az_cuentas`.

Para ver los ejes en tiempo real mientras se mueve el sensor:

    python monitor_vivo.py --port COMx
    python monitor_vivo.py --port COMx --odr 400

A 800 Hz el gráfico dibuja 1 de cada k muestras si la ventana tiene más de
4000 (lo indica el título); los datos que llegan no se pierden.

Prueba manual extra: apoyar el sensor sobre cada cara; el eje vertical debe
dar cerca de +1 g o -1 g y los otros dos cerca de 0.

LED de la placa: parpadeo lento = midiendo; rápido = el sensor no responde;
muy rápido = falló la inicialización de un periférico.

## Estructura

    Core/Src/main.c        generado por CubeMX; solo llama a app_init() y app_loop()
    Core/Src/app.c         lógica de la etapa (lo que cambia de etapa a etapa)
    Core/Src/adxl345.c     driver del sensor, independiente del bus (I2C hoy, SPI después)
    Core/Src/usb_link.c    envío a la PC por USB CDC
    USB_DEVICE/, Middlewares/, Drivers/   pila USB y HAL de ST
    pc/etapa2_prueba.py    prueba de aceptación de la etapa
    pc/monitor_vivo.py     gráfico en tiempo real

## Próximas etapas

| # | Etapa | Se aprueba cuando |
|---|---|---|
| 2 | Muestreo por interrupción DATA_READY (INT1 a un pin EXTI) y ODR más alto | muestras/s = ODR, sin saltos |
| 3 | Buffer por bloques y trama binaria con checksum | minutos sin pérdidas |
| 4 | Captura a archivo y gráfico en tiempo | reposo estable, golpe visible |
| 5 | Preproceso y FFT/PSD en Python | pico en la frecuencia de una fuente conocida |
| 6 | SPI a 3200 Hz | igual que 2 y 3 |
| 7 | FFT y filtro adaptativo (LMS) en el micro | mismo resultado que en Python |

## Sobre VibraScan_v2.ioc

Refleja la configuración de esta etapa (I2C1 a 400 kHz, INT1 en PB0 con EXTI0, USB CDC, LED en PC13, 96 MHz),
pero fue editado a mano y no se probó regenerando desde CubeMX. Antes de usar
"Generate Code", hacer una copia de la carpeta: los archivos de USB_DEVICE no
tienen bloques USER CODE y se sobrescriben.
