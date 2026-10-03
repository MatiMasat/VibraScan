# VibraScan

Firmware y herramientas de PC para la beca Vibra-Scan: adquisición de vibraciones
con un ADXL345 sobre una Black Pill (STM32F411CEU6). Se construye por etapas;
cada una se prueba sola antes de sumar la siguiente.

## Etapa actual: 1 - lectura del sensor y envío en texto

El micro busca el ADXL345 por I2C, verifica el DEVID, lo configura
(100 Hz, ±4 g, FULL_RES) y manda por el puerto COM virtual USB:

    n,x,y,z                              una línea por muestra (cuentas crudas, 3,9 mg/LSB)
    # estado=OK etapa=1 addr=0x53 ...    una línea de estado por segundo

El ritmo lo marca el sensor: el micro consulta el bit DATA_READY y lee cada
muestra una sola vez. No hay timer del micro muestreando en paralelo.

### Conexiones

| ADXL345 | Black Pill |
|---------|------------|
| VCC     | 3V3        |
| GND     | GND        |
| SDA     | PB7        |
| SCL     | PB6        |
| CS      | 3V3 (modo I2C) |
| SDO     | GND (dirección 0x53; a 3V3 es 0x1D, el firmware prueba las dos) |

La PC se conecta al USB propio de la Black Pill. El ST-Link solo se usa para grabar.

### Compilar y grabar

STM32CubeIDE: File > Import > General > Existing Projects into Workspace,
elegir esta carpeta (sin copiar) y compilar. Para grabar, Run o Debug con
"VibraScan Debug".

### Probar

    cd pc
    pip install -r requirements.txt
    python etapa1_prueba.py --list
    python etapa1_prueba.py --port COMx --plot

Con el sensor quieto, la prueba tiene que terminar en `ETAPA 1 APROBADA`:

| Chequeo | Qué confirma |
|---|---|
| Sensor detectado | DEVID = 0xE5: cableado y dirección I2C correctos |
| Tasa de muestreo | n crece ~100 por segundo: se lee cada muestra del sensor |
| Secuencia sin saltos | no se pierde ninguna línea entre el micro y la PC |
| Contadores de error en 0 | sin fallas de I2C, sobreescrituras ni líneas descartadas |
| 1 g en reposo | escala y lectura de los tres ejes correctas |

El CSV que guarda tiene una fila por muestra con las columnas `muestra`,
`tiempo_s` (con la tasa medida), `ax_g`, `ay_g`, `az_g`, `modulo_g` y los
valores crudos `ax_cuentas`, `ay_cuentas`, `az_cuentas`.

Para ver los ejes en tiempo real mientras se mueve el sensor:

    python monitor_vivo.py --port COMx

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
    pc/etapa1_prueba.py    prueba de aceptación de la etapa

## Próximas etapas

| # | Etapa | Se aprueba cuando |
|---|---|---|
| 2 | Muestreo por interrupción DATA_READY (INT1 a un pin EXTI) y ODR más alto | muestras/s = ODR, sin saltos |
| 3 | Buffer por bloques y trama binaria con checksum | minutos sin pérdidas |
| 4 | Captura a archivo y gráfico en tiempo | reposo estable, golpe visible |
| 5 | Preproceso y FFT/PSD en Python | pico en la frecuencia de una fuente conocida |
| 6 | SPI a 3200 Hz | igual que 2 y 3 |
| 7 | FFT y filtro adaptativo (LMS) en el micro | mismo resultado que en Python |

## Sobre VibraScan.ioc

Refleja la configuración de esta etapa (I2C1, USB CDC, LED en PC13, 96 MHz),
pero fue editado a mano y no se probó regenerando desde CubeMX. Antes de usar
"Generate Code", hacer una copia de la carpeta: los archivos de USB_DEVICE no
tienen bloques USER CODE y se sobrescriben.
