# VibraScan

Firmware y herramientas de PC para la beca Vibra-Scan: adquisición de vibraciones
con un ADXL345 sobre una Black Pill (STM32F411CEU6). Se construye por etapas;
cada una se prueba sola antes de sumar la siguiente. Cada etapa aprobada tiene
un tag en git (`etapa-1`, `etapa-2`, ...).

## Etapa 6 - SPI a 3200 Hz con el FIFO del ADXL345 (aprobada, tag `etapa-6`)

Aprobada el 2026-10-08 en la placa:

- `etapa3_prueba.py` a 3200, 1600, 800, 400 y 100 Hz sin fallas, y 10 minutos
  a 3200 Hz: 1 906 848 muestras, 0 CRC mal, 0 tramas perdidas, 0 huecos,
  contadores sin cambios, intervalo entre avisos de INT1 de 629 µs exactos.
- Tonos con el parlante del celular a 3200 Hz: **500 Hz en 499,97 Hz** (a
  800 Hz aparecía reflejado en 293 Hz) y **1000 Hz en 999,92 Hz**, ambos
  > 40 dB sobre el piso y Parseval 1,000. La armónica 3 del tono de 1000 Hz
  (3000 Hz, por encima de fs/2 ≈ 1589 Hz) apareció reflejada en 178 Hz: el
  aliasing sigue existiendo, ahora por encima de ~1590 Hz.
- Monitor a 3200 Hz: reposo, golpes, reconexión del USB y corte de INT1
  (un solo hueco, con hora exacta a los dos lados).

El sensor pasa de I2C a **SPI** (recableado, ver Conexiones) y se lee por su
**FIFO**. El ODR llega a 3200 Hz: el límite de medición sube de ~397 Hz a
~1590 Hz (a 800 Hz un tono de 500 Hz aparecía reflejado en 293 Hz).

### Cómo se lee

- **SPI1** en PA4–PA7, modo 3 (CPOL = 1, CPHA = 1), **3 MHz** (96 MHz / 32; el
  sensor admite hasta 5 MHz). CS por GPIO: entre dos transacciones queda en
  alto ≥ 6 µs (la hoja de datos pide ≥ 5 µs entre lecturas del FIFO por
  encima de 1,6 MHz). Lecturas por **DMA** (DMA2 Stream0/3, canal 3). Todo
  eso está en `Core/Src/sensor_spi.c`.
- **FIFO en modo stream** (32 niveles) con la interrupción **watermark** por
  INT1: aviso cada `fifo` muestras (**2** por defecto). En cada aviso: lectura
  de FIFO_STATUS (cuántas hay) y después cada muestra (6 bytes), encadenadas
  en las interrupciones del DMA. Si el micro se demora, el FIFO guarda hasta
  ~30 muestras más (~9 ms a 3200 Hz). FIFO lleno = `ovr` y hueco.
- El watermark es por nivel: si al terminar un lote INT1 sigue en alto (ya
  se juntaron otras `fifo` muestras) se lanza otro lote enseguida; si no
  habría flanco y el muestreo se trabaría.
- **Hora de cada muestra**: la muestra `fifo − 1` del lote es la del flanco
  de INT1; las demás se ubican con el período nominal.
- **SPI no avisa si el sensor se desconecta** (no hay acuse de recibo: un
  sensor ausente se lee como 0x00 o 0xFF). Una vez por segundo se lee DEVID;
  si no da 0xE5 cuenta `bus_err` y vuelve a buscar el sensor. El mismo
  chequeo guarda FIFO_STATUS e INT_SOURCE en `diag_fifo_status` y
  `diag_int_source`, que se leen con el ST-Link sin detener el micro.
- Buffer de 128 bloques (1,3 s a 3200 Hz).

### Lo que se encontró: la lectura no puede pisar la conversión siguiente

Con el aviso cada 16 muestras (el diseño inicial), a 1600 y 3200 Hz el ruido
de Z daba 32–43 mg en vez de ~13 mg. No era ruido: en cada ciclo, **la
muestra (o las dos) que el sensor convertía mientras se leía el lote salía
desviada en Z** (−20 mg a 800 Hz, hasta −139 mg a 3200 Hz), lo que en el
espectro es un peine de tonos en la frecuencia de los avisos (199, 397,
596 Hz…). Ni capacitores de desacople en el GY-291 ni bajar el reloj del SPI
lo resolvían. Lo que importa es **en qué momento** hay actividad en el bus:
cada muestra leída ocupa ~30 µs y a 3200 Hz llega una cada 312 µs.

| Aviso cada | z rms a 3200 Hz | Desvío por muestra | Picos |
|---|---|---|---|
| 16 | 39,0 mg | −66 y −92 mg | 199, 397… Hz |
| 8 | 42,6 mg | −67 y −139 mg | 397, 795… Hz |
| 4 | 13,0 mg | ±1 mg | ninguno |
| **2** | **13,1 mg** | ±1 mg | ninguno |
| 1 | 13,3 mg | ±1 mg | ninguno |

Con 4 o menos el lote termina antes de la conversión siguiente. Se eligió
**2**: ~235 µs de margen ante demoras con la mitad de interrupciones que 1.
**No subir el aviso, ni agregar trabajo que demore las lecturas (etapa 7),
sin volver a medir** con `fifo=`. En las etapas 2 a 5 (I2C, una muestra por
interrupción, siempre en la misma fase) esto no se veía.

### Hora de las muestras tras un corte (encontrado en las pruebas manuales)

Al reconectar INT1 con el FIFO lleno aparecían horas corridas (un bloque
9 ms fuera de lugar, después huecos falsos de 0,6–0,9 ms). Quedó así:

- Flanco de un aviso recién cumplido (el FIFO tiene entre `fifo` y `fifo`+3
  muestras): la muestra `fifo` − 1 es la del flanco (exacto).
- **FIFO lleno**: mientras se lee, el sensor sigue pisando las más viejas y la
  secuencia deja de ser continua. Se **resincroniza**: se leen y descartan
  los lotes hasta un aviso exacto (< 1 ms con aviso cada 2). El hueco se
  marca una vez (con OVR).
- Cualquier otro lote: **continuidad** (la anterior + un período).
- Un flanco que llega mientras se lee un lote es viejo y se descarta.

En la PC, `vibrascan_proto.Checker` solo promedia el período con mediciones
a ±2 % del actual (antes un bloque con la hora mal lo dejaba en 286 µs y cada
bloque siguiente parecía un hueco) e informa `tiempo` si `t0` retrocede. El
estado de cada segundo se saltea si el USB está ocupado, sin gastar `seq`.

### Otras observaciones

- A **1600 y 3200 Hz el bit menos significativo es siempre 0**: la
  resolución efectiva es 7,8 mg/cuenta (en ≤ 800 Hz, 3,9). No figura en la
  revisión 0 de la hoja de datos; es del sensor (la decodificación es la
  misma para todos los ODR).
- A **1600 Hz la media se corre ~30 mg** (z −1,047 g, y −0,041 g, contra
  −1,017 g y +0,005 g en las demás tasas), con el aviso en 2 también: es del
  sensor. No afecta al espectro (cada tramo resta su media).
- Ruido con el aviso en 2 (x/y/z, mg rms): 3200 Hz 9,3/7,6/13,1;
  1600 Hz 10,0/12,0/13,9; 800 Hz 5,1/4,8/7,6; 400 Hz 3,8/3,5/5,5;
  100 Hz 2,2/2,1/3,0. La densidad de ruido es la misma en todas las tasas
  (~230–390 µg/√Hz): el rms crece con la raíz del ancho de banda.

### Comandos nuevos

| Comando | Qué hace |
|---|---|
| `odr=1600`, `odr=3200` | además de 100–800 |
| `fifo=<n>` | muestras por aviso de INT1 (1 a 31, por defecto 2). Diagnóstico: ver arriba |
| `spi=<kHz>` | reloj del SPI: 3000, 1500, 750 o 375 (por defecto 3000). Diagnóstico |

El estado agrega `bus=spi`, `spi=<kHz>` y `fifo=<n>`; `i2c_err` pasa a
llamarse `bus_err`. INT1 da `odr/fifo` avisos por segundo (un poco menos:
un lote puede traer alguna muestra de más) y `tmin`/`tmax` son el intervalo
entre avisos (`fifo` períodos).

### Probar

    cd pc
    python etapa3_prueba.py --port COMx --odr 3200 1600 800 400 100
    python etapa3_prueba.py --port COMx --odr 3200 --seconds 600
    python captura.py --port COMx --segundos 30 --odr 3200 --nota "tono 500 Hz"
    python etapa5_prueba.py capturas/captura_<fecha> --frecuencia 500

Sin placa, el driver del sensor se prueba en la PC contra un ADXL345
simulado:

    gcc -std=c99 -Wall -Wextra -I Core/Inc test_fw/test_adxl345.c Core/Src/adxl345.c -o test_adxl345 && ./test_adxl345

## Etapa 5 - preproceso y FFT/PSD en Python (aprobada, tag `etapa-5`)

Solo PC; el firmware es el de la etapa 4.

### Método (`pc/vibrascan_espectro.py`)

PSD por el método de Welch escrito con numpy (sin scipy), para que la
etapa 7 lo replique en el micro con las mismas convenciones:

| | |
|---|---|
| Ventana | Hann periódica: w[k] = 0,5 − 0,5 cos(2πk/n), k = 0..n−1 |
| Tramos | 4096 muestras (~5,1 s, resolución ~0,19 Hz), solapados al 50 % |
| Preproceso | a cada tramo se le resta su media (gravedad y deriva lenta) |
| Escala | densidad de una cara en g²/Hz: P = 2 \|FFT(x·w)\|² / (fs·Σw²), sin duplicar DC ni Nyquist. Parseval: ∫P df = varianza |
| Segmentos | dentro de cada segmento de la captura (nunca a través de un hueco), con la **tasa medida** de cada uno, interpoladas a una grilla común; no mezcla ODR |
| Picos | máximos que superan al piso local (mediana ±20 Hz) en ≥ 10 dB; frecuencia por interpolación parabólica del log de la PSD; amplitud rms integrando ±3 bins menos el piso |
| Espectrograma | tramos de 256 muestras (~3,1 Hz de resolución), paso 64 |

Ejes por separado y su suma (energía total sin depender de la orientación).
No se usa |a|: no es lineal y crea armónicas falsas.

### Herramientas

    cd pc
    python -m unittest test_vibrascan_espectro -v                  (sin placa)
    python espectro.py capturas/captura_<fecha>                    (PSD y tabla de picos)
    python espectro.py capturas/captura_<fecha> --espectrograma --desde 10 --hasta 30 --png f.png
    python etapa5_prueba.py capturas/captura_<fecha> --frecuencia 250

`espectro.py` marca los picos que son armónicas del principal, directas o
**reflejadas por aliasing**, y avisa si la señal no es estacionaria (Parseval
< 0,95, por ejemplo con golpes): ahí los "picos" pueden ser ondulaciones de
una resonancia ancha y conviene mirar el espectrograma.

`etapa5_prueba.py` (una captura por tono, empezando con unos segundos de
silencio): sin pérdidas; el tono aparece en el espectrograma después de ≥ 2 s
de silencio (descarta un pico del ambiente); PSD desde el encendido; pico más
fuerte a ±0,5 Hz de la frecuencia (o de su alias si está por encima de fs/2);
≥ 20 dB sobre el piso; Parseval ±2 %.

### Resultados (2026-10-04, parlante de celular con un generador de tonos)

| Tono | Pico | Error | Sobre el piso | Amplitud | Tasa medida |
|---|---|---|---|---|---|
| 150 Hz | 150,00 Hz | < 0,01 Hz | 37,4 dB | 22 mg rms | 794,44 Hz |
| 250 Hz | 249,98 Hz | −0,02 Hz | 40,2 dB | 29 mg rms | 793,54 Hz |
| 350 Hz | 349,97 Hz | −0,03 Hz | 44,3 dB | 44 mg rms | 793,69 Hz |
| 500 Hz (> fs/2) | 293,34 Hz (alias de 793,30 − 500) | +0,04 Hz | 53,1 dB | 141 mg rms | 793,30 Hz |

Todas aprobadas (silencio previo, sin pérdidas, Parseval dentro de ±1,1 %).
Reposo de 30 min: sin picos, Parseval 1,000, piso 265/303/322 µg/√Hz
(x/y/z), coherente con el rms de la etapa 4.

Lo que mostraron:

- **Tasa medida, no nominal.** El sensor muestreó entre 793,3 y 796,3 Hz
  según la captura. Con 800 Hz el tono de 350 Hz habría salido en ~352,8 Hz.
- **Aliasing.** El parlante distorsiona: sus armónicas 2 y 3 (500, 700,
  750 Hz) aparecen reflejadas por debajo de fs/2 (295, 94, 45 Hz). A 350 Hz,
  en el eje y el pico más fuerte era la armónica 2 reflejada (93,8 Hz), no el
  tono. Un tono de 500 Hz apareció en 293,3 Hz con 141 mg, más que cualquier
  tono dentro de la banda: a 800 Hz de ODR el filtro del sensor casi no
  atenúa por encima de fs/2. **A 800 Hz solo son confiables fuentes sin
  contenido importante por encima de ~397 Hz**; si no, un pico puede ser una
  frecuencia más alta reflejada. La etapa 6 (SPI a 3200 Hz) lleva el límite
  a ~1600 Hz.
- **Golpes.** Cada golpe excita todas las frecuencias (línea vertical en el
  espectrograma) y deja una oscilación de la mesa/protoboard de ~7–25 Hz
  durante ~0,5 s (en y, máximo en ~7,6 Hz).
- **Pregunta abierta.** Con el tono prendido aparecen picos laterales
  (9,4 Hz de separación a 150 Hz; 15,6 Hz a 250 y 500 Hz). No coinciden con
  ninguna frecuencia del sistema (bloques a ~24,8 Hz, estado a 1 Hz);
  probablemente vienen de la fuente.

## Etapa 4 - captura a archivo y gráfico en tiempo (aprobada, tag `etapa-4`)

Aprobada el 2026-10-04 en la placa:

- **Reposo, 30 min a 800 Hz y ±4 g** (`captura_20261004_024609`): 1 433 088
  muestras en un solo segmento, 0 pérdidas, contadores sin cambios. Deriva
  de la media por minuto 1,0/1,1/1,0 mg (x/y/z; un leve asentamiento en los
  primeros ~7 min, probablemente térmico), ruido 5,3/6,1/6,5 mg rms sin
  ningún minuto fuera de ±20 %, |a| = 1,017 g. Periodo por regresión
  1256,03 µs (796,2 Hz): el oscilador del sensor cambió 0,16 % respecto de
  la etapa 3 (otra ubicación/temperatura), así que el análisis debe usar la
  tasa medida de cada captura y no 800 Hz.
- **Golpes, 30 s a ±16 g** (`captura_20261004_032553`): 5 de 5, separados
  1,8–2,1 s, picos de 1,9–2,5 g, 14–19 ms sobre el umbral, ancho a media
  altura 3,8–7,5 ms, vuelta al reposo en 31–134 ms. Después de cada golpe la
  mesa/protoboard oscila ~0,5 s (se va a ver en la FFT de la etapa 5).
- `rango=` en los cuatro valores, descarte de asentamiento verificado,
  regresión de la etapa 3 a 100/400/800 Hz, y monitor que retoma solo al
  desconectar y reconectar el USB (la captura queda con 2 segmentos).

Un primer intento de golpes falló "Golpes cortos" (57–138 ms): eran golpes
más fuertes (hasta 15,2 g, a 0,8 g del tope de ±16 g) con **rebote** (un
segundo impacto ~80 ms después). El detector junta impactos a menos de
0,2 s, así que la duración medía el rebote; cada impacto por separado medía
~5 ms. Si hace falta caracterizar golpes con rebote, la medida adecuada es el
ancho del pico a media altura. A 800 Hz un impacto de ~5 ms son 4 muestras:
el pico real puede ser mayor que el muestreado (la etapa 6, a 3200 Hz, lo
resuelve mejor).

Sobre la etapa 3. Casi todo es de PC; en el firmware solo se agregó el
comando `rango=` y el descarte de asentamiento.

### Capturar

    cd pc
    python captura.py --port COMx --minutos 30 --nota "reposo sobre la mesa"
    python captura.py --port COMx --segundos 20 --rango 16 --nota "5 golpes"

Escribe `pc/capturas/captura_<fecha>.csv` y `.json` mientras captura (el CSV
se vuelca cada 1 s, el JSON cada 10 s): si el programa se corta, lo grabado
queda. Si el USB se desconecta, sigue intentando y retoma sola. Ctrl+C termina
antes de tiempo. `monitor_vivo.py` graba en el mismo formato
(`monitor_<fecha>.csv/.json`) mientras grafica.

CSV (una fila por muestra, ~85 MB por hora a 800 Hz):

| Columna | Contenido |
|---|---|
| `seg` | segmento: muestras contiguas con ODR y rango fijos. Cambia en cada hueco, pérdida, reconexión o cambio de ODR/rango; un análisis (FFT) no debe cruzar segmentos |
| `n` | índice del firmware (vuelve a 0 si la placa se reinicia) |
| `t_s` | s desde el inicio: reloj del micro dentro de una conexión (`t0` del bloque + i × periodo); entre conexiones se alinea con el reloj de la PC (~10 ms) |
| `x`, `y`, `z` | cuentas crudas; g = cuentas × 0,0039 |

El JSON tiene fecha, nota, ODR, rango, escala, límites de saturación, periodo
por regresión de `t0` contra `n`, segmentos (con su motivo), eventos,
pérdidas y contadores del firmware al inicio y al final.

### Ver

    python ver_captura.py capturas/captura_<fecha>
    python ver_captura.py capturas/captura_<fecha> --desde 100 --hasta 160
    python ver_captura.py capturas/captura_<fecha> --png figura.png

Tres ejes y |a| contra el tiempo, con zoom. Si hay más muestras que píxeles,
cada píxel muestra el mínimo y el máximo de sus muestras: ningún pico
desaparece, por larga que sea la captura (el monitor dibuja igual). Huecos y
reconexiones en gris (la línea no los cruza), muestras saturadas en rojo y
golpes marcados. Por consola: resumen, estadística por minuto y golpes.

### Rango

`rango=2|4|8|16` (o `--rango` en `captura.py` y el monitor). En FULL_RES la
escala es 3,9 mg/cuenta en cualquier rango; cambia el tope (±512 cuentas a
2 g, ±1024 a 4 g, ±2048 a 8 g, ±4096 a 16 g). Para golpes usar 16 g: a 4 g
un golpecito en la mesa ya recorta.

### Asentamiento del sensor

Medido en esta etapa: después de configurar el sensor (al arrancar, con
`odr=`, `rango=` o al reconectar el bus) la primera muestra sale desviada
~0,12 g. El firmware descarta las 2 primeras (`SETTLE_SAMPLES`): no suman a `n`
ni dejan hueco; el contador informativo `asent` del estado las cuenta. Pasaba
desde la etapa 2 con `odr=`.

### Probar

    python -m unittest test_vibrascan_proto test_vibrascan_archivo -v   (sin placa)
    python etapa4_prueba.py reposo capturas/captura_<fecha>   (30 min quieto)
    python etapa4_prueba.py golpes capturas/captura_<fecha>   (5 golpes a 16 g)

`etapa4_prueba.py` analiza un archivo (no captura): se puede volver a evaluar
sin repetir la captura. Umbrales (constantes al principio del archivo):

| Prueba | Chequeos |
|---|---|
| reposo | captura completa de 30 min; sin pérdidas y un solo segmento; contadores sin cambios; sin saturación; deriva de la media por minuto ≤ 10 mg por eje; rms de cada minuto a ±20 % de la mediana; 1 g ± 10 %; periodo por regresión dentro de `tmin`–`tmax` del firmware |
| golpes | sin pérdidas; sin saturación; 5 golpes detectados; cada uno ≤ 100 ms sobre el umbral de 0,5 g; vuelve al reposo (±0,1 g durante 50 ms) en ≤ 500 ms |

## Etapa 3 - buffer por bloques y trama binaria con checksum (aprobada, tag `etapa-3`)

Aprobada el 2026-10-04 en la placa. `etapa3_prueba.py` sin fallas a 100, 400
y 800 Hz (20 s cada uno) y 10 minutos a 800 Hz: 476 800 muestras en 15 503
tramas, 0 CRC mal, 0 tramas perdidas, 0 muestras descartadas, 0 huecos,
contadores sin cambios, 794,57 muestras/s, ~5,4 kB/s. Pruebas manuales con
`monitor_vivo.py`: curva de 10 s completa, ±1 g en cinco caras, golpes de
hasta 4,6 g sin cortes, cuatro cortes de INT1 de 2,4 a 4,9 s informados
como `HUECO` con su duración (y `ovr` +1 cada uno) con recuperación sola,
reapertura y reconexión del USB con arranque limpio. Una pausa de 0,7 s de
la PC quedó absorbida por el buffer del micro sin perder muestras.

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
stall= bfull= hueco= int1= sps= tmin= tmax=` (desde la etapa 4 además
`rango=` después de `odr` y `asent=` después de `hueco`; ver la etapa 4 y el
comando `rango=`). Los contadores son acumulados
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

Desde la etapa 6 (SPI de 4 hilos; módulo GY-291):

| ADXL345 (GY-291) | Black Pill | Función |
|---------|------------|---|
| VCC     | 3V3        | con 100 nF y 10 µF a GND junto al módulo |
| GND     | GND        | |
| CS      | PA4        | selección (GPIO) |
| SCL     | PA5        | SCLK |
| SDA     | PA7        | SDI (MOSI) |
| SDO     | PA6        | SDO (MISO, con pull-up interno: sin sensor se lee 0xFF) |
| INT1    | PB0        | watermark del FIFO (desde la etapa 2: DATA_READY) |

Hasta la etapa 5 (I2C): SDA → PB7, SCL → PB6, CS → 3V3, SDO → GND
(dirección 0x53), INT1 → PB0.

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
    Core/Src/adxl345.c     driver del sensor, independiente del bus (SPI desde la etapa 6)
    Core/Src/sensor_spi.c  bus SPI del sensor: pines, SPI1, DMA y CS (etapa 6)
    Core/Src/trama.c       tramas binarias con CRC (no conoce el transporte)
    Core/Src/usb_link.c    envío a la PC por USB CDC
    USB_DEVICE/, Middlewares/, Drivers/   pila USB y HAL de ST
    pc/vibrascan_proto.py  protocolo del lado PC: decodificador y verificador de continuidad
    pc/test_vibrascan_proto.py  pruebas del protocolo sin placa
    pc/etapa3_prueba.py    prueba de aceptación de la etapa 3
    pc/vibrascan_captura.py  motor de captura a archivo (reconexión, segmentos, JSON)
    pc/captura.py          captura sin ventana
    pc/vibrascan_archivo.py  lectura y análisis de capturas (por minuto, golpes, saturación)
    pc/ver_captura.py      visor de capturas
    pc/etapa4_prueba.py    prueba de aceptación de la etapa 4 (analiza archivos)
    pc/test_vibrascan_archivo.py  pruebas de lectura y análisis sin placa
    pc/vibrascan_espectro.py  PSD por Welch (numpy), picos, aliasing, espectrograma
    pc/espectro.py         espectro y espectrograma de una captura
    pc/etapa5_prueba.py    prueba de aceptación de la etapa 5 (tono de frecuencia conocida)
    pc/test_vibrascan_espectro.py  pruebas del espectro sin placa
    pc/monitor_vivo.py     gráfico en tiempo real y registro de la sesión
    test_fw/test_adxl345.c prueba del driver en la PC contra un sensor simulado (gcc)
    pc/etapa2_prueba.py    prueba de la etapa 2 (no sirve con el firmware de la 3;
                           la 3 usa sus constantes y funciones)

## Próximas etapas

| # | Etapa | Se aprueba cuando |
|---|---|---|
| 2 | Muestreo por interrupción DATA_READY (INT1 a un pin EXTI) y ODR más alto | muestras/s = ODR, sin saltos (aprobada) |
| 3 | Buffer por bloques y trama binaria con checksum | minutos sin pérdidas (aprobada) |
| 4 | Captura a archivo y gráfico en tiempo | reposo estable, golpe visible (aprobada) |
| 5 | Preproceso y FFT/PSD en Python | pico en la frecuencia de una fuente conocida (aprobada) |
| 6 | SPI a 3200 Hz (con el FIFO del ADXL345) | igual que 2 y 3, y un tono de 500 Hz en 500 Hz (aprobada) |
| 7 | FFT y filtro adaptativo (LMS) en el micro | mismo resultado que en Python |

## Sobre VibraScan_v2.ioc

Refleja la configuración hasta la etapa 5 (I2C1 a 400 kHz, INT1 en PB0 con
EXTI0, USB CDC, LED en PC13, 96 MHz). **SPI1 y su DMA no están en el `.ioc`**:
se configuran en `sensor_spi.c` (desde la etapa 6), y `main.c` sigue
inicializando I2C1 en PB6/PB7, que quedan sin conectar. Fue editado a mano y no se probó regenerando desde
CubeMX. Antes de usar "Generate Code", hacer un commit: los archivos de
USB_DEVICE no tienen bloques USER CODE y se sobrescriben (se recuperan con
`git checkout -- USB_DEVICE`).
