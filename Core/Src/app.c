/**
  * @file    app.c
  * @brief   VibraScan - Etapa 6: ADXL345 por SPI con FIFO, hasta 3200 Hz.
  *
  * Que hace:
  *   - Configura el sensor por SPI (sensor_spi.c), verifica DEVID y pone su
  *     FIFO en modo stream con la interrupcion watermark por INT1 -> PB0 al
  *     juntar fifo_wm muestras (comando fifo=).
  *   - Cada flanco de INT1 (EXTI) toma la hora y lanza un lote de lecturas
  *     por DMA, encadenadas en las interrupciones: FIFO_STATUS (cuantas
  *     muestras hay) y despues cada muestra (DATAX0..DATAZ1). La muestra
  *     fifo_wm - 1 del lote es la del flanco; las demas se ubican con
  *     el periodo nominal. Si al terminar INT1 sigue en alto (el watermark es
  *     por nivel y no daria otro flanco) se lanza otro lote enseguida.
  *   - Cada muestra va al bloque en curso. Un bloque tiene hasta
  *     BLOCK_SAMPLES muestras contiguas; se cierra al llenarse, ante un hueco
  *     (mas de 1,5 periodos entre muestras o FIFO lleno) o al cambiar el ODR.
  *   - app_loop() manda cada bloque cerrado como una trama binaria con CRC
  *     (ver trama.h y pc/vibrascan_proto.py), varias tramas por envio USB.
  *   - No se manda nada hasta que la PC envia "start". Con "stop", o si el
  *     USB se desconecta, deja de mandar.
  *   - Una vez por segundo lee DEVID: SPI no tiene acuse de recibo, y un
  *     sensor desconectado se leeria como datos (0x00 o 0xFF) sin error.
  *   - Mientras manda, una trama de estado por segundo (texto).
  *
  * El unico reloj de muestreo sigue siendo el del ADXL345.
  *
  * Contadores de la trama de estado (todos deberian quedarse en 0):
  *   ovr      lotes en los que el FIFO estaba lleno (se perdieron muestras)
  *   bus_err  transferencias SPI fallidas, FIFO_STATUS imposible o DEVID
  *            incorrecto en el chequeo periodico
  *   drop     muestras que no se pudieron mandar a la PC
  *   stall    veces que INT1 quedo en alto sin lectura en curso y hubo que
  *            relanzarla desde app_loop (se perdio un flanco)
  *   bfull    muestras descartadas porque el buffer estaba lleno (la PC o el
  *            USB no llegan a llevarse los datos)
  *   hueco    veces que pasaron mas de 1,5 periodos entre dos muestras
  *
  * Informativo: asent = muestras descartadas tras configurar el sensor
  * (SETTLE_SAMPLES cada vez: al arrancar, con odr=, rango= o al reconectar).
  *
  * Campos informativos (del ultimo segundo):
  *   fifo     muestras por aviso de INT1 (fifo_wm)
  *   int1     flancos de INT1 (~ odr / fifo)
  *   sps      muestras leidas (= odr)
  *   tmin     intervalo minimo entre flancos de INT1, en us (reloj del micro)
  *   tmax     intervalo maximo (~ fifo x periodo; mucho mas = se demoro un lote)
  *
  * Comandos de la PC (una linea de texto; la respuesta va en una trama):
  *   start     vacia el buffer, reinicia seq y empieza a mandar
  *   stop      deja de mandar (el sensor sigue midiendo)
  *   odr=<Hz>  cambia el ODR: 100, 200, 400, 800, 1600 o 3200 (por defecto 800)
  *   rango=<g> cambia el rango: 2, 4, 8 o 16 g (por defecto 4). En FULL_RES la
  *             escala es 3,9 mg/cuenta en cualquier rango; cambia el tope.
  *             Si se manda mientras se transmite, la reconfiguracion deja un
  *             hueco real de unos ms y se informa como tal.
  *   fifo=<n>  muestras por aviso de INT1, 1 a 31 (por defecto FIFO_WM_DEFAULT).
  *             Reconfigura el sensor como rango=.
  *   spi=<kHz> cambia el reloj del SPI: 3000, 1500, 750 o 375 (por defecto
  *             3000). Para estudiar el acople de la lectura en la medicion;
  *             reconfigura el sensor como rango= (deja un hueco de unos ms).
  *
  * LED (PC13): parpadeo lento = midiendo, parpadeo rapido = sensor no responde.
  */
#include "app.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "adxl345.h"
#include "sensor_spi.h"
#include "trama.h"
#include "usb_link.h"

/* ---- Configuracion de la etapa ------------------------------------------ */
#define APP_STAGE             6
#define APP_RATE_DEFAULT      ADXL345_RATE_800HZ   /* se cambia con el comando odr= */
#define APP_RATE_MAX          ADXL345_RATE_3200HZ
#define APP_RANGE_DEFAULT     ADXL345_RANGE_4G     /* se cambia con el comando rango= */

#define FIFO_WM_DEFAULT       2U      /* muestras por aviso de INT1; se cambia con fifo= (ver abajo) */
/* Medido en la etapa 6: si hay actividad en el SPI mientras el sensor
   convierte una muestra, esa muestra sale desviada en Z (hasta ~140 mg a
   3200 Hz). Cada muestra leida ocupa ~30 us de bus; el lote tiene que
   terminar antes de la conversion siguiente (312 us a 3200 Hz). Con 8 o 16
   por aviso no termina; con 1, 2 o 4 los datos salen limpios. 2 deja ~235 us
   de margen ante demoras con la mitad de interrupciones que 1. No subir este
   valor sin volver a medir (comando fifo=). */
#define BATCH_EXTRA_MAX       3U      /* un lote normal trae a lo sumo el aviso + 3 muestras */
#define STOP_TIMEOUT_MS       10U
#define STATUS_PERIOD_MS      1000U
#define RETRY_PERIOD_MS       500U
#define ERROR_BLINK_MS        100U
#define MAX_CONSECUTIVE_ERR   10U

#define BLOCK_SAMPLES         32U     /* muestras por bloque = por trama (10 ms a 3200 Hz) */
#define BLOCK_COUNT           128U    /* bloques; potencia de 2 (4096 muestras, 1,3 s a 3200 Hz) */
#define TX_BATCH_SIZE         512U    /* bytes por envio USB (<= USB_LINK_MAX_WRITE): 2 tramas */
#define TEXT_MAX              288U    /* texto de estado o respuesta (el estado ocupa hasta 256) */
#define SETTLE_SAMPLES        2U      /* muestras descartadas tras configurar el sensor */

_Static_assert(TX_BATCH_SIZE <= USB_LINK_MAX_WRITE, "TX_BATCH_SIZE mayor que lo que acepta usb_link");
_Static_assert((BLOCK_COUNT & (BLOCK_COUNT - 1U)) == 0U, "BLOCK_COUNT debe ser potencia de 2");
_Static_assert(BLOCK_SAMPLES <= 255U, "cant es de 8 bits");
_Static_assert((FIFO_WM_DEFAULT > 0U) && (FIFO_WM_DEFAULT < ADXL345_FIFO_SIZE), "watermark de 1 a 31");
_Static_assert(ADXL345_DATA_LEN <= SENSOR_SPI_MAX_READ, "una muestra no entra en una lectura");

/* ---- Bloque de muestras -------------------------------------------------- */

/* Tiene la misma forma que el contenido de la trama de muestras (el micro y
   la PC son little-endian): la trama se arma con un memcpy de los primeros
   BLOCK_HEADER_LEN + 6 x cant bytes. */
typedef struct {
  uint32_t         n0;      /* indice de la primera muestra */
  uint32_t         t0;      /* us del micro de la primera muestra */
  uint16_t         odr;     /* Hz */
  uint8_t          flags;   /* TRAMA_FLAG_* */
  uint8_t          cant;    /* muestras cargadas */
  adxl345_sample_t s[BLOCK_SAMPLES];
} block_t;

#define BLOCK_HEADER_LEN      12U
_Static_assert(offsetof(block_t, t0) == 4U && offsetof(block_t, odr) == 8U &&
               offsetof(block_t, flags) == 10U && offsetof(block_t, cant) == 11U &&
               offsetof(block_t, s) == BLOCK_HEADER_LEN, "block_t no coincide con la trama");
_Static_assert(sizeof(block_t) == BLOCK_HEADER_LEN + 6U * BLOCK_SAMPLES, "block_t tiene relleno");
_Static_assert(2U * (sizeof(block_t) + TRAMA_EXTRA) <= TX_BATCH_SIZE, "no entran 2 tramas por envio");
_Static_assert(TEXT_MAX + TRAMA_EXTRA <= USB_LINK_MAX_WRITE, "la trama de texto no entra en un envio");

/* ---- Estado --------------------------------------------------------------- */
static adxl345_t        sensor;
static adxl345_status_t sensor_status = ADXL345_ERR_BUS;
static adxl345_rate_t   app_rate = APP_RATE_DEFAULT;
static adxl345_range_t  app_range = APP_RANGE_DEFAULT;
static uint8_t          fifo_wm = FIFO_WM_DEFAULT;   /* muestras por aviso de INT1 */

/* Compartido con las interrupciones (EXTI y DMA del SPI) */
static volatile bool     sampling;        /* true = las interrupciones pueden leer */
static volatile bool     rx_busy;         /* el bus esta reservado (lote o chequeo) */
static volatile bool     bus_lost;        /* demasiados errores seguidos */
static volatile uint32_t t_last_read;     /* HAL_GetTick() del ultimo lote completo */

/* Lote en curso: muestras leidas del FIFO en un aviso de INT1 */
static volatile uint32_t batch_t_us;      /* hora del flanco (o del inicio, sin flanco) */
static volatile uint8_t  batch_count;     /* muestras a leer */
static volatile uint8_t  batch_idx;       /* proxima a leer */
static volatile uint8_t  batch_ref;       /* indice de la muestra que corresponde a batch_t_us */
static volatile bool     batch_overrun;   /* el FIFO estaba lleno: faltan muestras antes */
static uint32_t          period_ns;       /* periodo nominal */
static uint32_t          stall_ms;        /* sin lotes por mas que esto con INT1 en alto = stall */

/* Bloques: las interrupciones cargan blocks[head] y al cerrarlo avanzan
   head; app_loop manda desde tail. Un bloque recien es visible para app_loop
   cuando esta cerrado. */
static block_t           blocks[BLOCK_COUNT];
static volatile uint32_t blk_head;
static volatile uint32_t blk_tail;
static bool              blk_open;        /* blocks[head] tiene muestras */
static uint8_t           blk_next_flags;  /* flags para el proximo bloque que se abra */
static uint16_t          blk_odr;         /* Hz, para los bloques */
static uint32_t          gap_limit_us;    /* 1,5 periodos */
static bool              have_last;       /* hay una muestra anterior para medir huecos */
static uint32_t          last_t_us;

static volatile uint32_t n_samples;       /* muestras leidas; es el indice que se envia */
static volatile uint32_t n_overrun;
static volatile uint32_t n_bus_err;
static volatile uint32_t n_buf_full;
static volatile uint32_t n_gap;
static volatile uint32_t n_settle;        /* muestras descartadas por asentamiento */
static volatile uint32_t settle_skip;     /* cuantas faltan descartar */
static volatile uint32_t n_int1_edges;
static volatile uint32_t consecutive_err;
static uint32_t n_drop;
static uint32_t n_stall;
static uint32_t int1_edges_prev;
static uint32_t n_samples_prev;
static uint32_t st_int1, st_sps, st_tmin_us, st_tmax_us;   /* ultimo segundo completo */

/* Intervalo entre flancos de INT1 medido con el contador de ciclos (DWT), que
   corre con el reloj del micro (cristal de 25 MHz). Min y max por segundo. */
static volatile uint32_t edge_cyc_prev;
static volatile bool     edge_have_prev;
static volatile uint32_t period_min_cyc;
static volatile uint32_t period_max_cyc;

/* Reloj en us a partir de DWT (que da la vuelta cada 44,7 s a 96 MHz): se
   acumula en cada llamada a clock_us(), que app_loop hace al menos una vez
   por segundo. Da la vuelta cada 71 minutos; la PC lo corrige. */
static uint32_t          cyc_per_us;
static volatile uint32_t clk_cyc_prev;
static volatile uint32_t clk_rem;
static volatile uint32_t clk_us;

/* Envio */
static bool     streaming;                /* la PC mando start */
static uint16_t tx_seq;
static uint8_t  tx_batch[TX_BATCH_SIZE];

static uint32_t t_status;
static uint32_t t_retry;
static uint32_t t_blink;

static const adxl345_bus_t spi_bus = { sensor_spi_read, sensor_spi_write, NULL };

/* Diagnostico por SWD (ver check_devid) */
volatile uint8_t diag_fifo_status;
volatile uint8_t diag_int_source;

/* ---- Reloj en us ---------------------------------------------------------- */

static uint32_t clock_us(void)
{
  uint32_t primask = __get_PRIMASK();
  uint32_t now, us;

  __disable_irq();
  now = DWT->CYCCNT;
  clk_rem += now - clk_cyc_prev;
  clk_cyc_prev = now;
  clk_us += clk_rem / cyc_per_us;
  clk_rem %= cyc_per_us;
  us = clk_us;
  __set_PRIMASK(primask);
  return us;
}

/* ---- Bloques (los llaman las interrupciones, o app_loop con el muestreo
        detenido) ---------------------------------------------------------- */

static void close_block(void)
{
  if (blk_open)
  {
    blk_open = false;
    blk_head = blk_head + 1U;          /* recien ahora lo ve app_loop */
  }
}

static void store_sample(const adxl345_sample_t *s, uint32_t t_us, bool overrun)
{
  block_t *b;

  if (!blk_open)
  {
    if ((blk_head - blk_tail) >= BLOCK_COUNT)
    {
      n_buf_full++;                    /* sin lugar: n salta y la PC lo ve */
      return;
    }
    b = &blocks[blk_head & (BLOCK_COUNT - 1U)];
    b->n0 = n_samples;
    b->t0 = t_us;
    b->odr = blk_odr;
    b->flags = blk_next_flags;
    b->cant = 0;
    blk_next_flags = 0;
    blk_open = true;
  }

  b = &blocks[blk_head & (BLOCK_COUNT - 1U)];
  b->s[b->cant] = *s;
  b->cant++;
  if (overrun)
  {
    b->flags |= TRAMA_FLAG_OVR;
  }
  if (b->cant >= BLOCK_SAMPLES)
  {
    close_block();
  }
}

/* Empieza de cero: sin bloques pendientes y sin muestra anterior (asi el
   salto de tiempo de un start o de un cambio de ODR no cuenta como hueco).
   Solo con el muestreo detenido o sin interrupciones. */
static void reset_blocks(uint8_t next_flags)
{
  blk_open = false;
  blk_head = 0;
  blk_tail = 0;
  blk_next_flags = next_flags;
  have_last = false;
}

/* ---- Lectura del FIFO por DMA --------------------------------------------- */

/* Reserva el bus. Puede llamarse desde la EXTI, desde el fin de un lote o
   desde app_loop, por eso la consulta y la marca se hacen sin interrupciones. */
static bool claim_bus(void)
{
  bool ok;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  ok = sampling && !rx_busy;
  if (ok)
  {
    rx_busy = true;
  }
  __set_PRIMASK(primask);
  return ok;
}

static void read_error(void)
{
  n_bus_err++;
  if (++consecutive_err >= MAX_CONSECUTIVE_ERR)
  {
    bus_lost = true;                   /* app_loop vuelve a buscar el sensor */
  }
}

static void on_fifo_status(const uint8_t *data, uint16_t len);
static void on_sample(const uint8_t *data, uint16_t len);

/* Empieza un lote. from_edge: lo lanzo un flanco de INT1 (t_us es la hora de
   la muestra fifo_wm - 1); si no, t_us es la hora de la ultima. */
static void start_batch(uint32_t t_us, bool from_edge)
{
  if (!claim_bus())
  {
    return;
  }
  batch_t_us = t_us;
  batch_ref = from_edge ? (uint8_t)(fifo_wm - 1U) : 0xFFU;
  if (!sensor_spi_read_async(ADXL345_REG_FIFO_STATUS, 1U, on_fifo_status))
  {
    rx_busy = false;
    read_error();
  }
}

/* Suelta el bus. Si INT1 sigue en alto (el watermark es por nivel: no habra
   otro flanco) se lanza otro lote enseguida. */
static void release_bus(void)
{
  rx_busy = false;
  if (sampling && (HAL_GPIO_ReadPin(ADXL_INT1_GPIO_Port, ADXL_INT1_Pin) == GPIO_PIN_SET))
  {
    start_batch(clock_us(), false);
  }
}

static void on_fifo_status(const uint8_t *data, uint16_t len)
{
  uint8_t entries;

  (void)len;
  if (data == NULL)
  {
    rx_busy = false;
    read_error();
    return;
  }
  entries = (uint8_t)(data[0] & 0x3FU);
  if (entries > ADXL345_FIFO_MAX_ENTRIES)
  {
    rx_busy = false;                   /* imposible: lectura corrupta */
    read_error();
    return;
  }
  if (entries == 0U)
  {
    t_last_read = HAL_GetTick();
    release_bus();
    return;
  }
  batch_overrun = (entries >= ADXL345_FIFO_SIZE);
  if (batch_overrun)
  {
    n_overrun++;                       /* el FIFO se lleno: se perdieron muestras */
  }
  /* Hora de las muestras del lote (medido en la etapa 6 al reconectar INT1):
     - FIFO lleno o sin muestra anterior: faltan muestras (no se sabe
       cuantas) o no hay referencia; la ultima es de ahora.
     - Flanco de un aviso recien cumplido: la muestra fifo_wm - 1 es la del
       flanco (exacto).
     - Cualquier otro lote (sin flanco, o un flanco con el FIFO muy lleno):
       el FIFO no pierde muestras salvo que se llene, asi que cada una va un
       periodo despues de la anterior. "Ahora" podria errar hasta un periodo
       y simular un hueco. */
  if (batch_overrun || !have_last)
  {
    batch_ref = (uint8_t)(entries - 1U);
  }
  else if ((batch_ref == 0xFFU) || (entries > (uint8_t)(fifo_wm + BATCH_EXTRA_MAX)))
  {
    batch_t_us = last_t_us + (period_ns + 500U) / 1000U;
    batch_ref = 0;
  }
  batch_count = entries;
  batch_idx = 0;
  if (!sensor_spi_read_async(ADXL345_REG_DATAX0, ADXL345_DATA_LEN, on_sample))
  {
    rx_busy = false;
    read_error();
  }
}

static void on_sample(const uint8_t *data, uint16_t len)
{
  adxl345_sample_t s;
  uint32_t t;
  uint8_t idx = batch_idx;

  (void)len;
  if (data == NULL)
  {
    rx_busy = false;
    read_error();
    return;
  }
  adxl345_decode_xyz(data, &s);
  /* En ns y dividiendo al final: a 3200 Hz el periodo es 312,5 us y redondearlo
     antes perderia 0,5 us por muestra. Entra en 32 bits: 33 x 10 ms (100 Hz). */
  t = batch_t_us + (uint32_t)((((int32_t)idx - (int32_t)batch_ref) * (int32_t)period_ns) / 1000);

  /* Hueco: mas de 1,5 periodos desde la muestra anterior, o FIFO lleno (no
     se sabe cuantas faltan; la PC lo calcula con t0). El bloque en curso se
     cierra para que cada bloque tenga solo muestras contiguas. */
  if (have_last && ((((int32_t)(t - last_t_us)) > (int32_t)gap_limit_us) || (batch_overrun && (idx == 0U))))
  {
    n_gap++;
    close_block();
    blk_next_flags |= TRAMA_FLAG_HUECO;
  }
  have_last = true;
  last_t_us = t;

  /* Recien configurado, el sensor da la primera muestra desviada (~0,12 g,
     medido en la etapa 4): se descartan SETTLE_SAMPLES. No suman a n (para
     la PC no existieron) pero si cuentan como ultima muestra, asi la
     siguiente no parece un hueco. */
  if (settle_skip > 0U)
  {
    settle_skip--;
    n_settle++;
  }
  else
  {
    store_sample(&s, t, batch_overrun && (idx == 0U));
    n_samples++;
  }

  batch_idx = (uint8_t)(idx + 1U);
  if (batch_idx < batch_count)
  {
    if (!sensor_spi_read_async(ADXL345_REG_DATAX0, ADXL345_DATA_LEN, on_sample))
    {
      rx_busy = false;
      read_error();
    }
    return;
  }
  consecutive_err = 0;
  t_last_read = HAL_GetTick();
  release_bus();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADXL_INT1_Pin)
  {
    uint32_t now = DWT->CYCCNT;

    if (edge_have_prev)
    {
      uint32_t period = now - edge_cyc_prev;
      if (period < period_min_cyc) period_min_cyc = period;
      if (period > period_max_cyc) period_max_cyc = period;
    }
    edge_cyc_prev = now;
    edge_have_prev = true;

    n_int1_edges++;
    start_batch(clock_us(), true);
  }
}

/* Red de seguridad: el watermark es por nivel. Si un lote falla, INT1 queda
   en alto, no hay mas flancos y el muestreo se detiene. Aca se detecta y se
   relanza. */
static void check_stall(uint32_t now)
{
  if (!rx_busy &&
      (HAL_GPIO_ReadPin(ADXL_INT1_GPIO_Port, ADXL_INT1_Pin) == GPIO_PIN_SET) &&
      /* con signo: la interrupcion puede haber escrito t_last_read despues
         de que app_loop leyo now, y la resta sin signo daria la vuelta */
      ((int32_t)(now - t_last_read) > (int32_t)stall_ms))
  {
    n_stall++;
    t_last_read = now;                 /* un relanzamiento por periodo de espera */
    start_batch(clock_us(), false);
  }
}

/* SPI no tiene acuse de recibo: un sensor desconectado se lee como datos.
   Una vez por segundo se confirma que sigue ahi. */
static void check_devid(void)
{
  uint8_t devid = 0;
  int st;

  if (!claim_bus())
  {
    return;                            /* hay un lote en curso: en el proximo segundo */
  }
  st = sensor_spi_read(NULL, ADXL345_REG_DEVID, &devid, 1U);
  if ((st != 0) || (devid != ADXL345_DEVID_VALUE))
  {
    sensor.devid = devid;
    n_bus_err++;
    bus_lost = true;                   /* app_loop detiene y vuelve a buscar el sensor */
  }
  else
  {
    /* Diagnostico (se lee por SWD): cuantas muestras tiene el FIFO y si el
       watermark esta activo adentro del sensor. Separa "el sensor no avisa"
       de "INT1 no llega a PB0". Leer INT_SOURCE no borra el watermark. */
    (void)sensor_spi_read(NULL, ADXL345_REG_FIFO_STATUS, (uint8_t *)&diag_fifo_status, 1U);
    (void)sensor_spi_read(NULL, ADXL345_REG_INT_SOURCE, (uint8_t *)&diag_int_source, 1U);
  }
  release_bus();
}

/* ---- Conexion con el sensor ---------------------------------------------- */

static void stop_sampling(void)
{
  uint32_t t0 = HAL_GetTick();

  HAL_NVIC_DisableIRQ(ADXL_INT1_EXTI_IRQn);
  sampling = false;
  while (rx_busy && ((HAL_GetTick() - t0) < STOP_TIMEOUT_MS))
  {
    /* espera a que termine el lote en curso (menos de 1 ms) */
  }
  if (rx_busy)
  {
    sensor_spi_abort();
  }
  rx_busy = false;
}

static void start_sampling(void)
{
  uint32_t hz = adxl345_rate_hz(app_rate);

  blk_odr = (uint16_t)hz;
  period_ns = 1000000000U / hz;
  gap_limit_us = 1500000U / hz;
  stall_ms = 2U * fifo_wm * 1000U / hz + 2U;
  settle_skip = SETTLE_SAMPLES;        /* el sensor se acaba de configurar */

  consecutive_err = 0;
  bus_lost = false;
  edge_have_prev = false;
  t_last_read = HAL_GetTick();
  /* start_fifo dejo el FIFO vacio (INT1 en bajo): se borra un flanco viejo */
  __HAL_GPIO_EXTI_CLEAR_IT(ADXL_INT1_Pin);
  sampling = true;
  HAL_NVIC_EnableIRQ(ADXL_INT1_EXTI_IRQn);
}

static void sensor_connect(void)
{
  sensor_status = adxl345_init(&sensor, &spi_bus, app_rate, app_range);
  if (sensor_status == ADXL345_OK)
  {
    sensor_status = adxl345_start_fifo(&sensor, fifo_wm);
  }
  if (sensor_status == ADXL345_OK)
  {
    start_sampling();
  }
  else
  {
    n_bus_err++;
  }
}

/* ---- Envio a la PC -------------------------------------------------------- */

/* Junta en tx_batch todas las tramas de bloques que entren (2 de 213 bytes)
   y las manda en un solo envio. Si el USB todavia esta ocupado con el envio
   anterior no se espera ni se descarta nada: los bloques quedan en el buffer
   y salen en una vuelta siguiente. seq avanza aunque el envio falle: la PC ve
   el salto y drop dice que fue el micro el que no pudo mandar. */
static void send_samples(void)
{
  uint32_t len = 0;
  uint32_t samples = 0;

  if (!usb_link_ready())
  {
    streaming = false;                 /* cable desconectado: hace falta otro start */
  }
  if (!streaming)
  {
    blk_tail = blk_head;               /* nadie escucha: se descarta sin contar */
    return;
  }
  if (!usb_link_can_write())
  {
    return;                            /* se reintenta en la proxima vuelta */
  }

  while (blk_tail != blk_head)
  {
    const block_t *b = &blocks[blk_tail & (BLOCK_COUNT - 1U)];
    uint16_t plen = (uint16_t)(BLOCK_HEADER_LEN + sizeof(adxl345_sample_t) * b->cant);

    if ((len + plen + TRAMA_EXTRA) > sizeof(tx_batch))
    {
      break;                           /* el resto va en el proximo envio */
    }
    len += trama_armar(TRAMA_MUESTRAS, tx_seq++, b, plen, &tx_batch[len]);
    samples += b->cant;
    blk_tail = blk_tail + 1U;
  }

  if ((len > 0U) && !usb_link_write(tx_batch, (uint16_t)len))
  {
    n_drop += samples;
  }
}

/* Manda una trama de texto (estado o respuesta). Solo se llama desde
   app_loop (nunca desde una interrupcion): los buffers pueden ser static y
   no ocupan pila. */
static void send_text(uint8_t tipo, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void send_text(uint8_t tipo, const char *fmt, ...)
{
  static char text[TEXT_MAX];
  static uint8_t frame[TEXT_MAX + TRAMA_EXTRA];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  if (n <= 0)
  {
    return;
  }
  if (n >= (int)sizeof(text))
  {
    n = (int)sizeof(text) - 1;         /* texto truncado */
  }
  (void)usb_link_write(frame, (uint16_t)trama_armar(tipo, tx_seq++, text, (uint16_t)n, frame));
}

static const char *status_text(adxl345_status_t st)
{
  switch (st)
  {
    case ADXL345_ERR_BUS:    return "error_spi";
    case ADXL345_ERR_DEVID:  return "devid_incorrecto";
    case ADXL345_ERR_VERIFY: return "registro_no_verifica";
    default:                 return "ok";
  }
}

/* Cierra el segundo: int1, sps, tmin y tmax de la trama de estado son los
   del ultimo segundo completo. Se llama cada segundo aunque no se este
   mandando, asi el estado que sale con start no acumula desde el arranque. */
static void update_rates(void)
{
  uint32_t edges = n_int1_edges;
  uint32_t samples = n_samples;
  uint32_t pmin, pmax;
  uint32_t primask = __get_PRIMASK();

  st_int1 = edges - int1_edges_prev;
  st_sps = samples - n_samples_prev;
  int1_edges_prev = edges;
  n_samples_prev = samples;

  __disable_irq();
  pmin = period_min_cyc;
  pmax = period_max_cyc;
  period_min_cyc = UINT32_MAX;
  period_max_cyc = 0;
  __set_PRIMASK(primask);
  if (pmin == UINT32_MAX)
  {
    pmin = 0;                          /* no hubo dos flancos en este segundo */
  }
  st_tmin_us = pmin / cyc_per_us;
  st_tmax_us = pmax / cyc_per_us;
}

/* Empieza una ventana nueva de 1 s para int1, sps, tmin y tmax (con start y
   al cambiar el ODR: si no, el primer segundo mezclaria periodos viejos). */
static void restart_rates(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  int1_edges_prev = n_int1_edges;
  n_samples_prev = n_samples;
  period_min_cyc = UINT32_MAX;
  period_max_cyc = 0;
  __set_PRIMASK(primask);
  t_status = HAL_GetTick();
}

static void send_status(void)
{
  if (sensor_status == ADXL345_OK)
  {
    send_text(TRAMA_ESTADO,
              "estado=OK etapa=%d bus=spi spi=%lu devid=0x%02X odr=%lu rango=%u fifo=%u n=%lu ovr=%lu bus_err=%lu drop=%lu stall=%lu bfull=%lu hueco=%lu asent=%lu int1=%lu sps=%lu tmin=%lu tmax=%lu",
              APP_STAGE, (unsigned long)sensor_spi_khz(), sensor.devid,
              (unsigned long)adxl345_rate_hz(app_rate), 2U << (unsigned)app_range,
              (unsigned)fifo_wm,
              (unsigned long)n_samples, (unsigned long)n_overrun,
              (unsigned long)n_bus_err, (unsigned long)n_drop,
              (unsigned long)n_stall, (unsigned long)n_buf_full, (unsigned long)n_gap,
              (unsigned long)n_settle,
              (unsigned long)st_int1, (unsigned long)st_sps,
              (unsigned long)st_tmin_us, (unsigned long)st_tmax_us);
  }
  else
  {
    send_text(TRAMA_ESTADO, "estado=ERROR etapa=%d bus=spi causa=%s devid=0x%02X bus_err=%lu",
              APP_STAGE, status_text(sensor_status), sensor.devid,
              (unsigned long)n_bus_err);
  }
}

/* ---- Comandos de la PC ---------------------------------------------------- */

/* Detiene el muestreo para poder tocar los bloques desde aca sin carreras;
   devuelve si estaba andando, para retomarlo. */
static bool pause_sampling(void)
{
  bool was = (sensor_status == ADXL345_OK);

  if (was)
  {
    stop_sampling();
  }
  return was;
}

static void cmd_start(void)
{
  /* El muestreo sigue andando: el reset va sin interrupciones. Si habia una
     lectura en curso, esa muestra es la primera del bloque de inicio. */
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  reset_blocks(TRAMA_FLAG_INICIO);
  __set_PRIMASK(primask);

  tx_seq = 0;
  streaming = true;
  send_text(TRAMA_RESPUESTA, "start ok");
  send_status();
  restart_rates();                     /* el proximo estado cubre 1 s desde start */
}

/* "odr=<Hz>": reconfigura el sensor con otro ODR (100 a 3200 Hz). */
static void cmd_odr(unsigned long hz)
{
  for (adxl345_rate_t r = ADXL345_RATE_100HZ; r <= APP_RATE_MAX; r++)
  {
    if (adxl345_rate_hz(r) == hz)
    {
      (void)pause_sampling();
      close_block();                   /* el bloque en curso queda con el ODR viejo */
      have_last = false;               /* el cambio no es un hueco */
      app_rate = r;
      sensor_connect();
      /* Todavia no hay un segundo completo con el ODR nuevo: hasta el
         proximo estado se informa 0 en vez de valores del ODR viejo. */
      restart_rates();
      st_int1 = st_sps = st_tmin_us = st_tmax_us = 0;
      send_text(TRAMA_RESPUESTA, "odr=%lu ok", hz);
      return;
    }
  }
  send_text(TRAMA_RESPUESTA, "odr=%lu no_valido (100, 200, 400, 800, 1600 o 3200)", hz);
}

/* "rango=<g>": reconfigura el sensor con otro rango (2, 4, 8 o 16 g). A
   diferencia de odr=, no se olvida la muestra anterior: el periodo no cambia
   y las muestras que no se leyeron durante la reconfiguracion faltan de
   verdad, asi que se informan como hueco. */
static void cmd_rango(unsigned long g)
{
  for (adxl345_range_t r = ADXL345_RANGE_2G; r <= ADXL345_RANGE_16G; r++)
  {
    if ((2UL << (unsigned)r) == g)
    {
      (void)pause_sampling();
      close_block();                   /* el bloque en curso queda con el rango viejo */
      app_range = r;
      sensor_connect();
      send_text(TRAMA_RESPUESTA, "rango=%lu ok", g);
      return;
    }
  }
  send_text(TRAMA_RESPUESTA, "rango=%lu no_valido (2, 4, 8 o 16)", g);
}

/* "fifo=<n>": muestras por aviso de INT1 (watermark del FIFO, 1 a 31). */
static void cmd_fifo(unsigned long n)
{
  if ((n == 0UL) || (n >= ADXL345_FIFO_SIZE))
  {
    send_text(TRAMA_RESPUESTA, "fifo=%lu no_valido (1 a 31)", n);
    return;
  }
  (void)pause_sampling();
  close_block();
  fifo_wm = (uint8_t)n;
  sensor_connect();
  send_text(TRAMA_RESPUESTA, "fifo=%lu ok", n);
}

/* "spi=<kHz>": cambia el reloj del SPI (ver sensor_spi_set_khz). */
static void cmd_spi(unsigned long khz)
{
  bool ok;

  (void)pause_sampling();
  close_block();
  ok = sensor_spi_set_khz(khz);
  sensor_connect();                    /* vuelve a medir (con el reloj nuevo si se pudo) */
  if (ok)
  {
    send_text(TRAMA_RESPUESTA, "spi=%lu ok", khz);
  }
  else
  {
    send_text(TRAMA_RESPUESTA, "spi=%lu no_valido (3000, 1500, 750 o 375)", khz);
  }
}

static void process_commands(void)
{
  char cmd[24];
  unsigned long hz;

  if (!usb_link_read_line(cmd, sizeof(cmd)))
  {
    return;
  }

  if (strcmp(cmd, "start") == 0)
  {
    cmd_start();
  }
  else if (strcmp(cmd, "stop") == 0)
  {
    streaming = false;
    send_text(TRAMA_RESPUESTA, "stop ok");
  }
  else if (sscanf(cmd, "rango=%lu", &hz) == 1)
  {
    cmd_rango(hz);
  }
  else if (sscanf(cmd, "fifo=%lu", &hz) == 1)
  {
    cmd_fifo(hz);
  }
  else if (sscanf(cmd, "spi=%lu", &hz) == 1)
  {
    cmd_spi(hz);
  }
  else if (sscanf(cmd, "odr=%lu", &hz) == 1)
  {
    cmd_odr(hz);
  }
  else
  {
    send_text(TRAMA_RESPUESTA, "comando_desconocido");
  }
}

/* ---- API ------------------------------------------------------------------ */

void app_init(void)
{
  /* Contador de ciclos: intervalo entre flancos de INT1, reloj en us y la
     espera de CS en alto de sensor_spi */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  period_min_cyc = UINT32_MAX;
  cyc_per_us = SystemCoreClock / 1000000U;
  clk_cyc_prev = DWT->CYCCNT;

  sensor_spi_init();
  reset_blocks(0);
  sensor_connect();
  t_status = t_retry = t_blink = HAL_GetTick();
}

void app_loop(void)
{
  uint32_t now = HAL_GetTick();

  if (sensor_status == ADXL345_OK)
  {
    if (bus_lost)
    {
      stop_sampling();
      sensor_status = ADXL345_ERR_BUS;   /* se desconecto: vuelve a buscarlo */
      t_retry = now;
    }
    else
    {
      check_stall(now);
    }
  }
  else
  {
    if ((now - t_retry) >= RETRY_PERIOD_MS)
    {
      t_retry = now;
      sensor_connect();
    }
    if ((now - t_blink) >= ERROR_BLINK_MS)
    {
      t_blink = now;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }

  process_commands();
  send_samples();

  /* Hora de nuevo: start pone t_status en el tick actual, que puede ser
     posterior al now del principio de la vuelta (la resta daria la vuelta
     y saldria un estado de mas). */
  now = HAL_GetTick();
  if ((now - t_status) >= STATUS_PERIOD_MS)
  {
    t_status = now;
    (void)clock_us();                  /* el reloj en us no se atrasa aunque no haya flancos */
    update_rates();
    if (sensor_status == ADXL345_OK)
    {
      check_devid();                   /* SPI no avisa si el sensor se desconecta */
    }
    /* Si el USB esta ocupado (la PC no lee) se saltea este estado sin gastar
       un numero de seq: si no, la PC lo veria como una trama perdida. El
       siguiente trae los contadores acumulados. (El estado que sale con
       start si espera: la PC lo necesita.) */
    if (streaming && usb_link_can_write())
    {
      send_status();
    }
    if (sensor_status == ADXL345_OK)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
}
