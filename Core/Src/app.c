/**
  * @file    app.c
  * @brief   VibraScan - Etapa 4: etapa 3 (bloques y trama binaria con CRC) + comando rango=.
  *
  * Que hace:
  *   - Busca el sensor (0x53 y 0x1D), verifica DEVID y lo configura con
  *     DATA_READY por INT1 -> PB0.
  *   - Cada flanco de subida en PB0 (EXTI) toma la hora (us del micro) y
  *     lanza una lectura I2C por interrupcion (no bloquea): 8 registros desde
  *     INT_SOURCE, que traen la muestra y el bit OVERRUN.
  *   - Al terminar la lectura, la muestra se agrega al bloque en curso. Un
  *     bloque tiene hasta BLOCK_SAMPLES muestras contiguas; se cierra al
  *     llenarse, al detectar un hueco (mas de 1,5 periodos desde la muestra
  *     anterior) o al cambiar el ODR.
  *   - app_loop() manda cada bloque cerrado como una trama binaria con CRC
  *     (ver trama.h y pc/vibrascan_proto.py), varias tramas por envio USB.
  *     Si el USB se demora, el muestreo no se entera: los bloques esperan.
  *   - No se manda nada hasta que la PC envia "start": asi no quedan datos
  *     viejos en camino al abrir el puerto. Con "stop", o si el USB se
  *     desconecta, deja de mandar.
  *   - Mientras manda, una trama de estado por segundo (texto).
  *
  * El unico reloj de muestreo sigue siendo el del ADXL345.
  *
  * Contadores de la trama de estado (todos deberian quedarse en 0):
  *   ovr      lecturas en las que el sensor ya habia pisado una muestra
  *   i2c_err  transferencias I2C fallidas
  *   drop     muestras que no se pudieron mandar a la PC
  *   stall    veces que INT1 quedo en alto sin lectura en curso y hubo que
  *            relanzarla desde app_loop (se perdio un flanco)
  *   bfull    muestras descartadas porque el buffer estaba lleno (la PC o el
  *            USB no llegan a llevarse los datos)
  *   hueco    veces que pasaron mas de 1,5 periodos entre dos muestras
  *
  * Campos informativos (del ultimo segundo):
  *   int1     flancos de subida en PB0 (= odr)
  *   sps      muestras leidas (= odr)
  *   tmin     periodo minimo entre flancos de INT1, en us (reloj del micro)
  *   tmax     periodo maximo; si tmax ~ 2 x tmin se perdio un flanco
  *
  * Comandos de la PC (una linea de texto; la respuesta va en una trama):
  *   start     vacia el buffer, reinicia seq y empieza a mandar
  *   stop      deja de mandar (el sensor sigue midiendo)
  *   odr=<Hz>  cambia el ODR: 100, 200, 400 u 800 (por defecto 800)
  *   rango=<g> cambia el rango: 2, 4, 8 o 16 g (por defecto 4). En FULL_RES la
  *             escala es 3,9 mg/cuenta en cualquier rango; cambia el tope.
  *             Si se manda mientras se transmite, la reconfiguracion deja un
  *             hueco real de unos ms y se informa como tal.
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
#include "trama.h"
#include "usb_link.h"

/* ---- Configuracion de la etapa ------------------------------------------ */
#define APP_STAGE             4
#define APP_RATE_DEFAULT      ADXL345_RATE_800HZ   /* se cambia con el comando odr= */
#define APP_RATE_MAX          ADXL345_RATE_800HZ   /* 1600 y 3200 Hz piden SPI (etapa 6) */
#define APP_RANGE_DEFAULT     ADXL345_RANGE_4G     /* se cambia con el comando rango= */

#define I2C_TIMEOUT_MS        10U
#define STATUS_PERIOD_MS      1000U
#define RETRY_PERIOD_MS       500U
#define ERROR_BLINK_MS        100U
#define MAX_CONSECUTIVE_ERR   10U

#define BLOCK_SAMPLES         32U     /* muestras por bloque = por trama (40 ms a 800 Hz) */
#define BLOCK_COUNT           32U     /* bloques; potencia de 2 (1024 muestras, 1,3 s a 800 Hz) */
#define TX_BATCH_SIZE         512U    /* bytes por envio USB (<= USB_LINK_MAX_WRITE): 2 tramas */
#define TEXT_MAX              200U    /* texto de una trama de estado o respuesta */

_Static_assert(TX_BATCH_SIZE <= USB_LINK_MAX_WRITE, "TX_BATCH_SIZE mayor que lo que acepta usb_link");
_Static_assert((BLOCK_COUNT & (BLOCK_COUNT - 1U)) == 0U, "BLOCK_COUNT debe ser potencia de 2");
_Static_assert(BLOCK_SAMPLES <= 255U, "cant es de 8 bits");


extern I2C_HandleTypeDef hi2c1;

/* ---- Acceso al bus: lo unico que cambia al pasar a SPI ------------------- */
typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint16_t           addr;    /* direccion de 7 bits ya desplazada (<< 1) */
} i2c_ctx_t;

static int i2c_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c_ctx_t *c = (i2c_ctx_t *)ctx;
  return (HAL_I2C_Mem_Read(c->hi2c, c->addr, reg, I2C_MEMADD_SIZE_8BIT,
                           buf, len, I2C_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

static int i2c_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
  i2c_ctx_t *c = (i2c_ctx_t *)ctx;
  return (HAL_I2C_Mem_Write(c->hi2c, c->addr, reg, I2C_MEMADD_SIZE_8BIT,
                            (uint8_t *)buf, len, I2C_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

/* ---- Bloque de muestras -------------------------------------------------- */

/* Tiene la misma forma que el contenido de la trama de muestras (el micro y
   la PC son little-endian): la trama se arma con un memcpy de los primeros
   BLOCK_HEADER_LEN + 6 x cant bytes. */
typedef struct {
  uint32_t         n0;      /* indice de la primera muestra */
  uint32_t         t0;      /* us del micro en el flanco de la primera muestra */
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

/* ---- Estado --------------------------------------------------------------- */
static i2c_ctx_t        i2c_ctx = { &hi2c1, 0 };
static adxl345_t        sensor;
static adxl345_status_t sensor_status = ADXL345_ERR_BUS;
static adxl345_rate_t   app_rate = APP_RATE_DEFAULT;
static adxl345_range_t  app_range = APP_RANGE_DEFAULT;

/* Compartido con las interrupciones (EXTI e I2C) */
static volatile bool     sampling;        /* true = las interrupciones pueden leer */
static volatile bool     rx_busy;         /* hay una lectura I2C en curso */
static volatile bool     bus_lost;        /* demasiados errores seguidos */
static volatile uint32_t t_last_read;     /* HAL_GetTick() de la ultima lectura completa */
static volatile uint32_t read_t_us;       /* hora del flanco de la lectura en curso */
static uint8_t           rx_block[ADXL345_BLOCK_LEN];

/* Bloques: la interrupcion I2C carga el bloque blocks[head] y al cerrarlo
   avanza head; app_loop manda desde tail. Un bloque recien es visible para
   app_loop cuando esta cerrado. */
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
static volatile uint32_t n_i2c_err;
static volatile uint32_t n_buf_full;
static volatile uint32_t n_gap;
static volatile uint32_t n_int1_edges;
static volatile uint32_t consecutive_err;
static uint32_t n_drop;
static uint32_t n_stall;
static uint32_t int1_edges_prev;
static uint32_t n_samples_prev;
static uint32_t st_int1, st_sps, st_tmin_us, st_tmax_us;   /* ultimo segundo completo */

/* Periodo entre flancos de INT1 medido con el contador de ciclos (DWT), que
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

/* SDO a GND -> 0x53, SDO a VCC -> 0x1D */
static const uint8_t ADDR_CANDIDATES[] = { 0x53, 0x1D };

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

/* ---- Bloques (los llama la interrupcion I2C, o app_loop con el muestreo
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
   Solo con el muestreo detenido. */
static void reset_blocks(uint8_t next_flags)
{
  blk_open = false;
  blk_head = 0;
  blk_tail = 0;
  blk_next_flags = next_flags;
  have_last = false;
}

/* ---- Lectura por interrupcion -------------------------------------------- */

/* Reserva el bus para una lectura. Puede llamarse desde la EXTI o desde
   app_loop, por eso la consulta y la marca se hacen sin interrupciones. */
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
  n_i2c_err++;
  if (++consecutive_err >= MAX_CONSECUTIVE_ERR)
  {
    bus_lost = true;                   /* app_loop vuelve a buscar el sensor */
  }
}

static void start_read(uint32_t t_us)
{
  if (!claim_bus())
  {
    return;
  }
  read_t_us = t_us;
  if (HAL_I2C_Mem_Read_IT(&hi2c1, i2c_ctx.addr, ADXL345_BLOCK_REG, I2C_MEMADD_SIZE_8BIT,
                          rx_block, ADXL345_BLOCK_LEN) != HAL_OK)
  {
    rx_busy = false;
    read_error();
  }
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
    start_read(clock_us());
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  adxl345_sample_t s;
  uint8_t src;
  uint32_t t;

  if (hi2c != &hi2c1)
  {
    return;
  }

  src = adxl345_parse_block(rx_block, &s);
  if ((src & ADXL345_INT_OVERRUN) != 0U)
  {
    n_overrun++;                       /* se perdio al menos una muestra */
  }

  /* Hueco: el bloque en curso se cierra para que cada bloque tenga solo
     muestras contiguas, y el siguiente lleva la marca. */
  t = read_t_us;
  if (have_last && ((t - last_t_us) > gap_limit_us))
  {
    n_gap++;
    close_block();
    blk_next_flags |= TRAMA_FLAG_HUECO;
  }
  have_last = true;
  last_t_us = t;

  store_sample(&s, t, (src & ADXL345_INT_OVERRUN) != 0U);
  n_samples++;

  consecutive_err = 0;
  t_last_read = HAL_GetTick();
  rx_busy = false;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != &hi2c1)
  {
    return;
  }
  rx_busy = false;
  read_error();
}

/* Red de seguridad: DATA_READY es por nivel. Si una lectura falla, INT1 queda
   en alto, no hay mas flancos y el muestreo se detiene. Aca se detecta y se
   relanza la lectura. */
static void check_stall(uint32_t now)
{
  /* Se perdio un flanco si INT1 esta en alto sin lectura en curso y no se
     completo ninguna lectura en dos periodos (mas un margen). */
  uint32_t stall_ms = 2U * 1000U / adxl345_rate_hz(app_rate) + 2U;

  if (!rx_busy &&
      (HAL_GPIO_ReadPin(ADXL_INT1_GPIO_Port, ADXL_INT1_Pin) == GPIO_PIN_SET) &&
      /* con signo: la interrupcion puede haber escrito t_last_read despues
         de que app_loop leyo now, y la resta sin signo daria la vuelta */
      ((int32_t)(now - t_last_read) > (int32_t)stall_ms))
  {
    n_stall++;
    t_last_read = now;                 /* un relanzamiento por periodo de espera */
    start_read(clock_us());
  }
}

/* ---- Conexion con el sensor ---------------------------------------------- */

static void stop_sampling(void)
{
  uint32_t t0 = HAL_GetTick();

  HAL_NVIC_DisableIRQ(ADXL_INT1_EXTI_IRQn);
  sampling = false;
  while (rx_busy && ((HAL_GetTick() - t0) < I2C_TIMEOUT_MS))
  {
    /* espera a que termine la lectura en curso */
  }
  rx_busy = false;   /* si no termino, sensor_connect reinicia el I2C */
}

static void start_sampling(void)
{
  adxl345_sample_t dummy;

  /* Se borra un flanco viejo y se lee la muestra pendiente para que INT1
     baje: asi el proximo flanco corresponde a una muestra nueva. Si llega
     una muestra antes de habilitar la EXTI, su flanco queda pendiente y se
     atiende al habilitarla. */
  __HAL_GPIO_EXTI_CLEAR_IT(ADXL_INT1_Pin);
  (void)adxl345_read_xyz(&sensor, &dummy);

  blk_odr = (uint16_t)adxl345_rate_hz(app_rate);
  gap_limit_us = 1500000U / blk_odr;

  consecutive_err = 0;
  bus_lost = false;
  edge_have_prev = false;
  t_last_read = HAL_GetTick();
  sampling = true;
  HAL_NVIC_EnableIRQ(ADXL_INT1_EXTI_IRQn);
}

static void sensor_connect(void)
{
  const adxl345_bus_t bus = { i2c_read, i2c_write, &i2c_ctx };

  for (uint32_t i = 0; i < sizeof(ADDR_CANDIDATES); i++)
  {
    i2c_ctx.addr = (uint16_t)(ADDR_CANDIDATES[i] << 1);
    sensor_status = adxl345_init(&sensor, &bus, app_rate, app_range);
    if (sensor_status == ADXL345_OK)
    {
      sensor_status = adxl345_enable_data_ready_int1(&sensor);
    }
    if (sensor_status != ADXL345_ERR_BUS)
    {
      break;    /* respondio algo en esta direccion: no se sigue probando */
    }
  }

  if (sensor_status == ADXL345_ERR_BUS)
  {
    /* Un corte a mitad de una transferencia puede dejar el I2C trabado en
       BUSY; reiniciar el periferico lo destraba para el proximo intento. */
    n_i2c_err++;
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
  }
  else if (sensor_status == ADXL345_OK)
  {
    start_sampling();
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

/* Manda una trama de texto (estado o respuesta). */
static void send_text(uint8_t tipo, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void send_text(uint8_t tipo, const char *fmt, ...)
{
  char text[TEXT_MAX];
  uint8_t frame[TEXT_MAX + TRAMA_EXTRA];
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
    case ADXL345_ERR_BUS:    return "sin_respuesta_i2c";
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
              "estado=OK etapa=%d addr=0x%02X devid=0x%02X odr=%lu rango=%u n=%lu ovr=%lu i2c_err=%lu drop=%lu stall=%lu bfull=%lu hueco=%lu int1=%lu sps=%lu tmin=%lu tmax=%lu",
              APP_STAGE, (unsigned)(i2c_ctx.addr >> 1), sensor.devid,
              (unsigned long)adxl345_rate_hz(app_rate), 2U << (unsigned)app_range,
              (unsigned long)n_samples, (unsigned long)n_overrun,
              (unsigned long)n_i2c_err, (unsigned long)n_drop,
              (unsigned long)n_stall, (unsigned long)n_buf_full, (unsigned long)n_gap,
              (unsigned long)st_int1, (unsigned long)st_sps,
              (unsigned long)st_tmin_us, (unsigned long)st_tmax_us);
  }
  else
  {
    send_text(TRAMA_ESTADO, "estado=ERROR etapa=%d causa=%s devid=0x%02X i2c_err=%lu",
              APP_STAGE, status_text(sensor_status), sensor.devid,
              (unsigned long)n_i2c_err);
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

/* "odr=<Hz>": reconfigura el sensor con otro ODR (100 a 800 Hz). */
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
  send_text(TRAMA_RESPUESTA, "odr=%lu no_valido (100, 200, 400 u 800)", hz);
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
  /* Contador de ciclos: periodo entre flancos de INT1 y reloj en us */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  period_min_cyc = UINT32_MAX;
  cyc_per_us = SystemCoreClock / 1000000U;
  clk_cyc_prev = DWT->CYCCNT;

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
    if (streaming)
    {
      send_status();
    }
    if (sensor_status == ADXL345_OK)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
}
