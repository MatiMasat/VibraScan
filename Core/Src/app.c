/**
  * @file    app.c
  * @brief   VibraScan - Etapa 2: muestreo por interrupcion DATA_READY a 800 Hz.
  *
  * Que hace:
  *   - Busca el sensor (0x53 y 0x1D), verifica DEVID y lo configura con
  *     DATA_READY por INT1 -> PB0.
  *   - Cada flanco de subida en PB0 (EXTI) lanza una lectura I2C por
  *     interrupcion (no bloquea): 8 registros desde INT_SOURCE, que traen la
  *     muestra y el bit OVERRUN. Al terminar, la muestra va a un buffer
  *     circular con su indice n.
  *   - app_loop() solo saca muestras del buffer y las manda a la PC como
  *     lineas "n,x,y,z" (cuentas crudas, 3.9 mg/LSB), juntando varias lineas
  *     por envio USB. Si el USB se demora, el muestreo no se entera: las
  *     muestras esperan en el buffer.
  *   - Una vez por segundo manda una linea de estado que empieza con '#'.
  *
  * El unico reloj de muestreo sigue siendo el del ADXL345.
  *
  * Contadores de la linea de estado (todos deberian quedarse en 0):
  *   ovr      muestras que el sensor piso antes de que el micro las leyera
  *   i2c_err  transferencias I2C fallidas
  *   drop     lineas que no se pudieron mandar a la PC
  *   stall    veces que INT1 quedo en alto sin lectura en curso y hubo que
  *            relanzarla desde app_loop (se perdio un flanco)
  *   bfull    muestras descartadas porque el buffer estaba lleno (la PC o el
  *            USB no llegan a llevarse los datos)
  *
  * Campos informativos (del ultimo segundo):
  *   int1     flancos de subida en PB0 (= odr)
  *   sps      muestras leidas (= odr)
  *   tmin     periodo minimo entre flancos de INT1, en us (reloj del micro)
  *   tmax     periodo maximo; si tmax ~ 2 x tmin se perdio un flanco
  *
  * Comandos de la PC (una linea de texto, respuesta en una linea con '>'):
  *   odr=<Hz>  cambia el ODR: 100, 200, 400 u 800 (por defecto 800)
  *
  * LED (PC13): parpadeo lento = midiendo, parpadeo rapido = sensor no responde.
  */
#include "app.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "adxl345.h"
#include "usb_link.h"

/* ---- Configuracion de la etapa ------------------------------------------ */
#define APP_STAGE             2
#define APP_RATE_DEFAULT      ADXL345_RATE_800HZ   /* se cambia con el comando odr= */
#define APP_RATE_MAX          ADXL345_RATE_800HZ   /* 1600 y 3200 Hz piden SPI (etapa 6) */
#define APP_RANGE             ADXL345_RANGE_4G

#define I2C_TIMEOUT_MS        10U
#define STATUS_PERIOD_MS      1000U
#define RETRY_PERIOD_MS       500U
#define ERROR_BLINK_MS        100U
#define MAX_CONSECUTIVE_ERR   10U

#define RING_SIZE             1024U   /* muestras; potencia de 2 (1,3 s a 800 Hz) */
#define TX_BATCH_SIZE         512U    /* bytes por envio USB (<= USB_LINK_MAX_WRITE) */
#define LINE_MAX              40U     /* "4294967295,-32768,-32768,-32768\r\n" */

_Static_assert(TX_BATCH_SIZE <= USB_LINK_MAX_WRITE, "TX_BATCH_SIZE mayor que lo que acepta usb_link");
_Static_assert((RING_SIZE & (RING_SIZE - 1U)) == 0U, "RING_SIZE debe ser potencia de 2");


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

/* ---- Estado --------------------------------------------------------------- */
typedef struct {
  uint32_t         n;
  adxl345_sample_t s;
} ring_item_t;

static i2c_ctx_t        i2c_ctx = { &hi2c1, 0 };
static adxl345_t        sensor;
static adxl345_status_t sensor_status = ADXL345_ERR_BUS;
static adxl345_rate_t   app_rate = APP_RATE_DEFAULT;

/* Compartido con las interrupciones (EXTI e I2C) */
static volatile bool     sampling;        /* true = las interrupciones pueden leer */
static volatile bool     rx_busy;         /* hay una lectura I2C en curso */
static volatile bool     bus_lost;        /* demasiados errores seguidos */
static volatile uint32_t t_last_read;     /* HAL_GetTick() de la ultima lectura completa */
static uint8_t           rx_block[ADXL345_BLOCK_LEN];

static ring_item_t       ring[RING_SIZE];
static volatile uint32_t ring_head;       /* lo avanza la interrupcion I2C */
static volatile uint32_t ring_tail;       /* lo avanza app_loop */

static volatile uint32_t n_samples;       /* muestras leidas; es el indice que se envia */
static volatile uint32_t n_overrun;
static volatile uint32_t n_i2c_err;
static volatile uint32_t n_buf_full;
static volatile uint32_t n_int1_edges;
static volatile uint32_t consecutive_err;
static uint32_t n_drop;
static uint32_t n_stall;
static uint32_t int1_edges_prev;
static uint32_t n_samples_prev;

/* Periodo entre flancos de INT1 medido con el contador de ciclos (DWT), que
   corre con el reloj del micro (cristal de 25 MHz). Min y max por segundo. */
static volatile uint32_t edge_cyc_prev;
static volatile bool     edge_have_prev;
static volatile uint32_t period_min_cyc;
static volatile uint32_t period_max_cyc;

static uint32_t t_status;
static uint32_t t_retry;
static uint32_t t_blink;

static char tx_batch[TX_BATCH_SIZE];

/* SDO a GND -> 0x53, SDO a VCC -> 0x1D */
static const uint8_t ADDR_CANDIDATES[] = { 0x53, 0x1D };

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

static void start_read(void)
{
  if (!claim_bus())
  {
    return;
  }
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
    start_read();
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  adxl345_sample_t s;
  uint8_t src;
  uint32_t head;

  if (hi2c != &hi2c1)
  {
    return;
  }

  src = adxl345_parse_block(rx_block, &s);
  if ((src & ADXL345_INT_OVERRUN) != 0U)
  {
    n_overrun++;                       /* se perdio al menos una muestra */
  }

  head = ring_head;
  if ((head - ring_tail) < RING_SIZE)
  {
    ring[head & (RING_SIZE - 1U)].n = n_samples;
    ring[head & (RING_SIZE - 1U)].s = s;
    ring_head = head + 1U;
  }
  else
  {
    n_buf_full++;
  }
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
    sensor_status = adxl345_init(&sensor, &bus, app_rate, APP_RANGE);
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

/* Junta en tx_batch todas las lineas que entren y las manda en un solo envio.
   Si el USB todavia esta ocupado con el envio anterior (la PC tarda en leer)
   no se espera ni se descarta nada: las muestras quedan en el buffer circular
   y salen en una vuelta siguiente, en un envio mas lleno. Solo se pierden si
   el buffer se llena (bfull) o si el USB no esta conectado (drop). */
static void send_samples(void)
{
  char line[LINE_MAX];
  uint32_t len = 0;
  uint32_t lines = 0;

  if (!usb_link_ready())
  {
    n_drop += ring_head - ring_tail;   /* sin PC: se descarta lo acumulado */
    ring_tail = ring_head;
    return;
  }
  if (!usb_link_can_write())
  {
    return;                            /* se reintenta en la proxima vuelta */
  }

  while (ring_tail != ring_head)
  {
    const ring_item_t *it = &ring[ring_tail & (RING_SIZE - 1U)];
    int n = snprintf(line, sizeof(line), "%lu,%d,%d,%d\r\n",
                     (unsigned long)it->n, it->s.x, it->s.y, it->s.z);

    if ((n <= 0) || ((len + (uint32_t)n) > sizeof(tx_batch)))
    {
      break;                           /* el resto va en el proximo envio */
    }
    memcpy(&tx_batch[len], line, (size_t)n);
    len += (uint32_t)n;
    lines++;
    ring_tail = ring_tail + 1U;
  }

  if ((len > 0U) && !usb_link_write((const uint8_t *)tx_batch, (uint16_t)len))
  {
    n_drop += lines;
  }
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

static void send_status(void)
{
  uint32_t edges = n_int1_edges;
  uint32_t samples = n_samples;
  uint32_t int1 = edges - int1_edges_prev;
  uint32_t sps = samples - n_samples_prev;
  uint32_t cyc_per_us = SystemCoreClock / 1000000U;
  uint32_t pmin, pmax;
  uint32_t primask = __get_PRIMASK();

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

  if (sensor_status == ADXL345_OK)
  {
    usb_link_printf("# estado=OK etapa=%d addr=0x%02X devid=0x%02X odr=%lu n=%lu ovr=%lu i2c_err=%lu drop=%lu stall=%lu bfull=%lu int1=%lu sps=%lu tmin=%lu tmax=%lu\r\n",
                    APP_STAGE, (unsigned)(i2c_ctx.addr >> 1), sensor.devid,
                    (unsigned long)adxl345_rate_hz(app_rate),
                    (unsigned long)samples, (unsigned long)n_overrun,
                    (unsigned long)n_i2c_err, (unsigned long)n_drop,
                    (unsigned long)n_stall, (unsigned long)n_buf_full,
                    (unsigned long)int1, (unsigned long)sps,
                    (unsigned long)(pmin / cyc_per_us), (unsigned long)(pmax / cyc_per_us));
  }
  else
  {
    usb_link_printf("# estado=ERROR etapa=%d causa=%s devid=0x%02X i2c_err=%lu\r\n",
                    APP_STAGE, status_text(sensor_status), sensor.devid,
                    (unsigned long)n_i2c_err);
  }
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
      ((now - t_last_read) > stall_ms))
  {
    n_stall++;
    t_last_read = now;                 /* un relanzamiento por periodo de espera */
    start_read();
  }
}

/* ---- Comandos de la PC ---------------------------------------------------- */

/* "odr=<Hz>": reconfigura el sensor con otro ODR (100 a 800 Hz). Responde con
   una linea que empieza con '>' para no confundirse con muestras ni estado. */
static void process_commands(void)
{
  char cmd[24];
  unsigned long hz;

  if (!usb_link_read_line(cmd, sizeof(cmd)))
  {
    return;
  }

  if (sscanf(cmd, "odr=%lu", &hz) == 1)
  {
    for (adxl345_rate_t r = ADXL345_RATE_100HZ; r <= APP_RATE_MAX; r++)
    {
      if (adxl345_rate_hz(r) == hz)
      {
        if (sensor_status == ADXL345_OK)
        {
          stop_sampling();
        }
        app_rate = r;
        sensor_connect();
        usb_link_printf("> odr=%lu ok\r\n", hz);
        return;
      }
    }
    usb_link_printf("> odr=%lu no_valido (100, 200, 400 u 800)\r\n", hz);
    return;
  }
  usb_link_printf("> comando_desconocido\r\n");
}

/* ---- API ------------------------------------------------------------------ */

void app_init(void)
{
  /* Contador de ciclos para medir el periodo entre flancos de INT1 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  period_min_cyc = UINT32_MAX;

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

  if ((now - t_status) >= STATUS_PERIOD_MS)
  {
    t_status = now;
    send_status();
    if (sensor_status == ADXL345_OK)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
}
