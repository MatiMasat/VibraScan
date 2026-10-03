/**
  * @file    app.c
  * @brief   VibraScan - Etapa 1: lectura del ADXL345 por I2C y envio en texto.
  *
  * Que hace:
  *   - Busca el sensor (0x53 y 0x1D), verifica DEVID y lo configura.
  *   - Consulta por I2C el bit DATA_READY y lee cada muestra una sola vez,
  *     al ritmo que marca el propio sensor (ODR). No hay timer del micro:
  *     el unico reloj de muestreo es el del ADXL345.
  *   - Manda una linea "n,x,y,z" por muestra (cuentas crudas, 3.9 mg/LSB).
  *   - Una vez por segundo manda una linea de estado que empieza con '#'.
  *
  * Contadores de la linea de estado (todos deberian quedarse en 0):
  *   ovr      muestras que el sensor piso antes de que el micro las leyera
  *            (orientativo: la prueba firme es que n crezca a ODR por segundo)
  *   i2c_err  transferencias I2C fallidas
  *   drop     lineas que no se pudieron mandar a la PC
  *
  * LED (PC13): parpadeo lento = midiendo, parpadeo rapido = sensor no responde.
  */
#include "app.h"

#include "main.h"
#include "adxl345.h"
#include "usb_link.h"

/* ---- Configuracion de la etapa ------------------------------------------ */
#define APP_STAGE             1
#define APP_RATE              ADXL345_RATE_100HZ
#define APP_RANGE             ADXL345_RANGE_4G

#define I2C_TIMEOUT_MS        10U
#define STATUS_PERIOD_MS      1000U
#define RETRY_PERIOD_MS       500U
#define ERROR_BLINK_MS        100U
#define MAX_CONSECUTIVE_ERR   10U

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
static i2c_ctx_t        i2c_ctx = { &hi2c1, 0 };
static adxl345_t        sensor;
static adxl345_status_t sensor_status = ADXL345_ERR_BUS;

static uint32_t n_samples;      /* muestras leidas; es el indice que se envia */
static uint32_t n_overrun;
static uint32_t n_i2c_err;
static uint32_t n_drop;
static uint32_t consecutive_err;

static uint32_t t_status;
static uint32_t t_retry;
static uint32_t t_blink;

/* SDO a GND -> 0x53, SDO a VCC -> 0x1D */
static const uint8_t ADDR_CANDIDATES[] = { 0x53, 0x1D };

static void sensor_connect(void)
{
  const adxl345_bus_t bus = { i2c_read, i2c_write, &i2c_ctx };

  for (uint32_t i = 0; i < sizeof(ADDR_CANDIDATES); i++)
  {
    i2c_ctx.addr = (uint16_t)(ADDR_CANDIDATES[i] << 1);
    sensor_status = adxl345_init(&sensor, &bus, APP_RATE, APP_RANGE);
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
  consecutive_err = 0;
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
  if (sensor_status == ADXL345_OK)
  {
    usb_link_printf("# estado=OK etapa=%d addr=0x%02X devid=0x%02X odr=%lu n=%lu ovr=%lu i2c_err=%lu drop=%lu\r\n",
                    APP_STAGE, (unsigned)(i2c_ctx.addr >> 1), sensor.devid,
                    (unsigned long)adxl345_rate_hz(APP_RATE),
                    (unsigned long)n_samples, (unsigned long)n_overrun,
                    (unsigned long)n_i2c_err, (unsigned long)n_drop);
  }
  else
  {
    usb_link_printf("# estado=ERROR etapa=%d causa=%s devid=0x%02X i2c_err=%lu\r\n",
                    APP_STAGE, status_text(sensor_status), sensor.devid,
                    (unsigned long)n_i2c_err);
  }
}

static void bus_error(void)
{
  n_i2c_err++;
  if (++consecutive_err >= MAX_CONSECUTIVE_ERR)
  {
    sensor_status = ADXL345_ERR_BUS;   /* se desconecto: vuelve a buscarlo */
  }
}

static void acquire(void)
{
  uint8_t src = 0;
  adxl345_sample_t s;

  if (adxl345_read_int_source(&sensor, &src) != ADXL345_OK)
  {
    bus_error();
    return;
  }
  if ((src & ADXL345_INT_DATA_READY) == 0U)
  {
    consecutive_err = 0;
    return;                            /* todavia no hay muestra nueva */
  }
  if ((src & ADXL345_INT_OVERRUN) != 0U)
  {
    n_overrun++;                       /* se perdio al menos una muestra */
  }
  if (adxl345_read_xyz(&sensor, &s) != ADXL345_OK)
  {
    bus_error();
    return;
  }
  consecutive_err = 0;

  if (!usb_link_printf("%lu,%d,%d,%d\r\n", (unsigned long)n_samples, s.x, s.y, s.z))
  {
    n_drop++;
  }
  n_samples++;
}

void app_init(void)
{
  sensor_connect();
  t_status = t_retry = t_blink = HAL_GetTick();
}

void app_loop(void)
{
  uint32_t now = HAL_GetTick();

  if (sensor_status == ADXL345_OK)
  {
    acquire();
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
