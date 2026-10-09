/**
  * Prueba del driver adxl345.c en la PC, contra un ADXL345 simulado.
  *
  * Compilar y correr (desde la raiz del repo, con el gcc de MinGW):
  *   gcc -std=c99 -Wall -Wextra -I Core/Inc test_fw/test_adxl345.c Core/Src/adxl345.c -o test_adxl345 && ./test_adxl345
  *
  * El sensor simulado guarda los 64 registros, registra cada escritura y
  * responde como el real: lectura de varios bytes seguidos, DEVID = 0xE5,
  * y FIFO_STATUS con el bit FIFO_TRIG en D7.
  */
#include <stdio.h>
#include <string.h>

#include "adxl345.h"

static uint8_t regs[64];
static struct { uint8_t reg, val; } writes[64];
static int n_writes;
static int fails;

static int sim_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
  (void)ctx;
  for (uint16_t i = 0; i < len; i++)
  {
    buf[i] = regs[(reg + i) & 0x3F];
  }
  return 0;
}

static int sim_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
  (void)ctx;
  for (uint16_t i = 0; i < len; i++)
  {
    regs[(reg + i) & 0x3F] = buf[i];
    writes[n_writes].reg = (uint8_t)(reg + i);
    writes[n_writes].val = buf[i];
    n_writes++;
  }
  return 0;
}

#define CHECK(cond, msg) do { if (!(cond)) { printf("FALLA: %s (linea %d)\n", msg, __LINE__); fails++; } \
                              else { printf("ok    %s\n", msg); } } while (0)

/* Indice de la ultima escritura a reg (o -1) */
static int last_write(uint8_t reg)
{
  for (int i = n_writes - 1; i >= 0; i--)
  {
    if (writes[i].reg == reg) return i;
  }
  return -1;
}

int main(void)
{
  adxl345_t dev;
  const adxl345_bus_t bus = { sim_read, sim_write, NULL };
  adxl345_sample_t s;
  uint8_t e;

  memset(regs, 0, sizeof(regs));
  regs[ADXL345_REG_DEVID] = 0xE5;

  CHECK(adxl345_init(&dev, &bus, ADXL345_RATE_3200HZ, ADXL345_RANGE_16G) == ADXL345_OK, "init a 3200 Hz y 16 g");
  CHECK(regs[ADXL345_REG_BW_RATE] == 0x0F, "BW_RATE = 0x0F (3200 Hz)");
  CHECK(regs[ADXL345_REG_DATA_FORMAT] == 0x0B, "DATA_FORMAT = FULL_RES | 16 g, bit SPI en 0 (4 hilos)");
  CHECK(adxl345_rate_hz(ADXL345_RATE_3200HZ) == 3200 && adxl345_rate_hz(ADXL345_RATE_1600HZ) == 1600,
        "rate_hz de 1600 y 3200");

  n_writes = 0;
  regs[ADXL345_REG_FIFO_CTL] = 0x9F;        /* algo viejo que tiene que borrarse */
  CHECK(adxl345_start_fifo(&dev, 16) == ADXL345_OK, "start_fifo(16)");
  CHECK(writes[0].reg == ADXL345_REG_POWER_CTL && writes[0].val == 0x00, "primero standby");
  CHECK(writes[n_writes - 1].reg == ADXL345_REG_POWER_CTL && writes[n_writes - 1].val == 0x08,
        "al final vuelve a medir");
  {
    int i_bypass = -1, i_stream = -1;
    for (int i = 0; i < n_writes; i++)
    {
      if (writes[i].reg == ADXL345_REG_FIFO_CTL && writes[i].val == 0x00 && i_bypass < 0) i_bypass = i;
      if (writes[i].reg == ADXL345_REG_FIFO_CTL && writes[i].val == 0x90) i_stream = i;
    }
    CHECK(i_bypass >= 0 && i_stream > i_bypass, "FIFO: bypass (vacia) y despues stream con 16 (0x90)");
  }
  CHECK(regs[ADXL345_REG_INT_MAP] == 0x00, "INT_MAP = 0: todo por INT1");
  CHECK(regs[ADXL345_REG_INT_ENABLE] == ADXL345_INT_WATERMARK, "solo la interrupcion watermark");
  CHECK(last_write(ADXL345_REG_INT_MAP) < last_write(ADXL345_REG_INT_ENABLE),
        "INT_MAP se escribe antes de habilitar");

  CHECK(adxl345_start_fifo(&dev, 0) != ADXL345_OK, "start_fifo(0) rechazado");
  CHECK(adxl345_start_fifo(&dev, 32) != ADXL345_OK, "start_fifo(32) rechazado");

  regs[ADXL345_REG_FIFO_STATUS] = 0x80 | 21;   /* FIFO_TRIG + 21 muestras */
  CHECK(adxl345_read_fifo_entries(&dev, &e) == ADXL345_OK && e == 21, "FIFO_STATUS: 21 muestras sin el bit FIFO_TRIG");

  {
    const uint8_t d[6] = { 0x03, 0x00, 0xF3, 0xFF, 0x00, 0xF0 };   /* 3, -13, -4096 */
    adxl345_decode_xyz(d, &s);
    CHECK(s.x == 3 && s.y == -13 && s.z == -4096, "decode: complemento a 2, byte bajo primero");
  }

  regs[ADXL345_REG_DEVID] = 0xFF;   /* SPI sin sensor: MISO flotando en alto */
  CHECK(adxl345_init(&dev, &bus, ADXL345_RATE_800HZ, ADXL345_RANGE_4G) == ADXL345_ERR_DEVID,
        "sin sensor (0xFF) da ERR_DEVID");

  printf("\n%s (%d fallas)\n", fails ? "CON FALLAS" : "TODO OK", fails);
  return fails ? 1 : 0;
}
