/**
  * @file    adxl345.c
  * @brief   Driver del acelerometro ADXL345, independiente del bus.
  */
#include "adxl345.h"

#define POWER_CTL_STANDBY     0x00
#define POWER_CTL_MEASURE     0x08
#define DATA_FORMAT_FULL_RES  0x08
#define FIFO_CTL_BYPASS       0x00

static adxl345_status_t rd(adxl345_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (dev->bus.read(dev->bus.ctx, reg, buf, len) == 0) ? ADXL345_OK : ADXL345_ERR_BUS;
}

/* Escribe un registro y lo vuelve a leer para confirmar que quedo grabado. */
static adxl345_status_t wr_verify(adxl345_t *dev, uint8_t reg, uint8_t value)
{
  uint8_t readback = 0;

  if (dev->bus.write(dev->bus.ctx, reg, &value, 1) != 0)
  {
    return ADXL345_ERR_BUS;
  }
  if (rd(dev, reg, &readback, 1) != ADXL345_OK)
  {
    return ADXL345_ERR_BUS;
  }
  return (readback == value) ? ADXL345_OK : ADXL345_ERR_VERIFY;
}

adxl345_status_t adxl345_init(adxl345_t *dev, const adxl345_bus_t *bus,
                              adxl345_rate_t rate, adxl345_range_t range)
{
  adxl345_status_t st;

  dev->bus = *bus;
  dev->devid = 0;

  st = rd(dev, ADXL345_REG_DEVID, &dev->devid, 1);
  if (st != ADXL345_OK)
  {
    return st;
  }
  if (dev->devid != ADXL345_DEVID_VALUE)
  {
    return ADXL345_ERR_DEVID;
  }

  /* Se configura en standby y se pasa a medicion al final, como recomienda
     el datasheet. */
  st = wr_verify(dev, ADXL345_REG_POWER_CTL, POWER_CTL_STANDBY);
  if (st != ADXL345_OK) return st;
  st = wr_verify(dev, ADXL345_REG_BW_RATE, (uint8_t)rate);
  if (st != ADXL345_OK) return st;
  st = wr_verify(dev, ADXL345_REG_DATA_FORMAT, (uint8_t)(DATA_FORMAT_FULL_RES | (uint8_t)range));
  if (st != ADXL345_OK) return st;
  st = wr_verify(dev, ADXL345_REG_INT_ENABLE, 0x00);
  if (st != ADXL345_OK) return st;
  st = wr_verify(dev, ADXL345_REG_FIFO_CTL, FIFO_CTL_BYPASS);
  if (st != ADXL345_OK) return st;

  return wr_verify(dev, ADXL345_REG_POWER_CTL, POWER_CTL_MEASURE);
}

adxl345_status_t adxl345_enable_data_ready_int1(adxl345_t *dev)
{
  /* INT_MAP: bit en 0 = esa interrupcion sale por INT1. Se mapea antes de
     habilitar para que nunca aparezca por INT2. */
  adxl345_status_t st = wr_verify(dev, ADXL345_REG_INT_MAP, 0x00);
  if (st != ADXL345_OK) return st;
  return wr_verify(dev, ADXL345_REG_INT_ENABLE, ADXL345_INT_DATA_READY);
}

adxl345_status_t adxl345_read_int_source(adxl345_t *dev, uint8_t *int_source)
{
  return rd(dev, ADXL345_REG_INT_SOURCE, int_source, 1);
}

/* 16 bits en complemento a 2, byte bajo primero */
static void decode_xyz(const uint8_t *d, adxl345_sample_t *out)
{
  out->x = (int16_t)(((uint16_t)d[1] << 8) | d[0]);
  out->y = (int16_t)(((uint16_t)d[3] << 8) | d[2]);
  out->z = (int16_t)(((uint16_t)d[5] << 8) | d[4]);
}

adxl345_status_t adxl345_read_xyz(adxl345_t *dev, adxl345_sample_t *out)
{
  uint8_t d[6];
  adxl345_status_t st = rd(dev, ADXL345_REG_DATAX0, d, sizeof(d));

  if (st == ADXL345_OK)
  {
    decode_xyz(d, out);
  }
  return st;
}

uint8_t adxl345_parse_block(const uint8_t *block, adxl345_sample_t *out)
{
  /* block[0] = INT_SOURCE, block[1] = DATA_FORMAT, block[2..7] = datos */
  decode_xyz(&block[ADXL345_REG_DATAX0 - ADXL345_BLOCK_REG], out);
  return block[0];
}

uint32_t adxl345_rate_hz(adxl345_rate_t rate)
{
  /* 0x0A -> 100 Hz y se duplica con cada codigo */
  return 100UL << ((uint8_t)rate - (uint8_t)ADXL345_RATE_100HZ);
}
