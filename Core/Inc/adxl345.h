/**
  * @file    adxl345.h
  * @brief   Driver del acelerometro ADXL345, independiente del bus.
  *
  * El driver no conoce HAL ni I2C/SPI: recibe dos funciones (leer y escribir
  * registros) en adxl345_bus_t. Desde la etapa 6 se usa con SPI (ver
  * sensor_spi.c); hasta la etapa 5, con I2C.
  */
#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>
#include <stdbool.h>

/* Registros (datasheet ADXL345, Tabla 19) */
#define ADXL345_REG_DEVID        0x00
#define ADXL345_REG_BW_RATE      0x2C
#define ADXL345_REG_POWER_CTL    0x2D
#define ADXL345_REG_INT_ENABLE   0x2E
#define ADXL345_REG_INT_MAP      0x2F
#define ADXL345_REG_INT_SOURCE   0x30
#define ADXL345_REG_DATA_FORMAT  0x31
#define ADXL345_REG_DATAX0       0x32
#define ADXL345_REG_FIFO_CTL     0x38
#define ADXL345_REG_FIFO_STATUS  0x39

#define ADXL345_DEVID_VALUE      0xE5

/* Bits de INT_SOURCE */
#define ADXL345_INT_DATA_READY   0x80
#define ADXL345_INT_WATERMARK    0x02
#define ADXL345_INT_OVERRUN      0x01

/* FIFO: 32 niveles, mas uno en la salida del filtro (hasta 33 disponibles) */
#define ADXL345_FIFO_SIZE        32U
#define ADXL345_FIFO_MAX_ENTRIES 33U
#define ADXL345_DATA_LEN         6U     /* DATAX0..DATAZ1: una muestra */

/* En FULL_RES la sensibilidad es fija para cualquier rango: 3.9 mg/LSB */
#define ADXL345_UG_PER_LSB       3900

typedef enum {
  ADXL345_OK         =  0,
  ADXL345_ERR_BUS    = -1,   /* el bus no respondio (cableado, direccion, pull-ups) */
  ADXL345_ERR_DEVID  = -2,   /* respondio algo, pero DEVID != 0xE5 */
  ADXL345_ERR_VERIFY = -3    /* un registro no quedo con el valor escrito */
} adxl345_status_t;

/* Codigo del registro BW_RATE. El ancho de banda del sensor es ODR/2. */
typedef enum {
  ADXL345_RATE_100HZ  = 0x0A,
  ADXL345_RATE_200HZ  = 0x0B,
  ADXL345_RATE_400HZ  = 0x0C,
  ADXL345_RATE_800HZ  = 0x0D,
  ADXL345_RATE_1600HZ = 0x0E,
  ADXL345_RATE_3200HZ = 0x0F
} adxl345_rate_t;

typedef enum {
  ADXL345_RANGE_2G  = 0,
  ADXL345_RANGE_4G  = 1,
  ADXL345_RANGE_8G  = 2,
  ADXL345_RANGE_16G = 3
} adxl345_range_t;

/* Devuelven 0 si la transferencia salio bien, distinto de 0 si fallo. */
typedef int (*adxl345_read_fn)(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len);
typedef int (*adxl345_write_fn)(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len);

typedef struct {
  adxl345_read_fn  read;
  adxl345_write_fn write;
  void            *ctx;      /* dato opaco que se le pasa a read/write */
} adxl345_bus_t;

typedef struct {
  adxl345_bus_t bus;
  uint8_t       devid;       /* ultimo DEVID leido (para diagnostico) */
} adxl345_t;

typedef struct {
  int16_t x, y, z;           /* cuentas crudas, 3.9 mg/LSB */
} adxl345_sample_t;

/** Verifica DEVID y deja el sensor midiendo en FULL_RES con el ODR y rango
  * pedidos, FIFO en bypass y sin interrupciones habilitadas. */
adxl345_status_t adxl345_init(adxl345_t *dev, const adxl345_bus_t *bus,
                              adxl345_rate_t rate, adxl345_range_t range);

/** Pone el FIFO en modo stream con la interrupcion watermark en INT1
  * (activo en alto) al juntar 'samples' muestras (1 a 31). Lo hace en standby
  * y pasando por bypass, asi el FIFO arranca vacio; al final vuelve a medir.
  * INT1 queda en alto mientras haya 'samples' muestras o mas sin leer. */
adxl345_status_t adxl345_start_fifo(adxl345_t *dev, uint8_t samples);

/** Lee cuantas muestras hay en el FIFO (0 a ADXL345_FIFO_MAX_ENTRIES). */
adxl345_status_t adxl345_read_fifo_entries(adxl345_t *dev, uint8_t *entries);

/** Lee INT_SOURCE. Los bits DATA_READY y OVERRUN se actualizan siempre,
  * aunque las interrupciones no esten habilitadas. */
adxl345_status_t adxl345_read_int_source(adxl345_t *dev, uint8_t *int_source);

/** Lee los 6 registros de datos en una sola transferencia (asi los 3 ejes
  * son de la misma muestra). Con el FIFO activo, cada lectura saca la muestra
  * mas vieja del FIFO. */
adxl345_status_t adxl345_read_xyz(adxl345_t *dev, adxl345_sample_t *out);

/** Interpreta los ADXL345_DATA_LEN bytes de DATAX0..DATAZ1 (para lecturas
  * hechas por fuera del driver, por ejemplo por DMA). */
void adxl345_decode_xyz(const uint8_t *data, adxl345_sample_t *out);

/** ODR en Hz que corresponde a un codigo de BW_RATE. */
uint32_t adxl345_rate_hz(adxl345_rate_t rate);

#endif /* ADXL345_H */
