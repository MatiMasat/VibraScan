/**
  * @file    sensor_spi.h
  * @brief   Bus SPI del ADXL345 (etapa 6). Es lo unico que conoce SPI1, sus
  *          pines, el DMA y el pin CS.
  *
  * Conexiones (SPI de 4 hilos, CPOL = 1, CPHA = 1, 3 MHz):
  *   PA4 -> CS     PA5 -> SCL (SCLK)     PA7 -> SDA (SDI)     PA6 <- SDO
  *
  * Entre dos transacciones CS queda en alto al menos SENSOR_SPI_GAP_US: la
  * hoja de datos pide >= 5 us entre lecturas del FIFO por encima de 1,6 MHz.
  */
#ifndef SENSOR_SPI_H
#define SENSOR_SPI_H

#include <stdint.h>
#include <stdbool.h>

#define SENSOR_SPI_GAP_US     6U    /* CS en alto entre transacciones (>= 5 us) */
#define SENSOR_SPI_MAX_READ   8U    /* bytes por lectura asincronica */

/** Configura pines, SPI1 y DMA. Llamar una vez, antes de usar el bus. */
void sensor_spi_init(void);

/** Lectura y escritura bloqueantes (para configurar el sensor). Devuelven 0
  * si la transferencia salio bien. Firma de adxl345_read_fn/adxl345_write_fn. */
int sensor_spi_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len);
int sensor_spi_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len);

/** Al terminar una lectura asincronica (en contexto de interrupcion): data
  * apunta a los len bytes leidos, o es NULL si fallo. */
typedef void (*sensor_spi_done_fn)(const uint8_t *data, uint16_t len);

/** Lectura por DMA de len bytes desde reg (len <= SENSOR_SPI_MAX_READ). No
  * bloquea, salvo la espera de SENSOR_SPI_GAP_US desde la transaccion
  * anterior. Devuelve false si no se pudo lanzar (no se llama a done). */
bool sensor_spi_read_async(uint8_t reg, uint16_t len, sensor_spi_done_fn done);

/** Cancela una lectura asincronica en curso y deja CS en alto. */
void sensor_spi_abort(void);

/** Cambia el reloj del SPI: 3000, 1500, 750 o 375 kHz (96 MHz / 32 a 256).
  * Solo sin transferencias en curso. Devuelve false si el valor no es valido. */
bool sensor_spi_set_khz(uint32_t khz);

/** Reloj actual del SPI en kHz. */
uint32_t sensor_spi_khz(void);

#endif /* SENSOR_SPI_H */
