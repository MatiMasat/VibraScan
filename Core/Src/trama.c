/**
  * @file    trama.c
  * @brief   Tramas binarias hacia la PC (etapa 3). Ver trama.h.
  */
#include "trama.h"

#include <string.h>

uint16_t trama_crc16(uint16_t crc, const uint8_t *data, uint32_t len)
{
  /* Bit a bit, sin tabla: unos 70 us por trama de 213 bytes a 96 MHz,
     suficiente para 25 tramas por segundo. */
  for (uint32_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (uint32_t b = 0; b < 8U; b++)
    {
      crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

uint32_t trama_armar(uint8_t tipo, uint16_t seq, const void *contenido,
                     uint16_t len, uint8_t *out)
{
  uint16_t crc;

  out[0] = 0xA5U;
  out[1] = 0x5AU;
  out[2] = tipo;
  out[3] = (uint8_t)(len & 0xFFU);
  out[4] = (uint8_t)(len >> 8);
  out[5] = (uint8_t)(seq & 0xFFU);
  out[6] = (uint8_t)(seq >> 8);
  memcpy(&out[7], contenido, len);

  crc = trama_crc16(0xFFFFU, &out[2], 5U + (uint32_t)len);
  out[7U + len] = (uint8_t)(crc & 0xFFU);
  out[8U + len] = (uint8_t)(crc >> 8);
  return TRAMA_EXTRA + (uint32_t)len;
}
