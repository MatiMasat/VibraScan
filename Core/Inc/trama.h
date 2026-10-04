/**
  * @file    trama.h
  * @brief   Tramas binarias hacia la PC (etapa 3). No conoce el transporte.
  *
  * Formato (little-endian), igual que pc/vibrascan_proto.py:
  *   sync   2  A5 5A
  *   tipo   1  TRAMA_MUESTRAS, TRAMA_ESTADO o TRAMA_RESPUESTA
  *   largo  2  bytes del contenido
  *   seq    2  contador de tramas
  *   contenido
  *   crc    2  CRC-16/CCITT-FALSE de tipo + largo + seq + contenido
  */
#ifndef TRAMA_H
#define TRAMA_H

#include <stdint.h>

#define TRAMA_MUESTRAS     1U
#define TRAMA_ESTADO       2U   /* contenido: texto "estado=OK clave=valor ..." */
#define TRAMA_RESPUESTA    3U   /* contenido: texto de respuesta a un comando */

/* Bytes que agrega la trama al contenido: sync + tipo + largo + seq + crc */
#define TRAMA_EXTRA        9U

/* Flags del bloque de muestras */
#define TRAMA_FLAG_OVR     0x01U   /* el sensor piso al menos una muestra del bloque */
#define TRAMA_FLAG_HUECO   0x02U   /* antes de este bloque faltaron flancos de INT1 */
#define TRAMA_FLAG_INICIO  0x04U   /* primer bloque desde "start" */

/** CRC-16/CCITT-FALSE (polinomio 0x1021, valor inicial 0xFFFF). */
uint16_t trama_crc16(uint16_t crc, const uint8_t *data, uint32_t len);

/** Arma en out una trama con el contenido dado. out tiene que tener lugar
  * para len + TRAMA_EXTRA bytes. Devuelve los bytes escritos. */
uint32_t trama_armar(uint8_t tipo, uint16_t seq, const void *contenido,
                     uint16_t len, uint8_t *out);

#endif /* TRAMA_H */
