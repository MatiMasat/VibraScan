/**
  * @file    filtro.h
  * @brief   Cadena de filtrado de un eje en el micro, etapa 7.
  *
  * Es el unico lugar que elige el algoritmo: para agregar otro filtro (RLS,
  * notch, ...) se agrega su archivo (como nlms.c), un valor en filtro_tipo_t
  * y un case en filtro_procesar(). No conoce HAL ni USB (se compila tambien en
  * la PC). Lo llama app.c con cada muestra del eje elegido; la referencia en
  * Python es Filtro en pc/vibrascan_filtro.py.
  *
  * Por muestra (filtro_procesar):
  *   1. Preprocesamiento: pasaaltos de 1er orden (FILTRO_FC_HZ) que saca la
  *      gravedad y la deriva lenta.
  *   2. Algoritmo adaptativo (hoy NLMS, nlms_procesar): devuelve la parte
  *      realzada y y el residuo e.
  */
#ifndef FILTRO_H
#define FILTRO_H

#include <stdbool.h>
#include <stdint.h>

#include "nlms.h"

#define FILTRO_FC_HZ     1.0f      /* corte del pasaaltos */

typedef enum {
  FILTRO_OFF  = 0,
  FILTRO_NLMS = 1,
  /* FILTRO_RLS = 2, FILTRO_NOTCH = 3, ...: futuros */
} filtro_tipo_t;

typedef struct {
  filtro_tipo_t tipo;
  float         fs;                /* Hz (nominal; el corte del pasaaltos no es critico) */
  float         hp_a;              /* coeficiente del pasaaltos */
  float         hp_x1, hp_y1;      /* estado del pasaaltos */
  bool          hp_listo;          /* false: la proxima muestra inicializa el estado */
  nlms_t        nlms;
} filtro_t;

/** Configura la cadena. Para FILTRO_NLMS usa taps, delta y mu. Devuelve 0, o
  * -1 si algun parametro no es valido (el filtro queda en FILTRO_OFF). */
int filtro_configurar(filtro_t *f, filtro_tipo_t tipo, float fs,
                      uint16_t taps, uint16_t delta, float mu);

/** Vuelve a empezar (estado del pasaaltos y del algoritmo en cero). Se llama
  * ante una discontinuidad: inicio, hueco o cambio de ODR. */
void filtro_reiniciar(filtro_t *f);

/** Procesa una muestra x (en g). Devuelve la salida realzada y deja el residuo
  * en *e. Con FILTRO_OFF devuelve la entrada sin la componente continua y e = 0. */
float filtro_procesar(filtro_t *f, float x, float *e);

#endif /* FILTRO_H */
