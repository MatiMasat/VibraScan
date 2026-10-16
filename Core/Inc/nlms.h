/**
  * @file    nlms.h
  * @brief   Filtro adaptativo NLMS (Normalized Least Mean Squares), etapa 7.
  *
  * Solo la matematica: no conoce HAL, USB ni el sensor (se compila tambien en
  * la PC, ver test_fw/filtro_pc.c). Lo usa filtro.c; la referencia en
  * Python es nlms() en pc/vibrascan_filtro.py. Justificacion de la eleccion:
  * docs/filtro_adaptativo.md.
  *
  * Configuracion: realzador de lineas (ALE, Adaptive Line Enhancer). El filtro
  * predice la muestra actual x[n] con las 'taps' muestras anteriores
  * retrasadas 'delta' muestras:
  *
  *   u = [x[n-delta], x[n-delta-1], ..., x[n-delta-taps+1]]
  *   y = w . u                         (prediccion: parte periodica)
  *   e = x[n] - y                      (error: parte de banda ancha)
  *   w = w + mu * e * u / (eps + u.u)  (adaptacion normalizada)
  */
#ifndef NLMS_H
#define NLMS_H

#include <stdint.h>

#define NLMS_MAX_TAPS    64U
#define NLMS_MAX_DELTA   32U
#define NLMS_EPS         1.0e-6f   /* g^2: evita dividir por cero (u.u real ~5e-3) */

typedef struct {
  uint16_t taps;                   /* coeficientes (M) */
  uint16_t delta;                  /* retardo de decorrelacion (muestras) */
  float    mu;                     /* paso de adaptacion normalizado (0 < mu < 2) */
  float    w[NLMS_MAX_TAPS];       /* coeficientes */
  float    hist[NLMS_MAX_TAPS + NLMS_MAX_DELTA];   /* hist[0] = x[n-1], hist[1] = x[n-2], ... */
} nlms_t;

/** Configura el filtro y lo deja en cero. Devuelve 0, o -1 si los
  * parametros estan fuera de rango (taps 1..NLMS_MAX_TAPS, delta
  * 1..NLMS_MAX_DELTA, mu > 0). */
int nlms_iniciar(nlms_t *f, uint16_t taps, uint16_t delta, float mu);

/** Borra coeficientes e historia (misma configuracion). */
void nlms_reiniciar(nlms_t *f);

/** Procesa una muestra x[n]. Devuelve la prediccion y[n] (parte periodica) y
  * deja en *e el error x[n] - y[n] (parte de banda ancha). */
float nlms_procesar(nlms_t *f, float x, float *e);

#endif /* NLMS_H */
