/**
  * @file    nlms.c
  * @brief   Filtro adaptativo NLMS como realzador de lineas (ALE). Ver nlms.h.
  */
#include "nlms.h"

/* El proyecto compila en Debug (-O0, para depurar con el ST-Link); este
   archivo corre por cada muestra y se optimiza aparte: con -O0 el filtro
   usaba ~18 % del CPU a 3200 Hz (medido en la etapa 7). Optimizar solo
   aca no cambia la temporizacion de la lectura del FIFO (etapa 6). */
#pragma GCC optimize ("O2")

#include <string.h>

int nlms_iniciar(nlms_t *f, uint16_t taps, uint16_t delta, float mu)
{
  if ((taps == 0U) || (taps > NLMS_MAX_TAPS) || (delta == 0U) || (delta > NLMS_MAX_DELTA) || !(mu > 0.0f))
  {
    return -1;
  }
  f->taps = taps;
  f->delta = delta;
  f->mu = mu;
  nlms_reiniciar(f);
  return 0;
}

void nlms_reiniciar(nlms_t *f)
{
  memset(f->w, 0, sizeof(f->w));
  memset(f->hist, 0, sizeof(f->hist));
}

float nlms_procesar(nlms_t *f, float x, float *e)
{
  /* Entrada del filtro: u[k] = x[n - delta - k] = hist[delta - 1 + k] */
  const float *u = &f->hist[f->delta - 1U];
  float y = 0.0f;
  float energia = 0.0f;
  float err, g;
  uint32_t k;

  for (k = 0; k < f->taps; k++)
  {
    y += f->w[k] * u[k];
    energia += u[k] * u[k];      /* se recalcula entera: sin deriva por redondeo */
  }
  err = x - y;

  g = f->mu * err / (NLMS_EPS + energia);
  for (k = 0; k < f->taps; k++)
  {
    f->w[k] += g * u[k];
  }

  /* La historia avanza: x[n] pasa a ser x[(n+1) - 1] */
  memmove(&f->hist[1], &f->hist[0], (size_t)(f->taps + f->delta - 1U) * sizeof(float));
  f->hist[0] = x;

  *e = err;
  return y;
}
