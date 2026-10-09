/**
  * @file    filtro.c
  * @brief   Cadena de filtrado de un eje en el micro. Ver filtro.h.
  */
#include "filtro.h"

#define PI_F  3.14159265358979f

int filtro_configurar(filtro_t *f, filtro_tipo_t tipo, float fs,
                      uint16_t taps, uint16_t delta, float mu)
{
  /* Pasaaltos RC discreto: a = RC / (RC + dt), RC = 1 / (2 pi fc) */
  float rc = 1.0f / (2.0f * PI_F * FILTRO_FC_HZ);
  float dt = 1.0f / fs;

  f->tipo = FILTRO_OFF;
  f->fs = fs;
  f->hp_a = rc / (rc + dt);
  if (tipo == FILTRO_NLMS)
  {
    if (nlms_iniciar(&f->nlms, taps, delta, mu) != 0)
    {
      return -1;
    }
  }
  else if (tipo != FILTRO_OFF)
  {
    return -1;
  }
  f->tipo = tipo;
  filtro_reiniciar(f);
  return 0;
}

void filtro_reiniciar(filtro_t *f)
{
  f->hp_listo = false;
  f->hp_x1 = 0.0f;
  f->hp_y1 = 0.0f;
  if (f->tipo == FILTRO_NLMS)
  {
    nlms_reiniciar(&f->nlms);
  }
}

/* 1. Preprocesamiento: y[n] = a (y[n-1] + x[n] - x[n-1]). La primera muestra
      inicializa el estado: la gravedad no entra como un escalon. */
static float pasaaltos(filtro_t *f, float x)
{
  float y;

  if (!f->hp_listo)
  {
    f->hp_x1 = x;
    f->hp_y1 = 0.0f;
    f->hp_listo = true;
    return 0.0f;
  }
  y = f->hp_a * (f->hp_y1 + x - f->hp_x1);
  f->hp_x1 = x;
  f->hp_y1 = y;
  return y;
}

float filtro_procesar(filtro_t *f, float x, float *e)
{
  float xp = pasaaltos(f, x);

  /* 2. Algoritmo adaptativo: aca se agregan los futuros */
  switch (f->tipo)
  {
    case FILTRO_NLMS:
      return nlms_procesar(&f->nlms, xp, e);
    case FILTRO_OFF:
    default:
      *e = 0.0f;
      return xp;
  }
}
