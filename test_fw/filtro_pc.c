/**
  * El filtro del micro (filtro.c + nlms.c) compilado en la PC, para compararlo
  * con la referencia de Python (pc/test_vibrascan_filtro.py lo compila y lo
  * usa). Lee muestras float32 de un archivo y escribe y, e (float32).
  *
  *   gcc -std=c99 -O2 -Wall -Wextra -I Core/Inc test_fw/filtro_pc.c Core/Src/filtro.c Core/Src/nlms.c -o filtro_pc
  *   filtro_pc entrada.f32 salida.f32 fs taps delta mu
  */
#include <stdio.h>
#include <stdlib.h>

#include "filtro.h"

int main(int argc, char **argv)
{
  filtro_t f;
  FILE *in, *out;
  float x, y, e;

  if (argc != 7)
  {
    fprintf(stderr, "uso: %s entrada.f32 salida.f32 fs taps delta mu\n", argv[0]);
    return 2;
  }
  if (filtro_configurar(&f, FILTRO_NLMS, (float)atof(argv[3]), (uint16_t)atoi(argv[4]),
                        (uint16_t)atoi(argv[5]), (float)atof(argv[6])) != 0)
  {
    fprintf(stderr, "parametros no validos\n");
    return 2;
  }
  in = fopen(argv[1], "rb");
  out = fopen(argv[2], "wb");
  if ((in == NULL) || (out == NULL))
  {
    fprintf(stderr, "no se pudo abrir un archivo\n");
    return 2;
  }
  while (fread(&x, sizeof(x), 1, in) == 1)
  {
    y = filtro_procesar(&f, x, &e);
    fwrite(&y, sizeof(y), 1, out);
    fwrite(&e, sizeof(e), 1, out);
  }
  fclose(in);
  fclose(out);
  return 0;
}
