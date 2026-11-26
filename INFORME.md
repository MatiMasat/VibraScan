# Guía para redactar el informe (parte del acelerómetro)

Esta rama (`informe`) reúne todo lo necesario para redactar la parte del acelerómetro del
informe final de la beca Vibra-Scan (UTN-FRP, 2026). La parte del micrófono la escribió el
otro integrante de la beca (PDF aparte, no está en el repo). No hay que modificar el código:
la rama es de lectura.

## Qué leer y en qué orden

| Archivo | Qué tiene |
|---|---|
| [README.md](README.md) | El sistema por etapas: sensor, conexiones, protocolo, pruebas de las etapas 1 a 6 |
| [docs/filtro_adaptativo.md](docs/filtro_adaptativo.md) | Filtro NLMS en el micro (etapa 7): por qué, cómo, resultados con tonos |
| [docs/prueba_motor.md](docs/prueba_motor.md) | Pruebas con el motor. **La sección 8 es la principal para el informe** |
| [docs/img/niveles/](docs/img/niveles/) | Figuras de la sección 8 (las 10 comparativas y los 10 espectrogramas) |
| `Core/Src/app.c`, `adxl345.c`, `sensor_spi.c`, `filtro.c`, `nlms.c` | Firmware, solo para explicar a grandes rasgos cómo funciona |
| `pc/vibrascan_espectro.py`, `vibrascan_motor.py`, `captura.py`, `monitor_vivo.py` | Programas de la PC |

Las capturas crudas (`pc/capturas/`) no están en git: usar los resultados documentados.

## Cadena del sistema

ADXL345 (acelerómetro, ±16 g, 3200 muestras/s) → SPI con FIFO y DMA → STM32F411 (Black Pill):
filtro adaptativo NLMS en tiempo real sobre un eje → USB → PC: captura a archivo, espectro (FFT,
método de Welch), espectrograma y análisis. **La FFT se calcula en la PC, no en el micro.**

## Ensayos, de más general a más específico

1. Etapas 1 a 6: adquisición sin pérdidas hasta 3200 Hz, verificada con tonos de frecuencia conocida.
2. Etapa 7: filtro NLMS en el micro, mismo resultado que Python, mejora de la señal con tonos.
3. Motor (30/10): línea de 100 Hz, rotación (~24,5 Hz) y su velocidad (`prueba_motor.md`, secciones 1–6).
4. Apoyo movido a mano (06/11): sección 7.
5. **Cinco niveles del selector con el apoyo normal y desplazado ≈ 3 mm (24/11): sección 8.** Es
   el ensayo equivalente al del micrófono, con las mismas figuras.

## Convenciones acordadas

- **Numeración del selector:** la del informe del micrófono, **Nivel 1 = máxima potencia,
  Nivel 5 = mínima**. Hay una contradicción sin resolver con esa numeración (sección 8,
  "Comparación con el micrófono").
- **"Freno manual" o "carga simulada"** en el informe del micrófono = el mismo desplazamiento del
  apoyo ≈ 3 mm hacia arriba. Acá se llama "apoyo desplazado".
- Las líneas se nombran por su relación con la rotación: 1× (~24,5 Hz), 2× (~49 Hz), y la línea
  eléctrica de 100 Hz (2 × la red de 50 Hz).

## Reglas para el texto

- Usar solo datos de estos archivos o de lo que diga el autor. Si falta algo, preguntar.
- Separar lo **medido**, lo **interpretado** y las **hipótesis**. Las secciones "Alcance" de
  `prueba_motor.md` dicen qué se puede afirmar.
- "Compatible con una desalineación", no "evidencia desalineación": no se midió la alineación.
- La velocidad del motor se estimó del espectro; no se verificó con tacómetro.
- Al comparar con el micrófono: comparar qué líneas aparecen y cómo cambian, no los niveles (son
  magnitudes distintas y las mediciones no fueron simultáneas).
- El texto tiene que describir lo que se ve en cada figura que se use.

## Lectores

Docentes de la facultad, no todos especialistas en vibraciones. Español claro y sencillo.
