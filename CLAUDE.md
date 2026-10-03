# VibraScan — contexto para Claude

Adquisición de vibraciones con un ADXL345 sobre una Black Pill (STM32F411CEU6),
para la beca Vibra-Scan. Se construye por etapas: cada etapa se prueba sola
(prueba automática en Python + pruebas manuales del usuario) antes de pasar
a la siguiente.

El detalle técnico de cada etapa (conexiones, protocolo, comandos, chequeos de
aprobación, próximas etapas) está en el `README.md` de cada proyecto. Leerlo
antes de tocar código; no repetirlo acá.

## Proyectos

| Carpeta | Etapa | Estado |
|---|---|---|
| `../VibraScan` | 1: polling de DATA_READY a 100 Hz, texto por USB CDC | Aprobada. Queda como referencia; no modificar. |
| `../VibraScan_v2` | 2: interrupción DATA_READY (INT1 → PB0, EXTI0), I2C 400 kHz, hasta 800 Hz | Parte automática aprobada (ver abajo). **Es el proyecto activo.** |

Cada etapa nueva arrancó como copia de la anterior en una carpeta propia.
Antes de abrir una etapa nueva, preguntar si se sigue ese esquema.

### Estado de la etapa 2 (2026-10-02)

- Aprobada la prueba automática: 100/400/800 Hz × 20 s y 800 Hz × 60 s
  (47 686 muestras, 0 saltos, todos los contadores en 0, periodo 1257–1258 µs).
- **Pendiente:** el usuario tiene que confirmar las pruebas manuales con
  `python monitor_vivo.py --port COMx` desde `VibraScan_v2\pc`:
  1. Reposo: la curva ocupa los 10 s de la ventana; el título muestra
     `sps` ≈ 795 y `periodo=1257-1258 us`. (Antes había un problema de eje
     horizontal recortado o desplazado; confirmar que quedó resuelto.)
  2. Caras: un eje en ±1 g y los otros dos cerca de 0.
  3. Golpe en la mesa: pico corto y definido, sin que el gráfico se corte.
  4. Robustez: soltar INT1 de PB0 unos segundos y reconectar. El gráfico se
     congela, se recupera solo y `stall` sube.
  5. `saltos=0` en el título durante toda la prueba.
- Cuando lo confirme, se cierra la etapa 2 y se sigue con la 3 (buffer por
  bloques y trama binaria con checksum).

## Cómo trabajar con el usuario

- **Consultar antes de modificar código.** Proponer el cambio, explicarlo y
  esperar el OK.
- En cada cambio, decir qué se hizo, qué se hará después y **qué pruebas debe
  hacer el usuario** (por ejemplo, qué se tiene que ver en el gráfico).
- **Nunca ST-Link y USB conectados a la vez.** Unir los dos 3,3 V hizo fallar
  uno de los USB. El usuario conecta uno solo por vez, y Claude le avisa con
  claridad cuándo cambiar:
  - ST-Link (SWDIO, SWCLK, GND; sin 3,3 V) → para grabar y leer memoria.
  - USB de la Black Pill → para las pruebas por el puerto COM.
- Respuestas en español.

## Compilar, grabar y probar

Las rutas son de la PC de escritorio. En otra máquina (por ejemplo la notebook)
hay que ubicar las equivalentes y, si cambian, anotarlas acá.

- **Compilar sin abrir el IDE** (STM32CubeIDE 2.1.1, build headless, con un
  workspace temporal en el scratchpad):
  ```
  "C:/Users/matim/STM32CubeIDE/STM32CubeIDE_2.1.1/STM32CubeIDE/stm32cubeidec.exe" \
    --launcher.suppressErrors -nosplash \
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
    -data "<scratchpad>/ws_build" \
    -import "<ruta del proyecto>" -build "VibraScan_v2/Debug"
  ```
  La primera vez va `-import`; después alcanza con `-build`, o `-cleanBuild`
  para una compilación limpia. Salida: `Debug/VibraScan_v2.elf`. Tiene que
  compilar sin warnings.
- **Grabar** (con el ST-Link conectado):
  ```
  "/f/Sofwares/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe" -c port=SWD mode=UR -w "<.elf>" -v -rst
  ```
  Para leer RAM o registros sin resetear se usa `mode=HOTPLUG`.
- **Probar** (con el USB conectado): `pc/etapa2_prueba.py` y
  `pc/monitor_vivo.py`; los argumentos están en el README. En la PC de
  escritorio la Black Pill aparece como **COM8**. Se puede ver con `--list`.
- Python 3.9, con `pyserial` y `matplotlib` (`pc/requirements.txt`). Las
  capturas CSV se guardan en `pc/capturas/` (ignoradas en `.gitignore`).

## Convenciones del código

- `main.c` lo genera CubeMX y solo llama a `app_init()` / `app_loop()`. La lógica
  de la etapa va en `app.c`.
- `adxl345.c` no conoce HAL ni el bus: recibe funciones read/write. El paso a
  SPI (etapa 6) solo cambia esas funciones.
- `usb_link.c` es lo único que conoce el transporte USB CDC.
- Comentarios y nombres en español, sin tildes en el código C.
- Protocolo de texto: `n,x,y,z` = muestra; `# ...` = estado (1 por segundo);
  `> ...` = respuesta a un comando de la PC.
- El `.ioc` se editó a mano. **No usar "Generate Code"** sin hacer antes una
  copia de la carpeta: los archivos de `USB_DEVICE/` no tienen bloques USER
  CODE y se sobrescriben.
