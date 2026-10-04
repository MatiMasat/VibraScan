# VibraScan — contexto para Claude

Adquisición de vibraciones con un ADXL345 sobre una Black Pill (STM32F411CEU6),
para la beca Vibra-Scan. Se construye por etapas: cada etapa se prueba sola
(prueba automática en Python + pruebas manuales del usuario) antes de pasar
a la siguiente.

El detalle técnico de la etapa actual (conexiones, protocolo, comandos,
chequeos de aprobación, próximas etapas) está en el `README.md`. Leerlo antes
de tocar código; no repetirlo acá.

## Repositorio y etapas

Todo vive en un solo repo git:
`F:\Users\Usuario\Desktop\UTN\Beca de servicio\VibraScan\repo`, con remoto
https://github.com/MatiMasat/VibraScan.git.

- `main` tiene solo etapas aprobadas. Cada etapa aprobada lleva un tag anotado
  (`etapa-1`, `etapa-2`, ...): `git checkout etapa-1` vuelve a esa versión.
- Una etapa nueva se trabaja en la rama `etapa-N`, con un commit por cada paso
  que funciona. Al aprobarse:
  1. `git switch main` y `git merge --no-ff refs/heads/etapa-N -m "..."`
     (`--no-ff`: en Git Graph la etapa se ve como rama con sus pasos).
  2. `git tag -a etapa-N -m "..."` sobre el merge y `git push origin main etapa-N`.
  3. **Borrar la rama**, primero en GitHub y después la local
     (`git push origin --delete refs/heads/etapa-N` y `git branch -d etapa-N`;
     al revés, `-d` se niega si la copia de GitHub quedó atrás): el tag tiene el mismo
     nombre y, si conviven, git los confunde ("refname is ambiguous"). Los
     pasos quedan en el historial del merge.
  Mientras convivan rama y tag, usar `refs/heads/...` y `refs/tags/...`.
- No se crean más carpetas por etapa (`VibraScan_v3`, etc.). Las carpetas
  `VibraScan/` y `VibraScan_v2/` que están al lado de `repo/` son copias
  viejas; no se usan.
- El proyecto de Eclipse se sigue llamando `VibraScan_v2` (salida
  `Debug/VibraScan_v2.elf`).

| Tag | Etapa | Estado |
|---|---|---|
| `etapa-1` | Polling de DATA_READY a 100 Hz, texto por USB CDC | Aprobada |
| `etapa-2` | Interrupción DATA_READY (INT1 → PB0, EXTI0), I2C 400 kHz, hasta 800 Hz | Aprobada el 2026-10-03 (automática y manual) |
| `etapa-3` | Bloques de 32 muestras y trama binaria con CRC-16, `start`/`stop`, huecos con `t0` | Aprobada el 2026-10-04 (automática 10 min y manual) |
| `etapa-4` | Captura a archivo (CSV de cuentas + JSON, segmentos, reconexión), visor con envolvente, `rango=`, descarte de asentamiento | Aprobada el 2026-10-04 (reposo 30 min, golpes, manual) |
| (sin rama) | 5: preproceso y FFT/PSD en Python | Próxima |

Desde la etapa 3 el enlace es binario (ver README y `pc/vibrascan_proto.py`):
un monitor serie ya no sirve para mirar los datos; usar
`python vibrascan_proto.py --port COM8 --dump`.

## Cómo trabajar con el usuario

- **Consultar antes de modificar código.** Proponer el cambio, explicarlo y
  esperar el OK.
- En cada cambio, decir qué se hizo, qué se hará después y **qué pruebas debe
  hacer el usuario** (por ejemplo, qué se tiene que ver en el gráfico).
- Las pruebas manuales se hacen con `monitor_vivo.py`, que deja un `.log` con
  eventos y resumen en `pc/capturas/`: leerlo para ver cómo fueron, además de
  lo que cuente el usuario.
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
    -import "<ruta del repo>" -build "VibraScan_v2/Debug"
  ```
  La primera vez va `-import`; después alcanza con `-build`, o `-cleanBuild`
  para una compilación limpia. Salida: `Debug/VibraScan_v2.elf`. Tiene que
  compilar sin warnings.
- **Grabar** (con el ST-Link conectado):
  ```
  "/f/Sofwares/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe" -c port=SWD mode=UR -w "<.elf>" -v -rst
  ```
  Para leer RAM o registros sin resetear se usa `mode=HOTPLUG`.
- **Probar** (con el USB conectado), desde `pc/`: `etapa3_prueba.py`,
  `captura.py` + `etapa4_prueba.py`, `monitor_vivo.py` y `ver_captura.py`
  (sin placa: `python -m unittest test_vibrascan_proto test_vibrascan_archivo`;
  `ver_captura.py --png` sirve para mirar capturas sin abrir ventana). Los
  argumentos están en el README. En la PC de escritorio la Black Pill aparece
  como **COM8**. Se puede ver con `--list`.
- Python 3.9, con `pyserial`, `matplotlib` y `numpy` (`pc/requirements.txt`). Las
  capturas se guardan en `pc/capturas/` (ignoradas en `.gitignore`).

## Convenciones del código

- `main.c` lo genera CubeMX y solo llama a `app_init()` / `app_loop()`. La lógica
  de la etapa va en `app.c`.
- `adxl345.c` no conoce HAL ni el bus: recibe funciones read/write. El paso a
  SPI (etapa 6) solo cambia esas funciones.
- `usb_link.c` es lo único que conoce el transporte USB CDC.
- Comentarios y nombres en español, sin tildes en el código C.
- Protocolo binario (etapa 3): tramas `A5 5A` + tipo + largo + seq + contenido
  + CRC-16, definido igual en `Core/Src/trama.c` y `pc/vibrascan_proto.py`.
  Si cambia, cambiar los dos y correr `test_vibrascan_proto`.
- En el firmware, al comparar ticks que escribe una interrupción, restar con
  signo o leer la hora de nuevo (un bug de la etapa 3 salió de ahí).
- El `.ioc` se editó a mano. **No usar "Generate Code"** sin crear antes una
  rama o un commit: los archivos de `USB_DEVICE/` no tienen bloques USER CODE
  y se sobrescriben (con git se recuperan con `git checkout -- USB_DEVICE`).
