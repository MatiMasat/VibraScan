/**
  * @file    app.h
  * @brief   Aplicacion VibraScan. main.c solo llama a estas dos funciones.
  */
#ifndef APP_H
#define APP_H

void app_init(void);   /* una vez, despues de inicializar los perifericos */
void app_loop(void);   /* en cada vuelta del while(1) */

#endif /* APP_H */
