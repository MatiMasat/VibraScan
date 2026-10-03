/**
  * @file    usb_link.h
  * @brief   Envio de datos a la PC por el puerto COM virtual (USB CDC).
  *
  * Es la unica parte de la aplicacion que conoce el transporte. Si mas
  * adelante se cambia a UART, se reemplaza usb_link.c y nada mas.
  */
#ifndef USB_LINK_H
#define USB_LINK_H

#include <stdint.h>
#include <stdbool.h>

/** true si la PC ya enumero el dispositivo (el puerto COM existe). */
bool usb_link_ready(void);

/** Envia len bytes. Devuelve true si quedaron en camino a la PC y false si
  * se descartaron (PC desconectada, puerto cerrado o PC que no lee). Nunca
  * bloquea mas de unos pocos milisegundos. */
bool usb_link_write(const uint8_t *data, uint16_t len);

/** printf hacia la PC. Devuelve lo mismo que usb_link_write. */
bool usb_link_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif /* USB_LINK_H */
