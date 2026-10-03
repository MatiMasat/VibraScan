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

/* Maximo de bytes por llamada a usb_link_write. */
#define USB_LINK_MAX_WRITE   512U

/** true si la PC ya enumero el dispositivo (el puerto COM existe). */
bool usb_link_ready(void);

/** true si un usb_link_write ahora saldria sin esperar (el envio anterior ya
  * termino). Sirve para no bloquear: si da false, reintentar mas tarde. */
bool usb_link_can_write(void);

/** Envia len bytes. Devuelve true si quedaron en camino a la PC y false si
  * se descartaron (PC desconectada, puerto cerrado o PC que no lee). Nunca
  * bloquea mas de unos pocos milisegundos. */
bool usb_link_write(const uint8_t *data, uint16_t len);

/** printf hacia la PC. Devuelve lo mismo que usb_link_write. */
bool usb_link_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** Copia en line la proxima linea completa que mando la PC (sin el fin de
  * linea, terminada en 0) y devuelve true. Devuelve false si todavia no hay
  * una linea completa. Las lineas mas largas que max-1 se descartan. */
bool usb_link_read_line(char *line, uint32_t max);

/** Lo llama la pila USB (interrupcion) con cada paquete recibido de la PC. */
void usb_link_on_receive(const uint8_t *data, uint32_t len);

#endif /* USB_LINK_H */
