/**
  * @file    usb_link.c
  * @brief   Envio de datos a la PC por el puerto COM virtual (USB CDC).
  */
#include "usb_link.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#define TX_BUF_SIZE       USB_LINK_MAX_WRITE
#define PRINTF_BUF_SIZE   160U   /* va en la pila: no usar TX_BUF_SIZE */
#define TX_WAIT_MS        5U     /* espera maxima a que termine el envio anterior */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* La pila USB transmite desde este buffer DESPUES de que CDC_Transmit_FS
   retorna, asi que tiene que ser propio y no tocarse mientras haya un envio
   en curso. */
static uint8_t tx_buf[TX_BUF_SIZE];

/* Recepcion: la interrupcion USB escribe y app_loop lee. */
#define RX_RING_SIZE      128U   /* potencia de 2 */
static uint8_t           rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head;
static volatile uint32_t rx_tail;

/* false = el ultimo envio no salio. Mientras este asi no se espera nada:
   sin PC escuchando, el lazo de muestreo no se frena. */
static bool link_up = true;

bool usb_link_ready(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) &&
         (hUsbDeviceFS.pClassData != NULL);
}

static bool tx_busy(void)
{
  const USBD_CDC_HandleTypeDef *hcdc = (const USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  return hcdc->TxState != 0U;
}

bool usb_link_can_write(void)
{
  return usb_link_ready() && !tx_busy();
}

bool usb_link_write(const uint8_t *data, uint16_t len)
{
  if ((len == 0U) || (len > TX_BUF_SIZE) || !usb_link_ready())
  {
    link_up = false;
    return false;
  }

  if (tx_busy())
  {
    uint32_t t0 = HAL_GetTick();
    uint32_t wait = link_up ? TX_WAIT_MS : 0U;

    while (tx_busy())
    {
      if ((HAL_GetTick() - t0) >= wait)
      {
        link_up = false;
        return false;
      }
      if (!usb_link_ready())
      {
        link_up = false;
        return false;
      }
    }
  }

  memcpy(tx_buf, data, len);
  if (CDC_Transmit_FS(tx_buf, len) != USBD_OK)
  {
    link_up = false;
    return false;
  }
  link_up = true;
  return true;
}

bool usb_link_printf(const char *fmt, ...)
{
  char line[PRINTF_BUF_SIZE];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);

  if (n <= 0)
  {
    return false;
  }
  if (n >= (int)sizeof(line))
  {
    n = (int)sizeof(line) - 1;   /* linea truncada */
  }
  return usb_link_write((const uint8_t *)line, (uint16_t)n);
}

void usb_link_on_receive(const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    if ((rx_head - rx_tail) >= RX_RING_SIZE)
    {
      break;                           /* lleno: se pierde el resto */
    }
    rx_ring[rx_head & (RX_RING_SIZE - 1U)] = data[i];
    rx_head = rx_head + 1U;
  }
}

bool usb_link_read_line(char *line, uint32_t max)
{
  static char     acc[64];
  static uint32_t acc_len;
  static bool     overflow;

  while (rx_tail != rx_head)
  {
    char c = (char)rx_ring[rx_tail & (RX_RING_SIZE - 1U)];
    rx_tail = rx_tail + 1U;

    if ((c == '\r') || (c == '\n'))
    {
      bool ok = !overflow && (acc_len > 0U) && (acc_len < max);
      if (ok)
      {
        memcpy(line, acc, acc_len);
        line[acc_len] = '\0';
      }
      acc_len = 0;
      overflow = false;
      if (ok)
      {
        return true;
      }
    }
    else if (acc_len < sizeof(acc))
    {
      acc[acc_len++] = c;
    }
    else
    {
      overflow = true;
    }
  }
  return false;
}
