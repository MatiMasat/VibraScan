/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          : Implementacion de la interfaz CDC. CDC_Transmit_FS() es
  *                   la funcion que usb_link.c usa para mandar datos por el
  *                   puerto COM virtual.
  ******************************************************************************
  */
#include "usbd_cdc_if.h"
#include "usb_link.h"

#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  64

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

/* No necesitamos ninguno de los line-coding/control requests para este
   proyecto (solo transmitimos datos), asi que se ignoran todos. */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  UNUSED(cmd);
  UNUSED(pbuf);
  UNUSED(length);
  return (USBD_OK);
}

/* No usamos recepcion (solo transmitimos muestras del sensor), pero hay que
   volver a armar el buffer de recepcion para que la pila USB no se cuelgue. */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
  usb_link_on_receive(Buf, *Len);      /* comandos de la PC, ver usb_link.c */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

/**
  * @brief  Manda datos por el puerto COM virtual. Devuelve USBD_BUSY si la
  *         transferencia anterior todavia no termino (el host no leyo lo
  *         suficientemente rapido). OJO: Buf se transmite despues de que
  *         esta funcion retorna, no hay que modificarlo hasta que termine
  *         (usb_link.c se encarga de eso).
  */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  if (hcdc == NULL)
  {
    return USBD_FAIL;   /* la PC todavia no enumero el dispositivo */
  }
  if (hcdc->TxState != 0)
  {
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  return result;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  return (USBD_OK);
}
