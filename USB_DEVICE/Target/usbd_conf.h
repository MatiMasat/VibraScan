/**
  ******************************************************************************
  * @file           : usbd_conf.h
  * @brief          : Header for usbd_conf.c file.
  ******************************************************************************
  */
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USBD_MAX_NUM_INTERFACES               1U
#define USBD_MAX_NUM_CONFIGURATION            1U
#define USBD_MAX_STR_DESC_SIZ                 512U
#define USBD_SUPPORT_USER_STRING_DESC         0U
#define USBD_SELF_POWERED                     0U
#define USBD_DEBUG_LEVEL                      0U
#define DEVICE_FS                             0U

/* Memory management macros (no custom heap used) */
#define USBD_malloc               (void *)USBD_static_malloc
#define USBD_free                 USBD_static_free
#define USBD_memset               memset
#define USBD_memcpy               memcpy
#define USBD_Delay                HAL_Delay

/* For footprint reasons and since only one allocation is handled in the
   HID class driver, the malloc/free is changed into a static allocation
   method */
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)   printf(__VA_ARGS__); printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)
#define USBD_ErrLog(...)   printf("ERROR: ") ; printf(__VA_ARGS__); printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)   printf("DEBUG : ") ; printf(__VA_ARGS__); printf("\n");
#else
#define USBD_DbgLog(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H */
