/**
  * @file    sensor_spi.c
  * @brief   Bus SPI del ADXL345 (etapa 6). Ver sensor_spi.h.
  */
#include "sensor_spi.h"

#include <string.h>

#include "main.h"

#define SPI_TIMEOUT_MS     5U
#define ADDR_READ           0x80U   /* primer byte: R/W */
#define ADDR_MULTI          0x40U   /* primer byte: varios bytes seguidos */

#define CS_PORT            GPIOA
#define CS_PIN             GPIO_PIN_4

static SPI_HandleTypeDef hspi1;
static DMA_HandleTypeDef hdma_rx;
static DMA_HandleTypeDef hdma_tx;

/* El DMA lee y escribe estos buffers despues de que se lanza la
   transferencia: tienen que ser propios y no tocarse mientras dure. */
static uint8_t tx_buf[SENSOR_SPI_MAX_READ + 1U];
static uint8_t rx_buf[SENSOR_SPI_MAX_READ + 1U];
static volatile uint16_t async_len;
static sensor_spi_done_fn async_done;
static volatile uint32_t cs_high_cyc;      /* DWT->CYCCNT cuando subio CS */
static uint32_t gap_cyc;

static void cs_low(void)
{
  /* Respeta el tiempo minimo con CS en alto desde la transaccion anterior */
  while ((DWT->CYCCNT - cs_high_cyc) < gap_cyc)
  {
  }
  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
  cs_high_cyc = DWT->CYCCNT;
}

static uint8_t addr_byte(uint8_t reg, uint16_t len, bool read)
{
  return (uint8_t)((reg & 0x3FU) | (read ? ADDR_READ : 0U) | ((len > 1U) ? ADDR_MULTI : 0U));
}

void sensor_spi_init(void)
{
  GPIO_InitTypeDef g = {0};

  /* El contador de ciclos mide la espera entre transacciones */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  gap_cyc = (SystemCoreClock / 1000000U) * SENSOR_SPI_GAP_US;
  cs_high_cyc = DWT->CYCCNT - gap_cyc;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* CS: salida, en alto (sensor no seleccionado) antes de configurar el pin */
  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
  g.Pin = CS_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_PORT, &g);

  /* PA5 SCK, PA6 MISO, PA7 MOSI. MISO con pull-up: sin sensor se lee 0xFF
     (DEVID incorrecto) en vez de un valor al azar. */
  g.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_6;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);

  /* SPI1 en APB2 (96 MHz) / 32 = 3 MHz (el maximo del sensor es 5 MHz; /16
     daria 6 MHz). Modo 3: CPOL = 1, CPHA = 1. */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  /* DMA2: SPI1_RX = Stream0 canal 3, SPI1_TX = Stream3 canal 3 */
  hdma_rx.Instance = DMA2_Stream0;
  hdma_rx.Init.Channel = DMA_CHANNEL_3;
  hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode = DMA_NORMAL;
  hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_rx) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&hspi1, hdmarx, hdma_rx);

  hdma_tx = hdma_rx;
  hdma_tx.Instance = DMA2_Stream3;
  hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  if (HAL_DMA_Init(&hdma_tx) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&hspi1, hdmatx, hdma_tx);

  /* Misma prioridad que tenia el I2C: por encima de la EXTI (1) y del USB (2) */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

int sensor_spi_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint8_t tx[SENSOR_SPI_MAX_READ + 1U] = {0};
  uint8_t rx[SENSOR_SPI_MAX_READ + 1U];
  HAL_StatusTypeDef st;

  (void)ctx;
  if ((len == 0U) || (len > SENSOR_SPI_MAX_READ))
  {
    return -1;
  }
  tx[0] = addr_byte(reg, len, true);
  cs_low();
  st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(len + 1U), SPI_TIMEOUT_MS);
  cs_high();
  if (st != HAL_OK)
  {
    return -1;
  }
  memcpy(buf, &rx[1], len);
  return 0;
}

int sensor_spi_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
  uint8_t tx[SENSOR_SPI_MAX_READ + 1U];
  HAL_StatusTypeDef st;

  (void)ctx;
  if ((len == 0U) || (len > SENSOR_SPI_MAX_READ))
  {
    return -1;
  }
  tx[0] = addr_byte(reg, len, false);
  memcpy(&tx[1], buf, len);
  cs_low();
  st = HAL_SPI_Transmit(&hspi1, tx, (uint16_t)(len + 1U), SPI_TIMEOUT_MS);
  cs_high();
  return (st == HAL_OK) ? 0 : -1;
}

bool sensor_spi_read_async(uint8_t reg, uint16_t len, sensor_spi_done_fn done)
{
  if ((len == 0U) || (len > SENSOR_SPI_MAX_READ))
  {
    return false;
  }
  memset(tx_buf, 0, sizeof(tx_buf));
  tx_buf[0] = addr_byte(reg, len, true);
  async_len = len;
  async_done = done;
  cs_low();
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, rx_buf, (uint16_t)(len + 1U)) != HAL_OK)
  {
    cs_high();
    return false;
  }
  return true;
}

void sensor_spi_abort(void)
{
  (void)HAL_SPI_Abort(&hspi1);
  cs_high();
}

static const struct { uint32_t div; uint32_t prescaler; } SPI_DIVS[] = {
  { 32U, SPI_BAUDRATEPRESCALER_32 }, { 64U, SPI_BAUDRATEPRESCALER_64 },
  { 128U, SPI_BAUDRATEPRESCALER_128 }, { 256U, SPI_BAUDRATEPRESCALER_256 },
};

bool sensor_spi_set_khz(uint32_t khz)
{
  uint32_t pclk_khz = HAL_RCC_GetPCLK2Freq() / 1000U;

  for (uint32_t i = 0; i < sizeof(SPI_DIVS) / sizeof(SPI_DIVS[0]); i++)
  {
    if (pclk_khz / SPI_DIVS[i].div == khz)
    {
      hspi1.Init.BaudRatePrescaler = SPI_DIVS[i].prescaler;
      return HAL_SPI_Init(&hspi1) == HAL_OK;
    }
  }
  return false;
}

uint32_t sensor_spi_khz(void)
{
  for (uint32_t i = 0; i < sizeof(SPI_DIVS) / sizeof(SPI_DIVS[0]); i++)
  {
    if (SPI_DIVS[i].prescaler == hspi1.Init.BaudRatePrescaler)
    {
      return HAL_RCC_GetPCLK2Freq() / 1000U / SPI_DIVS[i].div;
    }
  }
  return 0U;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    cs_high();
    if (async_done != NULL)
    {
      async_done(&rx_buf[1], async_len);
    }
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    cs_high();
    if (async_done != NULL)
    {
      async_done(NULL, 0);
    }
  }
}

void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_rx);
}

void DMA2_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tx);
}
