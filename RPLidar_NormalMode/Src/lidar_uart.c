/*
 * lidar_uart.c
 *
 *  Created on: 10-Jun-2019
 *      Author: gautam
 */

#include "lidar_uart.h"
#include "stm32mp1xx_hal.h"
#include "res_mgr.h"

#include <string.h>

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;

void MX_UART7_Init(void)
{

  if (ResMgr_Request(RESMGR_ID_UART7, RESMGR_FLAGS_ACCESS_NORMAL | \
                  RESMGR_FLAGS_CPU_SLAVE , 0, NULL) != RESMGR_OK)
  {
    /* USER CODE BEGIN RESMGR_UTILITY_UART7 */
    Error_Handler();
    /* USER CODE END RESMGR_UTILITY_UART7 */
  }
  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}


void lidar_uart_tx(unsigned char *data, unsigned char length){

	HAL_UART_Transmit(&huart7, data,length , 0xFFFF);
}

void lidar_uart_tx_byte(uint8_t data){

	HAL_UART_Transmit(&huart7, &data, strlen((const char*)&data), 0xFFFF);

}

uint8_t lidar_uart_rx(){
	uint8_t data;
	HAL_UART_Receive(&huart7, &data, 1, 0xFFFF);
	return data;
}

void lidar_uart_rx_buffer(uint8_t *data,size_t length){

	HAL_UART_Receive(&huart7, data, length, 0xFFFF);

}


