/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "resmgr_utility.h"
#include "openamp.h"
#include "RPLidar.h"
#include "lidar_uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

IPCC_HandleTypeDef hipcc;

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;

VIRT_UART_HandleTypeDef huart0;
VIRT_UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;

__IO FlagStatus VirtUart0RxMsg = RESET;
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;

__IO FlagStatus VirtUart1RxMsg = RESET;
uint8_t VirtUart1ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart1ChannelRxSize = 0;

uint16_t VirtUart0ChannelBuffTx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelTxSize = 0;


/*
 * Global Variables
 */

uint16_t lidarSensorData[720];
uint16_t lidarFrame1[180];
uint16_t lidarFrame2[180];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IPCC_Init(void);

int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);
void StartDefaultTask(void const * argument);

void sendmessageToA7(uint16_t *lidarData, uint8_t dataID, uint16_t dataSize);
void processLidarData();

/* USER CODE BEGIN PFP */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);

/* USER CODE END PFP */
uint16_t datafromSensor[360];
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
 {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE BEGIN Init */
	if (IS_ENGINEERING_BOOT_MODE()) {
		/* Configure the system clock */
		SystemClock_Config();
	}
		MX_IPCC_Init();
	  /* OpenAmp initialisation ---------------------------------*/
	  MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
	/* Resource Manager Utility initialisation ---------------------------------*/
	//MX_RESMGR_UTILITY_Init();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_UART7_Init();
	/* USER CODE BEGIN 2 */
	uint8_t firstFrame[17] = "Send First Halff";
	uint8_t secondFrame[18] = "Send Second Half";

	if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
		Error_Handler();
	}

	/*Need to register callback for message reception by channels*/
	if (VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID,
			VIRT_UART0_RxCpltCallback) != VIRT_UART_OK) {
		Error_Handler();
	}

	rplidar_response_device_info_t info;
	memset(&info, 0, sizeof(rplidar_response_device_info_t));
	u_result result_status=1;
	u_result data_status=1;
	if (IS_OK(getDeviceInfo(&info, RPLIDAR_DEFAULT_TIMEOUT))) {

		result_status = startScan(0, RPLIDAR_DEFAULT_TIMEOUT);
	}

	if (result_status == RESULT_OK) {

		while (1) {
			OPENAMP_check_for_message();

			/* USER CODE END WHILE */
			uint16_t lidarData[180] = { 0 };
			if (VirtUart0RxMsg) {
				VirtUart0RxMsg = RESET;
				VirtUart0ChannelTxSize = 364;

				if (0 == memcmp(VirtUart0ChannelBuffRx, firstFrame, 17)) {

					data_status = cacheScanData();

					if (data_status == RESULT_OK) { //If passes, full scan data is available in _cached_scan_node_hq_buf
						processLidarData();
					}
					//Identifier to notify that first frame is being sent.
					lidarData[0] = 0xEE;
					for (int i = 1; i < 180; i++) {
						lidarData[i] = datafromSensor[i - 1];
					}
				}

				if (0 == memcmp(VirtUart0ChannelBuffRx, secondFrame, 18)) {

					//Identifier to notify that second frame is being sent.
					lidarData[0] = 0xFF;
					for (int i = 1; i < 180; i++) {
						lidarData[i] = datafromSensor[179 + i];
					}

				}
				sendmessageToA7(lidarData, 10, 364);
			}
		}

	}
	/* USER CODE END 3 */

}
/*
 * Function to send data from sensor / UART to A7 core
 * @param lidarData IN
 * @param dataID - The message ID
 * @param datasize amount of data to be sent
 * */
void sendmessageToA7(uint16_t *lidarData, uint8_t dataID, uint16_t dataSize) {

	//Message ID
	VirtUart0ChannelBuffTx[0] = dataID;

	for (uint16_t i = 1; i <= dataSize; i++) {

		VirtUart0ChannelBuffTx[i] = lidarData[i - 1];
	}

	OPENAMP_send(&huart0.ept, VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);
}
/*
 * Collect the data from sensor once a complete value is obtained
 * This collected data is transfered to A7
 *  */
void processLidarData() {

	for (uint16_t index_sensorData = 0; index_sensorData < 360;
			index_sensorData++) {
		datafromSensor[index_sensorData] =
				_cached_scan_node_hq_buf[index_sensorData].dist_mm_q2;
	}
}

//
//	uint16_t lidardata_rearranged[360];
//
//	uint16_t i=0;
//
//	while (i < 720) {
//
//		uint16_t lidarAngle = lidarSensorData[i];
//		lidardata_rearranged[lidarAngle] = lidarSensorData[i + 1];
//		i += 2;
//	}
//
//	for (uint16_t j = 0; j < 720; j++) {
//		if (j == 180) {
//			lidarFrame2[180 - j] = lidardata_rearranged[j];
//		} else {
//			lidarFrame2[j] = lidardata_rearranged[j];
//		}
//	}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 3;
	RCC_OscInitStruct.PLL.PLLN = 81;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLFRACV = 0x800;
	RCC_OscInitStruct.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
	RCC_OscInitStruct.PLL.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
	RCC_OscInitStruct.PLL2.PLLM = 3;
	RCC_OscInitStruct.PLL2.PLLN = 66;
	RCC_OscInitStruct.PLL2.PLLP = 2;
	RCC_OscInitStruct.PLL2.PLLQ = 1;
	RCC_OscInitStruct.PLL2.PLLR = 1;
	RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
	RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
	RCC_OscInitStruct.PLL2.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL2.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
	RCC_OscInitStruct.PLL3.PLLM = 2;
	RCC_OscInitStruct.PLL3.PLLN = 34;
	RCC_OscInitStruct.PLL3.PLLP = 2;
	RCC_OscInitStruct.PLL3.PLLQ = 17;
	RCC_OscInitStruct.PLL3.PLLR = 37;
	RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
	RCC_OscInitStruct.PLL3.PLLFRACV = 6660;
	RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
	RCC_OscInitStruct.PLL3.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL3.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
	RCC_OscInitStruct.PLL4.PLLM = 4;
	RCC_OscInitStruct.PLL4.PLLN = 99;
	RCC_OscInitStruct.PLL4.PLLP = 6;
	RCC_OscInitStruct.PLL4.PLLQ = 8;
	RCC_OscInitStruct.PLL4.PLLR = 8;
	RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
	RCC_OscInitStruct.PLL4.PLLFRACV = 0;
	RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
	RCC_OscInitStruct.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** RCC Clock Config
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_ACLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3
			| RCC_CLOCKTYPE_PCLK4 | RCC_CLOCKTYPE_PCLK5 | RCC_CLOCKTYPE_MPU;
	RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;
	RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
	RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
	RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
	RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
	RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
	RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
	RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
	RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USBO
			| RCC_PERIPHCLK_DDRPHYC | RCC_PERIPHCLK_SAI2 | RCC_PERIPHCLK_RTC
			| RCC_PERIPHCLK_LTDC | RCC_PERIPHCLK_UART24 | RCC_PERIPHCLK_UART78
			| RCC_PERIPHCLK_SPI23 | RCC_PERIPHCLK_SDMMC12 | RCC_PERIPHCLK_I2C12
			| RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_I2C46 | RCC_PERIPHCLK_CEC
			| RCC_PERIPHCLK_RNG1 | RCC_PERIPHCLK_CKPER;
	PeriphClkInit.I2c12ClockSelection = RCC_I2C12CLKSOURCE_HSI;
	PeriphClkInit.I2c46ClockSelection = RCC_I2C46CLKSOURCE_HSI;
	PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLL3_Q;
	PeriphClkInit.Spi23ClockSelection = RCC_SPI23CLKSOURCE_PLL3_Q;
	PeriphClkInit.Uart24ClockSelection = RCC_UART24CLKSOURCE_HSI;
	PeriphClkInit.Uart78ClockSelection = RCC_UART78CLKSOURCE_BCLK;
	PeriphClkInit.EthClockSelection = RCC_ETHCLKSOURCE_PLL4;
	PeriphClkInit.DsiClockSelection = RCC_DSICLKSOURCE_PHY;
	PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_HSE;
	PeriphClkInit.Rng1ClockSelection = RCC_RNG1CLKSOURCE_LSI;
	PeriphClkInit.UsbphyClockSelection = RCC_USBPHYCLKSOURCE_HSE;
	PeriphClkInit.UsboClockSelection = RCC_USBOCLKSOURCE_PHY;
	PeriphClkInit.CecClockSelection = RCC_CECCLKSOURCE_LSE;
	PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_OFF;
	PeriphClkInit.Lptim23ClockSelection = RCC_LPTIM23CLKSOURCE_OFF;
	PeriphClkInit.Lptim45ClockSelection = RCC_LPTIM45CLKSOURCE_OFF;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PER;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInit.TIMG1PresSelection = RCC_TIMG1PRES_DEACTIVATED;
	PeriphClkInit.TIMG2PresSelection = RCC_TIMG2PRES_DEACTIVATED;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Set the HSE division factor for RTC clock
	 */
	__HAL_RCC_RTC_HSEDIV(24);
}

/**
 * @brief ETZPC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETZPC_Init(void) {

	/* USER CODE BEGIN ETZPC_Init 0 */

	/* USER CODE END ETZPC_Init 0 */

	/* USER CODE BEGIN ETZPC_Init 1 */

	/* USER CODE END ETZPC_Init 1 */
	/* USER CODE BEGIN ETZPC_Init 2 */

	/* USER CODE END ETZPC_Init 2 */

}

/**
 * @brief IPCC Initialization Function
 * @param None
 * @retval None
 */
static void MX_IPCC_Init(void) {

	/* USER CODE BEGIN IPCC_Init 0 */

	/* USER CODE END IPCC_Init 0 */

	/* USER CODE BEGIN IPCC_Init 1 */

	/* USER CODE END IPCC_Init 1 */
	hipcc.Instance = IPCC;
	if (HAL_IPCC_Init(&hipcc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IPCC_Init 2 */

	/* USER CODE END IPCC_Init 2 */

}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */

//static void MX_UART7_Init(void)
//{
//
//  if (ResMgr_Request(RESMGR_ID_UART7, RESMGR_FLAGS_ACCESS_NORMAL | \
//                   RESMGR_FLAGS_CPU_SLAVE , 0, NULL) != RESMGR_OK)
//  {
//    /* USER CODE BEGIN RESMGR_UTILITY_UART7 */
//    Error_Handler();
//    /* USER CODE END RESMGR_UTILITY_UART7 */
//  }
//  /* USER CODE BEGIN UART7_Init 0 */
//
//  /* USER CODE END UART7_Init 0 */
//
//  /* USER CODE BEGIN UART7_Init 1 */
//
//  /* USER CODE END UART7_Init 1 */
//  huart7.Instance = UART7;
//  huart7.Init.BaudRate = 115200;
//  huart7.Init.WordLength = UART_WORDLENGTH_8B;
//  huart7.Init.StopBits = UART_STOPBITS_1;
//  huart7.Init.Parity = UART_PARITY_NONE;
//  huart7.Init.Mode = UART_MODE_TX_RX;
//  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart7) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN UART7_Init 2 */
//
//  /* USER CODE END UART7_Init 2 */
//
//}
/** 
 * Enable DMA controller clock
 */
//static void MX_DMA_Init(void)
//{
//  /* DMA controller clock enable */
//  __HAL_RCC_DMAMUX_CLK_ENABLE();
//  __HAL_RCC_DMA2_CLK_ENABLE();
//
//  /* DMA interrupt init */
//  /* DMA2_Stream0_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
//
//}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOZ_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
/*void StartDefaultTask(void const * argument)
 {
 init code for RESMGR_UTILITY
 MX_RESMGR_UTILITY_Init();

 init code for OPENAMP
 MX_OPENAMP_Init();

 USER CODE BEGIN 5
 Infinite loop
 for(;;)
 {
 osDelay(1);
 }
 USER CODE END 5
 }*/

void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart) {

	/* copy received msg in a variable to sent it back to master processor in main infinite loop*/
	VirtUart0ChannelRxSize =
			huart->RxXferSize < MAX_BUFFER_SIZE ?
					huart->RxXferSize : MAX_BUFFER_SIZE - 1;
	memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
	VirtUart0RxMsg = SET;
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
