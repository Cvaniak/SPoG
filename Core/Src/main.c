/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "../Oth/stm32f411e_discovery_audio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float absFloat(float in) {
	return in < 0 ? -in : in;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void PDMDecoder_Init();


volatile int32_t data_full;
volatile int16_t data_short;
volatile uint32_t counter;
uint16_t data_in[2];
uint16_t data_pcm[2];
PDM_Filter_Handler_t  PDM_FilterHandler[2];
PDM_Filter_Config_t   PDM_FilterConfig[2];



#define PDM_BUFFER_SIZE 1
#define PCM_BUFFER_SIZE 2500
#define PDM_BLOCK_SIZE_BITS 16
#define LEAKY_KEEP_RATE 0.95

volatile HAL_StatusTypeDef result;
uint8_t i;
uint16_t PDM_buffer[PDM_BUFFER_SIZE]; // Buffer for pdm value from hi2s2 (Mic)
uint16_t PDM_value = 0;
uint8_t PCM_value = 0;    // For keeping pcm value calculated from pdm_value
float leaky_PCM_buffer = 0.0;    // Fast Estimation of moving average of PDM
float leaky_AMP_buffer = 0.0; // Fast Estimation of moving average of abs(PCM)
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2S2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	BSP_AUDIO_IN_Init(44000, 16, 1);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	result = HAL_I2S_Receive(&hi2s2, PDM_buffer, PDM_BUFFER_SIZE, 1000);
//	if (result == HAL_OK) {
////		int32_t data_full = (int32_t) data_in[0] << 16 | data_in[1];
////		data_short = (int16_t) data_in[0];
////		counter = 10;
//////		while(counter -- );
////		  uint16_t AppPDM[INTERNAL_BUFF_SIZE/2];
////		  uint32_t index = 0;
//
//		  /* PDM Demux */
//		for (i = 0; i < PDM_BUFFER_SIZE; i++) {
//			PCM_value = -PDM_BLOCK_SIZE_BITS / 2;
//			PDM_value = PDM_buffer[i];
//			// calculate PCM value
//			while (PDM_value != 0)    // while pdm_value still have 1s in binary
//			{
//				PCM_value++;
//				PDM_value ^= PDM_value & -PDM_value; // remove left most 1 in binary
//			}
//			leaky_PCM_buffer += PCM_value;
//			leaky_PCM_buffer *= LEAKY_KEEP_RATE;
//			leaky_AMP_buffer += absFloat(leaky_PCM_buffer);
//			leaky_AMP_buffer *= LEAKY_KEEP_RATE;
//		}
//
//		if(data_full>=200000){
//	    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
//	    }
//	    else{
//	    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
//
//	    }
//	}
	  	result = HAL_I2S_Receive(&hi2s2, data_in, 2, 100);
	  	if (result == HAL_OK) {
	  //		int32_t data_full = (int32_t) data_in[0] << 16 | data_in[1];
	  //		data_short = (int16_t) data_in[0];
	  //		counter = 10;
	  ////		while(counter -- );
	  //		  uint16_t AppPDM[INTERNAL_BUFF_SIZE/2];
	  //		  uint32_t index = 0;

	  		  /* PDM Demux */
	  		BSP_AUDIO_IN_PDMToPCM(data_in, data_pcm);

	  		if(data_full>=200000){
	  	    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
	  	    }
	  	    else{
	  	    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);

	  	    }
	  	}


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void PDMDecoder_Init()
{
  uint32_t index = 0;

  /* Enable CRC peripheral to unlock the PDM library */
  __HAL_RCC_CRC_CLK_ENABLE();

  for(index = 0; index < 1; index++)
  {
    /* Init PDM filters */
    PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
    PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
    PDM_FilterHandler[index].high_pass_tap = 2122358088;
    PDM_FilterHandler[index].out_ptr_channels = 1;
    PDM_FilterHandler[index].in_ptr_channels  = 1;
    PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));

    /* PDM lib config phase */
    PDM_FilterConfig[index].output_samples_number = 440000/1000;
    PDM_FilterConfig[index].mic_gain = 24;
    PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
    PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
