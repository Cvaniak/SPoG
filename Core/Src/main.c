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
#include "crc.h"
#include "i2c.h"
#include "i2s.h"
#include "pdm2pcm.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__uint8_t UserPressButton = 0;

/* Wave Player Pause/Resume Status. Defined as external in waveplayer.c file */
__uint32_t PauseResumeStatus = IDLE_STATUS;

/* Counter for User button presses */
__uint32_t PressCount = 0;

#define AUDIO_BUFFER_SIZE   8192
#define WR_BUFFER_SIZE      0x7000

typedef struct {
  int32_t offset;
  uint32_t fptr;
}Audio_BufferTypeDef;

typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;


Audio_BufferTypeDef  BufferCtl;

uint8_t  pHeaderBuff[44];
uint16_t WrBuffer[WR_BUFFER_SIZE];

static uint16_t RecBuf[2*PCM_OUT_SIZE];
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];

__IO uint32_t AUDIODataReady = 0, AUDIOBuffOffset = 0;
__IO uint32_t ITCounter = 0;
__IO uint8_t volume = 70;
volatile sum = 0;
 uint32_t AudioTotalSize; /* This variable holds the total size of the audio file */
uint32_t AudioRemSize;   /* This variable holds the remaining data in audio file */
uint16_t *CurrentPos;   /* This variable holds the current position of audio pointer */
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
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART1_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);
  BufferCtl.offset = BUFFER_OFFSET_NONE;
  BufferCtl.fptr = 0;

  Led_Flash(0);
  printf("\n\nAUDIO LOOPBACK EXAMPLE START:\n");
  if(BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) != AUDIO_OK)
  {
	  Led_Flash(2);
  }

  printf("AUDIO loop from digital micro (U20 & U21 components on board) to headphone (CN10 jack connector)\n");


  Led_Flash(0);

  /* Start Recording */
  if (BSP_AUDIO_IN_Record((uint16_t*)&InternalBuffer[0], 64) != AUDIO_OK)
  {
    /* Record Error */
	  Led_Flash(2);
  }
  BSP_AUDIO_IN_SetVolume(75);


  AUDIODataReady = 0;

  while (AUDIODataReady != 2)
  {
    if(BufferCtl.offset == BUFFER_OFFSET_HALF)
    {
      /* PDM to PCM data convert */
//    	Led_Flash(3);
//    	Led_Flash(3);
      BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[0], (uint16_t*)&RecBuf[0]);

      /* Copy PCM data in internal buffer */
      memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);

      BufferCtl.offset = BUFFER_OFFSET_NONE;

//      if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*4))-1)
//      {
//        AUDIODataReady = 1;
//        AUDIOBuffOffset = 0;
//        ITCounter++;
//      }
//      else if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*2))-1)
//      {
//        AUDIODataReady = 2;
//        AUDIOBuffOffset = WR_BUFFER_SIZE/2;
//        ITCounter = 0;
//      }
//      else
//      {
        ITCounter++;
//      }

    }

    if(BufferCtl.offset == BUFFER_OFFSET_FULL)
    {
      /* PDM to PCM data convert */
//    	Led_Flash(3);
//    	Led_Flash(3);
      BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[0], (uint16_t*)&RecBuf[0]);

      /* Copy PCM data in internal buffer */
      memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);

      BufferCtl.offset = BUFFER_OFFSET_NONE;
      if (BSP_AUDIO_IN_Record((uint16_t*)&InternalBuffer[0], 128) != AUDIO_OK)
      {
        /* Record Error */
    	  Led_Flash(2);
      }
      else{
//    	  Led_Flash(3);

      }

//      if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*4))-1)
//      {
//        AUDIODataReady = 1;
//        AUDIOBuffOffset = 0;
//        ITCounter++;
//      }
//      else if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*2))-1)
//      {
//        AUDIODataReady = 2;
//        AUDIOBuffOffset = WR_BUFFER_SIZE/2;
//        ITCounter = 0;
//      }
//      else
//      {
        ITCounter = 0;
//        AUDIODataReady = 2;
//      }

    }
  };


  if (BSP_AUDIO_IN_Stop() != AUDIO_OK)
  {
    /* Record Error */
	  BSP_LED_On(LED5);

  }
  else{
	  Led_Flash(2);
	  Led_Flash(2);
	  Led_Flash(2);
  }


  Led_Flash(1);
//  my_wait(20);
//  if(BSP_ACCELERO_Init() != ACCELERO_OK)
//  {
//    /* Initialization Error */
//    Error_Handler();
//  }
//
//  /* MEMS Accelerometer configure to manage PAUSE, RESUME operations */
//  BSP_ACCELERO_Click_ITConfig();

  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_BOTH, 70, 48000) != AUDIO_OK)
  {
	  BSP_LED_On(LED5);
	  my_wait(100);
  }

//
//  AudioTotalSize = INTERNAL_BUFF_SIZE;
//  /* Set the current audio pointer position */
//  CurrentPos = (uint16_t *)(&RecBuf[0]);
////
//  BSP_AUDIO_OUT_Play(CurrentPos, AudioTotalSize);

  /* Start Playback */

//  while (1) {
//      /* Wait end of half block recording */
//      while (audio_rec_buffer_state == BUFFER_OFFSET_HALF) {
//      }
//      audio_rec_buffer_state = BUFFER_OFFSET_NONE;
//
//      /* Copy recorded 1st half block */
//      memcpy((uint16_t *)(AUDIO_BUFFER_OUT), (uint16_t *)(AUDIO_BUFFER_IN), AUDIO_BLOCK_SIZE);
//
//      /* Wait end of one block recording */
//      while (audio_rec_buffer_state == BUFFER_OFFSET_FULL) {
//      }
//      audio_rec_buffer_state = BUFFER_OFFSET_NONE;
//
//      /* Copy recorded 2nd half block */
//      memcpy((uint16_t *)(AUDIO_BUFFER_OUT + (AUDIO_BLOCK_SIZE)), (uint16_t *)(AUDIO_BUFFER_IN + (AUDIO_BLOCK_SIZE)), AUDIO_BLOCK_SIZE);
//  }

//    AudioPlay_Test();
  while (1)
  {
//	  PressCount++;
	  Toggle_Leds();
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
//	volatile HAL_StatusTypeDef result = HAL_I2S_Receive(&hi2s2, data_in, 2, 100);
//	if (result == HAL_OK) {
//		volatile int32_t data_full = (int32_t) data_in[0] << 16 | data_in[1];
//		volatile int16_t data_short = (int16_t) data_in[0];
//		volatile uint32_t counter = 10;
////		while(counter -- );
//	    if(data_full>=200000){
//	    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
//	    }
//	    else{
//	    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
//
//	    }
//	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 4;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void my_wait(int k){
	for(int j=0;j<k;j++)
	  for(int i=0;i<80000;){i++;}

}
void Toggle_Leds(void)
{
  BSP_LED_Toggle(LED3);
  my_wait(1);
  BSP_LED_Toggle(LED4);
  my_wait(1);
  BSP_LED_Toggle(LED5);
  my_wait(1);
  BSP_LED_Toggle(LED6);
  my_wait(1);
}

void Led_Flash(int i)
{
	if(i == 0){
		  BSP_LED_Toggle(LED3);
		  my_wait(3);
		  BSP_LED_Toggle(LED3);
	}
	if(i == 1){
		  BSP_LED_Toggle(LED4);
		  my_wait(3);
		  BSP_LED_Toggle(LED4);

	}
	if(i == 2){
		  BSP_LED_Toggle(LED5);
		  my_wait(3);
		  BSP_LED_Toggle(LED5);

	}
	if(i == 3){
		  BSP_LED_Toggle(LED6);
		  my_wait(3);
		  BSP_LED_Toggle(LED6);

	}
	  my_wait(3);
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  BufferCtl.offset = BUFFER_OFFSET_FULL;
 BSP_LED_Toggle(LED3);
 sum = 0;
 for(int i = 0; i<64;i++){
	 sum += WrBuffer[i]/64.0;
 }
//	Led_Flash(1);
//	Led_Flash(0);
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  BufferCtl.offset = BUFFER_OFFSET_HALF;
 BSP_LED_Toggle(LED6);
//	Led_Flash(1);
//	Led_Flash(0);
//	Led_Flash(0);
//	Led_Flash(0);
}

/**
  * @brief  Audio IN Error callback function
  * @param  pData
  * @retval None
  */
void BSP_AUDIO_IN_Error_Callback(void)
{
  /* Stop the program with an infinite loop */
	Led_Flash(1);
	Led_Flash(2);
	Led_Flash(1);
}


void BSP_AUDIO_OUT_TransferComplete_CallBack()
{
  uint32_t replay = 0;

	Led_Flash(3);
	Led_Flash(3);
	Led_Flash(3);
	Led_Flash(3);
	Led_Flash(3);
//  if (AudioRemSize > 0)
//  {
//    /* Replay from the current position */
//    BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)CurrentPos, DMA_MAX(AudioRemSize/AUDIODATA_SIZE));
//
//    /* Update the current pointer position */
//    CurrentPos += DMA_MAX(AudioRemSize);
//
//    /* Update the remaining number of data to be played */
//    AudioRemSize -= AUDIODATA_SIZE * DMA_MAX(AudioRemSize/AUDIODATA_SIZE);
//  }
//  else
//  {
//    /* Request to replay audio file from beginning */
//    replay = 1;
//  }
//
//  /* Audio sample used for play */
//  if((AudioTest == 0) && (replay == 1))
//  {
//    /* Replay from the beginning */
//    /* Set the current audio pointer position */
//    CurrentPos = (uint16_t *)(AUDIO_FILE_ADDRESS);
//    /* Replay from the beginning */
//    BSP_AUDIO_OUT_Play(CurrentPos, AudioTotalSize);
//    /* Update the remaining number of data to be played */
//    AudioRemSize = AudioTotalSize - AUDIODATA_SIZE * DMA_MAX(AudioTotalSize);
//    /* Update the current audio pointer position */
//    CurrentPos += DMA_MAX(AudioTotalSize);
//  }
//
//  /* Audio sample saved during record */
//  if((AudioTest == 1) && (replay == 1))
//  {
//    /* Replay from the beginning */
//    BSP_AUDIO_OUT_Play(WrBuffer, AudioTotalSize);
//  }
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
	  BSP_LED_On(LED5);
	  for(int i=0;i<80000;){i++;}
	  for(int i=0;i<80000;){i++;}
	  for(int i=0;i<80000;){i++;}
	  for(int i=0;i<80000;){i++;}
	  for(int i=0;i<80000;){i++;}
	  for(int i=0;i<80000;){i++;}
//	  while(1)
//	  {
//	  }
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
