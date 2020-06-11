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
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Oth/stm32f411e_discovery.h"
#include "../Oth/stm32f411e_discovery_audio.h"
#include "complex.h"
#include "math.h"
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
#define MAX 256

volatile int32_t data_full;
volatile int16_t data_short;
volatile uint32_t counter;
uint16_t data_in[128];
int ends;
uint16_t data_pcm[256];
uint16_t a;
uint16_t b;
uint16_t c;
uint16_t d;
uint16_t e;


#define PDM_BUFFER_SIZE 1
#define PCM_BUFFER_SIZE 2500
#define PDM_BLOCK_SIZE_BITS 16
#define LEAKY_KEEP_RATE 0.95

volatile HAL_StatusTypeDef result;
uint8_t toPDM;
uint16_t PDM_buffer[PDM_BUFFER_SIZE]; // Buffer for pdm value from hi2s2 (Mic)
uint16_t PDM_value = 0;
uint8_t PCM_value = 0;    // For keeping pcm value calculated from pdm_value
float leaky_PCM_buffer = 0.0;    // Fast Estimation of moving average of PDM
float leaky_AMP_buffer = 0.0; // Fast Estimation of moving average of abs(PCM)




int n = 256;
double _Complex vec[MAX];
double freq = 100;
double dt = 0.001;
double vec1[60];
double vec2[60];
double vec3[60];
double vec4[MAX];


//#define M_PI 3.1415926535897932384
//
int log22(int N)    /*function to calculate the log2(.) of int numbers*/
{
  int k = N, i = 0;
  while(k) {
    k >>= 1;
    i++;
  }
  return i - 1;
}

int check(int n)    //checking if the number of element is a power of 2
{
  return n > 0 && (n & (n - 1)) == 0;
}

int reverse(int N, int n)    //calculating revers number
{
  int j, p = 0;
  for(j = 1; j <= (int)log22(N); j++) {
    if(n & (1 << ((int)log22(N) - j)))
      p |= 1 << (j - 1);
  }
  return p;
}

void ordina(double _Complex* f1, int N) //using the reverse order in the array
{
	double _Complex f2[MAX];
  for(int i = 0; i < N; i++)
    f2[i] = f1[reverse(N, i)];
  for(int j = 0; j < N; j++)
    f1[j] = f2[j];
}

double _Complex polar1(double rho, double theta){
	return rho * cos(theta) + rho * sin(theta)*I;
}

void transform(double _Complex* f, int N) //
{
  ordina(f, N);    //first: reverse order
  double _Complex *W;
  W = (double _Complex *)malloc(N / 2 * sizeof(double _Complex));
  W[1] = polar1(1., -2. * M_PI / N);
  W[0] = 1;
  for(int i = 2; i < N / 2; i++)
    W[i] = cpow(W[1], i);
  int n = 1;
  int a = N / 2;
  for(int j = 0; j < log22(N); j++) {
    for(int i = 0; i < N; i++) {
      if(!(i & n)) {
    	  double _Complex temp = f[i];
    	  double _Complex Temp = W[(i * a) % (n * a)] * f[i + n];
        f[i] = temp + Temp;
        f[i + n] = temp - Temp;
      }
    }
    n *= 2;
    a = a / 2;
  }
  free(W);
}

void FFT(double _Complex* f, int N, double d)
{
  transform(f, N);
  for(int i = 0; i < N; i++)
    f[i] *= 0.001; //multiplying by step
}
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);
  HAL_Delay(2000);
  BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);

  BSP_AUDIO_IN_Record(data_in, INTERNAL_BUFF_SIZE/2);
  BSP_AUDIO_IN_SetVolume(64);
  toPDM = 0;
  ends = 0;
//  for(int i = 0; i < n; i++) {
//	    vec[i] = 30*sin(2*M_PI*freq*i*dt);
//  }
//  FFT(vec,n,d);
//  for(int j = 0; j < 512; j++)
//    vec4[j] = cabs(vec[j]);
//  for(int j = 58; j < 116; j++)
//    vec2[j%58] = cabs(vec[j]);
//  for(int j = 116; j < 174; j++)
//    vec3[j%58] = cabs(vec[j]);
//  for(int j = 174; j < 512; j++)
//    vec4[j%58] = cabs(vec[j]);
//	while (1){}

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
	  	if(toPDM){
	  		toPDM = 0;

	  		BSP_AUDIO_IN_PDMToPCM(data_in, &data_pcm[(a%8)*32]);
			a++;
			if(a%8 == 0){
//				BSP_AUDIO_IN_Pause();
				for (int i = 0; i<256; i++){
					vec[i] = data_pcm[i];
				}
				FFT(vec, n, 1);
				b = 0;
				c = 0;
				for (int j = 0; j<128; j++){
					vec4[j] = cabs(vec[j]);
					if(b < vec4[j]){
						b = vec4[j];
						c = j;
					}

				}
				if(b > 500){
					d = b;
					e = 100;
				}
				else{
					d = b;
					e = 0;
				}
//				BSP_AUDIO_IN_Resume();
			}
	  	}
//	  	result = HAL_I2S_Receive(&hi2s2, data_in, 2, 100);
//	  	if (result == HAL_OK) {
//	  //		int32_t data_full = (int32_t) data_in[0] << 16 | data_in[1];
//	  //		data_short = (int16_t) data_in[0];
//	  //		counter = 10;
//	  ////		while(counter -- );
//	  //		  uint16_t AppPDM[INTERNAL_BUFF_SIZE/2];
//	  //		  uint32_t index = 0;
//
//	  		  /* PDM Demux */
//	  		BSP_AUDIO_IN_PDMToPCM(data_in, data_pcm);
//
//	  		if(data_full>=200000){
//	  	    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
//	  	    }
//	  	    else{
//	  	    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
//
//	  	    }
//	  	}


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

  /** Macro to configure the PLL multiplication factor 
  */
  __HAL_RCC_PLL_PLLM_CONFIG(8);
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BSP_AUDIO_IN_HalfTransfer_CallBack(){
	BSP_LED_Toggle(LED3);


}
void BSP_AUDIO_IN_TransferComplete_CallBack(){
	toPDM = 1;
	ends++;
	if(ends%100==0)
		BSP_LED_Toggle(LED6);
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
