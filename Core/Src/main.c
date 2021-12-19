/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dcmi.h"
#include "dfsdm.h"
#include "i2c.h"
#include "usart.h"
#include "quadspi.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

#include "arm_math.h"
#include "arm_const_structs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CONVERTED_DATA_BUFFER_SIZE ((uint32_t) 4096)
#define FFT_Length 4096

//#define FFT_INVERSE_FLAG        ((uint8_t)0)
//#define FFT_Normal_OUTPUT_FLAG  ((uint8_t)1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADCval[ADC_CONVERTED_DATA_BUFFER_SIZE];
char ADCstr[100];
float32_t fftInput[FFT_Length * 2];
float32_t fftOutput[FFT_Length];
float32_t fftOutput_half[FFT_Length / 2];

uint16_t count = 0;

float32_t maxValue;  /* Max FFT value is stored here */
uint32_t maxIndex;  /* Index in Output array where max value is */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void FFT_Processing(void);
/* USER CODE END PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_DCMI_Init();
  MX_DFSDM1_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_QUADSPI_Init();
  MX_SAI1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	HAL_ADC_Start(&hadc3);
////	if (HAL_ADC_Start(&hadc3) != HAL_OK)
////	{
////	  Error_Handler();
////	}
//	HAL_ADC_PollForConversion(&hadc3, 0xffff);
////	if (HAL_ADC_PollForConversion(&hadc3, 0xffff) != HAL_OK)
////	{
////	  Error_Handler();
////	}
////	if (HAL_ADC_GetState(&hadc3) != HAL_OK)
////	{
////	  Error_Handler();
////	}
////	else
////	{
//	  ADCval = HAL_ADC_GetValue(&hadc3);
////	}
////
////	if (HAL_ADC_Stop(&hadc3) != HAL_OK)
////	{
////	  Error_Handler();
////	}
//    HAL_ADC_Stop(&hadc3);

//	sprintf(ADCstr, "%.3f", ADCval * 2.5 / 4096);
//	HAL_UART_Transmit(&huart2, (uint8_t*) ADCstr, sizeof(ADCstr), 0xffff);
//	HAL_UART_Transmit(&huart2, (uint8_t*) " \n", sizeof(" \n"), 0xffff);
//	HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK
                              |RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
/* sampling frequency = 8k */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    HAL_ADC_Start_IT(&hadc3);
    count++;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *htim)
{
  HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
  HAL_ADC_Stop_IT(&hadc3);
  HAL_TIM_Base_Stop_IT(&htim2);
  ADCval[count] = HAL_ADC_GetValue(&hadc3);
  if (count == ADC_CONVERTED_DATA_BUFFER_SIZE - 1)
  {
	FFT_Processing();
	count = 0;
  }
  HAL_TIM_Base_Start_IT(&htim2);
}

void FFT_Processing(void)
{
//  arm_cfft_radix4_instance_f32  FFT_F32_struct;

  uint32_t fftSize = 4096;
  uint32_t ifftFlag = 0;
  uint32_t doBitReverse = 1;

  unsigned index_fill_input_buffer;

  for (index_fill_input_buffer = 0; index_fill_input_buffer < fftSize * 2; index_fill_input_buffer += 2)
  {
    /*The data in the complex arrays is stored in an interleaved fashion
      (real, imag, real, imag, ...).*/
    fftInput[(uint16_t)index_fill_input_buffer] = (float32_t) ADCval[index_fill_input_buffer / 2] / (float32_t) 4096.0 * 2.5;
    /* Imaginary part */
    fftInput[(uint16_t)(index_fill_input_buffer + 1)] = 0;
  }

  arm_cfft_f32(&arm_cfft_sR_f32_len4096, fftInput, ifftFlag, doBitReverse);


//  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
//  arm_cfft_radix4_init_f32(&FFT_F32_struct, fftSize, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//
//  arm_cfft_radix4_f32(&FFT_F32_struct, fftInput);

  /* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
  arm_cmplx_mag_f32(fftInput, fftOutput, fftSize);

  fftOutput[0] = 0; /* remove the DC component, otherwise the DC component may be the largest */
  for (uint16_t i = 0; i <= fftSize / 2; i++)
  {
	fftOutput_half[i] = fftOutput[i];
  }

  /* Calculates maxValue and returns corresponding value */
  arm_max_f32(fftOutput_half, fftSize / 2 + 1, &maxValue, &maxIndex);
//  arm_max_f32(fftOutput, fftSize, &maxValue, &maxIndex);

//  printf("FFT(F32) maxValue %f, maxIndex %d\n\r", maxValue, maxIndex);
//  for(uint32_t i = 0; i < fftSize; i++)
//  printf("%f\n\r", fftOutput[i]);

  char maxIndex_str[100];

  maxIndex = maxIndex * 8000 / fftSize; // one bin on the spectrum = 8k / 1024
  sprintf(maxIndex_str, "%ld", maxIndex);
  HAL_UART_Transmit(&huart2, (uint8_t*) maxIndex_str, sizeof(maxIndex_str), 0xffff);
  HAL_UART_Transmit(&huart2, (uint8_t*) " \n", sizeof(" \n"), 0xffff);

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
