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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	uint16_t adc1,adc2,adc3;
	char msg[10];
	


/*----------------LEFT------------------------------*/
void servo_write2(int angle)
{
			htim2.Instance->CCR3	= map(0,180,25,125,angle*3);					
}
/*----------------RIGHT------------------------------*/
void servo_write1(int angle)
{
	htim2.Instance->CCR2 = map(0,180,25,125,angle*2);
}	
/*----------------BASE------------------------------*/
void servo_write(int angle)
{
	htim2.Instance->CCR1 = map(0,180,25,125,angle*3);

}

void servo_sweep(adc1,adc2,adc3)
{
				servo_write(adc1);
		   	HAL_Delay(10);
					
				servo_write1(adc2);
		   	HAL_Delay(10); 
	
				servo_write2(adc3);
		   	HAL_Delay(10); 
	

}
int map(int st1, int fn1, int st2, int fn2, int value)
{
    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
}

void manualMode(){
		
		 // Get ADC value
    HAL_ADC_Start(&hadc1);
 
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adc1 = HAL_ADC_GetValue(&hadc1);
		
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adc3 = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adc2 = HAL_ADC_GetValue(&hadc1);
		
		adc1 = adc1*0.045;
		
		adc2 = adc2*0.045;
		
		adc3 = adc3*0.045;
		

		sprintf(msg, "%hu\r\n", adc1);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "%hu\r\n", adc2);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			
    sprintf(msg, "%hu\r\n", adc3);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		
		
		
		HAL_ADC_Stop(&hadc1);	
		servo_sweep(adc1,adc2,adc3); //Use servo 
}

void autoMode(){
		
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,0);
		
		
		htim2.Instance->CCR1 = 400;
		HAL_Delay(2000);
		
		htim3.Instance->CCR4 = 300;
		HAL_Delay(2000);
		
		htim3.Instance->CCR4 = 200;
		HAL_Delay(2000);
		
		htim2.Instance->CCR1 = 60;
		HAL_Delay(2000);
		
		htim3.Instance->CCR4 = 300;
		HAL_Delay(2000);
		
		htim3.Instance->CCR4 = 400;
		HAL_Delay(500);
		
}

void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==0){
		autoMode();
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
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
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	int check(int);
	void servo_write(int);
	void servo_write1(int);
	void servo_write2(int);
	void servo_sweep(int,int,int);
	int map(int, int, int, int, int);
	void manualMode(void);
	void autoMode(void);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		/*----------------HEAD------------------------------if open ==300   close==200*/
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0){
			
			htim3.Instance->CCR4 = 300;
			
		}else{
			
			htim3.Instance->CCR4 = 200;			
		}
		
		/*------------------------------LED-----------------------------------*/
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,0);
		/*--------------------------------ADC---------------------------------*/
		HAL_Delay(10);
		manualMode();
		
		// Convert to string and print
    // Pretend we have to do something else for a while
    HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
