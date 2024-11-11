/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lcd.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "LCD_Show.h"
#include "fft_handle.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_LENGTH 4096										//采样点个数

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t i = 0;
uint16_t j = 0;
uint16_t DrawingTime_cnt = 0;
uint16_t PWMCheck_cnt = 0;					
uint16_t adcBuff[FFT_LENGTH];							//adc采样数组
float adcVolt[FFT_LENGTH];								//adc转换后的真实电压数组
__IO uint8_t AdcConvEnd = 0;							//adc采集完成标志
uint32_t f_sa = 512000;										//基础采样频率为512K
float fft_frequency = 0;
float fft_frequency2=0;   //FFT测出最大基波频率、次大基波频率
extern float fft_outputbuf[FFT_LENGTH];  				//fft计算后数组
extern float bizhi1[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float PWM_RisingCount;
float PWM_FallingCount;
float duty;
char P_buff[30];
float PWM_frequency = 0;



void PWM_Input_Capture(void)
{
//		printf("PWM_Frequency = %.4f \r\n",1000000/PWM_RisingCount);			//数据测试
//		printf("PWM_RisingCount = %.2f %% \r\n",PWM_RisingCount);
//		printf("PWM_FallingCount = %.2f %% \r\n",PWM_FallingCount);
//	  printf("PWM_Duty = %.2f %% \r\n",duty * 100);
		PWMCheck_cnt++;
		if (PWMCheck_cnt==10)
		{
			lcd_show_string(405,160,80,16,16,"PWM_F(Hz):",YELLOW);
			sprintf(P_buff,"%0.3fHz",1000000/PWM_RisingCount);									//PWM测频率
			lcd_show_string(405,180,130,16,16,P_buff,WHITE);			
			lcd_show_string(405,120,80,16,16,"Duty:",YELLOW);	
			sprintf(P_buff,"%.2f %%",duty * 100);																//PWM测占空比
			lcd_show_string(405,140,80,16,16,P_buff,WHITE);
			PWMCheck_cnt = 0;
		}
		
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_FSMC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	lcd_display_dir(1);
	HAL_TIM_Base_Start(&htim2);											
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim3);//启动定时器3
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);							//定时器3通道1——上升沿捕获
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);							//定时器3通道2——下降沿捕获
	Set_BackGround();	        //显示背景
	Lcd_DrawNetwork();
	Draw_Prompt();				//画提示符
	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcBuff,FFT_LENGTH);//启动ADC的DMA传输

	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		     
	  while (!AdcConvEnd);                                   //等待转换完毕
		for (j = 0; j < FFT_LENGTH; j++)										//输出adc模拟采样值
		{
			adcVolt[j] = adcBuff[j] * 3.3 / 4095;							// adcBuff[j]* 3.3 / 4096是为了将ADC采集到的值转换成实际电压


			//printf("adcBuff[%d]:%.3f\n",j,adcVolt[j]*1000); 			//数据打印，查看结果
		}
		ADCvolt_ANA();
//		DrawingTime_cnt++;           //去掉波形显示
//		if (DrawingTime_cnt==10)
//		{
//			//DrawOscillogram();
//			DrawingTime_cnt = 0;
//		}
		
		fftCalculate();
		
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcBuff,FFT_LENGTH);//启动ADC的DMA传输
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)    
    //注意这里为HAL_TIM_ACTIVE_CHANNEL_1而不是TIM_CHANNEL_1
	{
		PWM_RisingCount = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
		duty =	PWM_FallingCount / PWM_RisingCount;
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		PWM_FallingCount = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);
	}
	PWM_Input_Capture();
}



//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    if(hadc==&hadc1 && i < FFT_LENGTH)
//    {
//				adcBuff[i] =HAL_ADC_GetValue(&hadc1);				//采样存入adcBuff[FFT_LENGTH]
//        i++;
//		}
//		else 
//		{
//			AdcConvEnd=1;																	//采样完成，标志置1，开始fft计算
//			i = 0;
//		}
//			
//}
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
