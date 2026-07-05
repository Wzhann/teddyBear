/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_servo.h"
#include "Action_library.h"
#include "user_imu_i2c.h"
#include "user_communication.h"
#include "user_adc.h"
#include <time.h>
#include "stdlib.h"
#include "user_IAP.h"
#include "user_flash.h"

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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t usart_tx_buf[13] = {0};
uint8_t POWERON = 0;
uint8_t SHUTDOWN = 0;
int cococo = 0;
uint32_t t = 10000000;
extern uint8_t poseNow;
extern buzzerType buzzerWorking;
extern uint8_t rgbTimes;
extern uint8_t buzzerTimes;
uint32_t debugFlag;
uint8_t debugUse;
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
	__enable_irq();
	__set_FAULTMASK(0);
	SCB->VTOR = FLASH_BASE | 0x100000;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_UART7_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
//  __use_no_semihosting();
//HAL_Delay(10);

//HAL_GPIO_WritePin(upperComputerPower_5V_GPIO_Port,upperComputerPower_5V_Pin,GPIO_PIN_RESET);//
//  HAL_GPIO_WritePin(Servo_Power_12V_GPIO_Port,Servo_Power_12V_Pin,GPIO_PIN_RESET);//舵机供电
//  
  HAL_Delay(200);
	HAL_GPIO_WritePin(upperComputerPower_5V_GPIO_Port,upperComputerPower_5V_Pin,GPIO_PIN_SET);//
  HAL_GPIO_WritePin(Servo_Power_12V_GPIO_Port,Servo_Power_12V_Pin,GPIO_PIN_SET);//舵机供电
//  HAL_TIM_Base_Start(&htim17);

//	debugFlag = *(__IO uint32_t*)(0x080aff00);
//	debugUse = *(__IO uint8_t*)(0x080aff00);
//	while(debugUse < 1);
//	debugFlag--;
//	FLASH_Write(0x080Aff00,&debugFlag,8);

//  printf("APPAPP1!\r\n");
	HAL_TIM_Base_Start(&htim13);
	srand(__HAL_TIM_GET_COUNTER(&htim13));//随机数种子设置
	
	HAL_Delay(3000);
	
	
//  

//  User_CommunicationInit(); // 通信协议初始化
  
//  if (HAL_GPIO_ReadPin(Power_in_GPIO_Port, Power_in_Pin) == GPIO_PIN_RESET)
//  {
//	  HAL_GPIO_WritePin(Power_out_GPIO_Port,Power_out_Pin,GPIO_PIN_SET);//板子供电
//	  POWERON = 1;
//  }
  //while(HAL_GPIO_ReadPin(Power_in_GPIO_Port, Power_in_Pin) != GPIO_PIN_SET);
  
//  srand((unsigned)time(NULL));
//  HAL_Delay(3000);//等待舵机上电
  //User_ServoInit();
  cococo = 1;
//	printf("STARTAPP1!\r\n");
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
	//FEETECH_ReadServoPos(cococo);
//	 Action_Teachmode();
//	 if (HAL_GPIO_ReadPin(Power_in_GPIO_Port, Power_in_Pin) == GPIO_PIN_RESET)
//	{
//		cococo++;
//		if(cococo > 5)
//			HAL_GPIO_WritePin(Power_out_GPIO_Port,Power_out_Pin,GPIO_PIN_RESET);//���ӹ���
//	}
//	else if(cococo>0)
//		cococo--;
//	  ledSet(ioState.led);
//	  HAL_Delay(1);
//	  buzzerSet(ioState.buzzer);HAL_Delay(1);
//	  fanSet(ioState.fan);
	  HAL_UART_Transmit(&huart7, (uint8_t*)"HELLO: this is APP2!\r\n", 23, 100);
	HAL_Delay(1000);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t countTimerForMPU6050;
uint16_t countTimerForUser_ADC;
uint16_t countForSendSum;
uint16_t test_Flag_User;

extern uint8_t countForCharging;
extern uint8_t buzzerForcharging;
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if(htim->Instance == TIM3)
  {
	  static uint8_t countForTimerTouch = 0;
	countForTimerTouch++;
	if(countForTimerTouch == 100)
	{
		 Key_Downside_Record();
		/*触摸人体检测传感器数据上报*/
		if(touchTopofHead_Downside  || touchChin_Downside || humanDetectionAbdomen_Downside
			||touchTopofHead_Upside || touchChin_Upside   || humanDetectionAbdomen_Upside
			||touchBody_Upside 		|| touchBody_Downside || humanDetectionBackside_Downside 
			|| humanDetectionBackside_Upside)
		{
			sendSensorActive(&ph,!touchTopofHead, !touchBody, !touchChin, humanDetectionAbdomen, humanDetectionBackside);
		}
		
		countForTimerTouch = 0;
	}
	
	/***********做动作控制动作时间的计时器**********/
		User_TimerActionIRQ();
				
		countTimerForUser_ADC++;
		if(countTimerForUser_ADC > 20)
		{
		/*电流和电压检测（综合检测：空闲时才更新电压/电量）*/
		User_AdcBatteryStatusUpdate();
			countTimerForUser_ADC = 0;
		}
			
		/*充电*/
		if(USER_ADC.bat_current > 2.6f) 
		{
			countForCharging++;
		}
		else countForCharging = 0;
		if(countForCharging >= 10)
		{
			countForCharging = 10;
			USER_ADC.bat_charging = 1;
		}
		else
		{
			USER_ADC.bat_charging = 0;
			buzzerForcharging = 0;
		}
		
		/*充电检测上报*/
	static uint8_t countForTimerIOCharging = 0;
	countForTimerIOCharging++;
	if(countForTimerIOCharging == 100)
	{
		IOForChargingDownside();
		
		if(IOForCharging_Upside || IOForCharging_Downside)
		{
			test_Flag_User++;
			sendStateActive(&ph, stateRobot);
		}
		countForTimerIOCharging = 0;
	}
	
		/*低电压*/
		static uint16_t countForbuzzerColt = 0;
		static uint8_t startBuzzerForVolt = 0;
		countForbuzzerColt ++;
		if(countForbuzzerColt >= 100) 
		{
			startBuzzerForVolt = 1;
			countForbuzzerColt = 100;
		}
		if(USER_ADC.bat_power < 8 && startBuzzerForVolt == 1)
		{
			buzzerWorking.buzzerForvoltage = 1;
			rgbTimes = 3;
			buzzerTimes = 2;
		}
		else 
		{
			buzzerWorking.buzzerForvoltage = 0;
			rgbTimes = 0;
		}
		
  }

  /* USER CODE END Callback 1 */
}

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
