#ifndef _user_includes_h
#define _user_includes_h

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"


#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define u64 uint64_t
typedef union
{
	float data;
	uint8_t buf[4];
}UNION_FLOAT;
typedef union
{
	int64_t data;
	uint8_t buf[8];
}UNION_INT64;
typedef union
{
	int32_t data;
	uint8_t buf[4];
}UNION_INT32;
typedef union
{
	int16_t data;
	uint8_t buf[2];
}UNION_INT16;

//extern ADC_HandleTypeDef hadc1;
//extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim17;

#include "main.h"

#include "user_tasks.h"
#include "user_led.h"
#include "user_adc.h"
#include "user_timer.h"
#include "user_imu.h"
#include "user_comm.h"
#include "user_servo.h"
#include "user_gait.h"
#include "Action_Library.h"
#endif




