#ifndef _user_led_h
#define _user_led_h

#include "user_includes.h"


#define _LED_TASK_TIME 20 //20ms

#define LED_R 0
#define LED_G 1
#define LED_P 2


#define LED_R_ON 		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)
#define LED_R_OFF		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)

#define LED_G_ON 		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET)
#define LED_G_OFF		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET)

#define LED_B_ON 		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET)
#define LED_B_OFF		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET)

#define LED_A_L 		HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_RESET)
#define LED_A_H			HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET)

typedef struct
{
	uint8_t flash_freq;
	uint8_t flash_time;
	uint8_t flash_en;
	
	uint16_t task_cnt;
	uint8_t remain_time;
	uint8_t on_cnt;

}LED_TYPEDEF;
extern LED_TYPEDEF LED[3];

typedef struct
{
	uint32_t grb;	
	uint8_t send_busy;
}LEDA_TYPEDEF;
extern LEDA_TYPEDEF LEDA;

void User_LedInit(void);
void User_LedTaskShow(void);
void User_LedShowSet(uint8_t rgb,uint8_t freq,uint8_t time,uint8_t en);
uint8_t User_LedaSetColor(uint8_t r,uint8_t g,uint8_t b);
void User_LedaTurnColor(void);
#endif










