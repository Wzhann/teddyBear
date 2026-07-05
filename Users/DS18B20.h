/**
 ****************************************************************************************************
 * @file        ds18b20.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-27
 * @brief       DS18B20数字温度传感器 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211027
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __DS18B20_H
#define __DS18B20_H
#include "main.h"


/******************************************************************************************/
/* DS18B20引脚 定义 */

#define DS18B20_DQ_GPIO_PORT                GPIOE
#define DS18B20_DQ_GPIO_PIN                 GPIO_PIN_0
#define DS18B20_DQ_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* 时钟使能 */

/******************************************************************************************/

/* IO操作函数 */

#define DS18B20_DQ_OUT_0 HAL_GPIO_WritePin(DS18B20_DQ_GPIO_PORT, DS18B20_DQ_GPIO_PIN, GPIO_PIN_RESET)	/* 数据端口输出 */
#define DS18B20_DQ_OUT_1 HAL_GPIO_WritePin(DS18B20_DQ_GPIO_PORT, DS18B20_DQ_GPIO_PIN, GPIO_PIN_SET)	/* 数据端口输出 */														
#define DS18B20_DQ_IN    HAL_GPIO_ReadPin(DS18B20_DQ_GPIO_PORT, DS18B20_DQ_GPIO_PIN)     /* 数据端口输入 */

#define _DS18_NUM 2
														
typedef struct
{
	uint8_t id[_DS18_NUM][8];	
	float temper[_DS18_NUM];
	uint8_t online_num;
}USER_DS18B20_TYPE;

extern USER_DS18B20_TYPE DS18B20; 														
														
														
uint8_t ds18b20_init(void);         /*初始化DS18B20*/

void ds18b20_get_temperature_single(void);	/*总线上连接单个传感器时，读取该传感器温度*/
void ds18b20_search_ID(void);								/*总线上连接多个传感器时，搜索总线上的传感器ID*/
void ds18b20_get_temperature_multiple(void);/*总线上连接多个传感器时，分次读取各传感器温度*/

void TimerDelayUs(uint16_t us);														
void User_DelayMs(uint16_t ms);
														
#endif




