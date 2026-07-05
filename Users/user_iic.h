#ifndef _user_iic_h
#define _user_iic_h
#include "main.h" 

//////////////////////////////////////////////////////////////////////////////////	 
							  
////////////////////////////////////////////////////////////////////////////////// 	

#define _IIC_SCL_H	HAL_GPIO_WritePin(MPU_SCL_GPIO_Port, MPU_SCL_Pin, GPIO_PIN_SET)
#define _IIC_SCL_L	HAL_GPIO_WritePin(MPU_SCL_GPIO_Port, MPU_SCL_Pin, GPIO_PIN_RESET)
#define _IIC_SDA_H	HAL_GPIO_WritePin(MPU_SDA_GPIO_Port, MPU_SDA_Pin, GPIO_PIN_SET)
#define _IIC_SDA_L	HAL_GPIO_WritePin(MPU_SDA_GPIO_Port, MPU_SDA_Pin, GPIO_PIN_RESET)
#define _IIC_SDA_R	HAL_GPIO_ReadPin(MPU_SDA_GPIO_Port, MPU_SDA_Pin)

//IIC所有操作函数
void User_IIC_Init(void);                //初始化IIC的IO口				 
void User_IIC_Start(void);				//发送IIC开始信号
void User_IIC_Stop(void);	  			//发送IIC停止信号
void User_IIC_SendByte(uint8_t txd);			//IIC发送一个字节
uint8_t User_IIC_ReadByte(unsigned char ack);//IIC读取一个字节
uint8_t User_IIC_WaitAck(void); 				//IIC等待ACK信号
void User_IIC_Ack(void);					//IIC发送ACK信号
void User_IIC_NAck(void);				//IIC不发送ACK信号

void User_IIC_DelayMs(uint16_t ms);
void Timer_IIC_DelayUs(uint16_t us);

#endif
















