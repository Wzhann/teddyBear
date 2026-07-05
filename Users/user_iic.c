/**
IIC驱动程序（IO模拟IIC）
用于OLED操作
**/

#include "user_iic.h"
#include "tim.h"
/**
定义IIC通信中延时时间
长延时——2us
短延时——1us
若IIC通信不稳定，可尝试增加该延时
**/
#define CLK_T_LONG 	4
#define CLK_T_SHORT 2


#define _USER_HTIM_IIC		htim8
#define _USER_TIM_IIC			TIM8

/* TIM8 init function */
void Timer_IIC_Delay_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */
	TIM_HandleTypeDef htim8;
  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim8.Instance = TIM7;
  htim8.Init.Prescaler = 240-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}
/**
us级延时，该延时函数将阻塞程序执行，非必要不使用
用于IO口模拟IIC通信使用（stm32f1硬件IIC尚存在问题）
**/
void Timer_IIC_DelayUs(uint16_t us) 
{
	_USER_TIM_IIC->CNT = 0;
	HAL_TIM_Base_Start(&_USER_HTIM_IIC); 	
	while(_USER_TIM_IIC->CNT < us);
	HAL_TIM_Base_Stop(&_USER_HTIM_IIC); 
}
void User_IIC_DelayMs(uint16_t ms)
{
	while(ms--)
		Timer_IIC_DelayUs(1000);
}

//初始化IIC
void User_IIC_Init(void)
{			
	_IIC_SCL_H;
	_IIC_SDA_H;
	Timer_IIC_Delay_Init();
}

void User_SetSdaIn(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = MPU_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MPU_SDA_GPIO_Port, &GPIO_InitStruct);	
}
void User_SetSdaOut(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = MPU_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MPU_SDA_GPIO_Port, &GPIO_InitStruct);
}

//产生IIC起始信号
void User_IIC_Start(void) 
{
	User_SetSdaOut();     //sda线输出
	_IIC_SDA_H;	  	  
	_IIC_SCL_H;
	Timer_IIC_DelayUs(CLK_T_LONG);
 	_IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	Timer_IIC_DelayUs(CLK_T_LONG);
	_IIC_SCL_L;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void User_IIC_Stop(void)
{
	User_SetSdaOut();//sda线输出
	_IIC_SCL_L;
	_IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	Timer_IIC_DelayUs(CLK_T_LONG);
	_IIC_SCL_H; 
	_IIC_SDA_H;//发送I2C总线结束信号
	Timer_IIC_DelayUs(CLK_T_LONG);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t User_IIC_WaitAck(void)
{
	uint8_t cnt_time=0;
	User_SetSdaIn();      //SDA设置为输入  
	_IIC_SDA_H;
	Timer_IIC_DelayUs(CLK_T_SHORT);	   
	_IIC_SCL_H;
	Timer_IIC_DelayUs(CLK_T_SHORT);	 
	while(_IIC_SDA_R)
	{
		cnt_time++;
		if(cnt_time>250)
		{
			User_IIC_Stop();
			return 1;
		}
	}
	_IIC_SCL_L;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void User_IIC_Ack(void)
{
	_IIC_SCL_L;
	User_SetSdaOut();
	_IIC_SDA_L;
	Timer_IIC_DelayUs(CLK_T_SHORT);
	_IIC_SCL_H;
	Timer_IIC_DelayUs(CLK_T_SHORT);
	_IIC_SCL_L;
}
//不产生ACK应答		    
void User_IIC_NAck(void)
{
	_IIC_SCL_L;
	User_SetSdaOut();
	_IIC_SDA_H;
	Timer_IIC_DelayUs(CLK_T_SHORT);
	_IIC_SCL_H;
	Timer_IIC_DelayUs(CLK_T_SHORT);
	_IIC_SCL_L;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void User_IIC_SendByte(uint8_t txd)
{                        
  uint8_t t;   
	User_SetSdaOut(); 	    
	_IIC_SCL_L;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		if((txd & 0x80) >> 7)
			_IIC_SDA_H;
		else
			_IIC_SDA_L;
		txd <<= 1; 	  
		Timer_IIC_DelayUs(CLK_T_SHORT);   //
		_IIC_SCL_H;
		Timer_IIC_DelayUs(CLK_T_SHORT); 
		_IIC_SCL_L;	
		Timer_IIC_DelayUs(CLK_T_SHORT);
	}	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t User_IIC_ReadByte(unsigned char ack)
{
	unsigned char i,receive=0;
	User_SetSdaIn();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
    _IIC_SCL_L; 
    Timer_IIC_DelayUs(CLK_T_SHORT);
		_IIC_SCL_H;
    receive <<= 1;
    if(_IIC_SDA_R)
			receive++;   
		Timer_IIC_DelayUs(CLK_T_SHORT); 
  }					 
  if (!ack)
    User_IIC_NAck();//发送nACK
  else
    User_IIC_Ack(); //发送ACK   
  return receive;
}



























