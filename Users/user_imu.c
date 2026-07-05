#include "user_imu.h"
#include "stdio.h"
#include "string.h"

#define USART_IMU1_HUSART huart6
#define USART_IMU1_HDMA_RX hdma_usart6_rx


USART_IMU_TYPEDEF IMU1;


void User_ImuInit(void)
{               
	__HAL_UART_ENABLE_IT(&USART_IMU1_HUSART, UART_IT_IDLE);         
	HAL_UART_Receive_DMA(&USART_IMU1_HUSART, (uint8_t*)IMU1.usart_rx_buf, USART_IMU_RX_SIZE);   
}

void User_Imu1IRQHandler(void)
{
	if(RESET != __HAL_UART_GET_FLAG(&USART_IMU1_HUSART, UART_FLAG_IDLE))   
	{
		__HAL_UART_CLEAR_IDLEFLAG(&USART_IMU1_HUSART);    		
		HAL_UART_DMAStop(&USART_IMU1_HUSART);                                                        
    IMU1.rx_data_len  = USART_IMU_RX_SIZE - __HAL_DMA_GET_COUNTER(&USART_IMU1_HDMA_RX);  
		User_Imu1DataParas();///////
		HAL_UART_Receive_DMA(&USART_IMU1_HUSART, (uint8_t*)IMU1.usart_rx_buf, USART_IMU_RX_SIZE);
	}
}

 
void User_Imu1DataParas(void)
{
	uint8_t i = 0,j = 0;
	uint8_t sum = 0;
	float temp_ang = 0;
	
	for(i=0;i<IMU1.rx_data_len-9;i++)
	{
		if((IMU1.usart_rx_buf[i] == 0x55) && (IMU1.usart_rx_buf[i+1] == 0x51))//acc data
		{
			sum = 0;
			for(j=i;j<i+10;j++)
			{
				sum += IMU1.usart_rx_buf[j];
			}
			if(sum == IMU1.usart_rx_buf[j])
			{
				IMU1.accx = ((int16_t)IMU1.usart_rx_buf[i+3]<<8 | IMU1.usart_rx_buf[i+2])/32768.0*16;//g
				IMU1.accy = ((int16_t)IMU1.usart_rx_buf[i+5]<<8 | IMU1.usart_rx_buf[i+4])/32768.0*16;//g
				IMU1.accz = ((int16_t)IMU1.usart_rx_buf[i+7]<<8 | IMU1.usart_rx_buf[i+6])/32768.0*16;//g
				IMU1.temper = ((int16_t)IMU1.usart_rx_buf[i+9]<<8 | IMU1.usart_rx_buf[i+8])/340.0+36.53;//°C	
				
				i = j;
			}
			else
				break;		
		}
		else if((IMU1.usart_rx_buf[i] == 0x55) && (IMU1.usart_rx_buf[i+1] == 0x52))//gyro data
		{
			sum = 0;
			for(j=i;j<i+10;j++)
			{
				sum += IMU1.usart_rx_buf[j];
			}
			if(sum == IMU1.usart_rx_buf[j])
			{
				IMU1.gyrox = ((int16_t)IMU1.usart_rx_buf[i+3]<<8 | IMU1.usart_rx_buf[i+2])/32768.0*2000;//°/s
				IMU1.gyroy = ((int16_t)IMU1.usart_rx_buf[i+5]<<8 | IMU1.usart_rx_buf[i+4])/32768.0*2000;//°/s
				IMU1.gyroz = ((int16_t)IMU1.usart_rx_buf[i+7]<<8 | IMU1.usart_rx_buf[i+6])/32768.0*2000;//°/s
				
				i = j;
			}
			else
				break;
		}
		else if((IMU1.usart_rx_buf[i] == 0x55) && (IMU1.usart_rx_buf[i+1] == 0x53))//ang data
		{
			sum = 0;
			for(j=i;j<i+10;j++)
			{
				sum += IMU1.usart_rx_buf[j];
			}
			if(sum == IMU1.usart_rx_buf[j])
			{
				temp_ang = ((int16_t)IMU1.usart_rx_buf[i+3]<<8 | IMU1.usart_rx_buf[i+2])/32768.0*180;//°
				IMU1.angx = (temp_ang>180)?(temp_ang-360):temp_ang;
				temp_ang = ((int16_t)IMU1.usart_rx_buf[i+5]<<8 | IMU1.usart_rx_buf[i+4])/32768.0*180;//°
				IMU1.angy = (temp_ang>180)?(temp_ang-360):temp_ang;
				temp_ang = ((int16_t)IMU1.usart_rx_buf[i+7]<<8 | IMU1.usart_rx_buf[i+6])/32768.0*180;//°
				IMU1.angz = (temp_ang>180)?(temp_ang-360):temp_ang;
				
				i = j;
			}
			else
				break;
		}
	}
}





