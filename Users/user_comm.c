#include "user_comm.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define USART_COMM_HUSART huart7



COMM_TYPEDEF COMM = {0}; 

void User_CommInit(void)
{               
	__HAL_UART_ENABLE_IT(&USART_COMM_HUSART, UART_IT_RXNE);         
}

void User_CommIRQHandler(void)
{
	if(RESET != __HAL_UART_GET_FLAG(&USART_COMM_HUSART, UART_FLAG_RXNE))   
	{
		__HAL_UART_CLEAR_NEFLAG(&USART_COMM_HUSART);    		                                                         
		User_CommDataParas();///////
	}
} 


void User_CommDataParas(void)
{
	if(COMM.rx_data_len == 1)
	{

	}
}


void CommSendOut(char *str)
{
//	HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
}







