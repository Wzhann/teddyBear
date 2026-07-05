#ifndef _USER_COMM_H
#define _USER_COMM_H

#include "user_includes.h"

#define USART_COMM_TX_SIZE	20
#define USART_COMM_RX_SIZE	40


typedef struct
{
	char usart_tx_buf[USART_COMM_TX_SIZE];
	char usart_rx_buf[USART_COMM_RX_SIZE];				
	uint8_t rx_data_len;
	
}COMM_TYPEDEF;
extern COMM_TYPEDEF COMM;




void User_CommInit(void);
void User_CommIRQHandler(void);
void User_CommDataParas(void);
void User_CommResponseInfo(void);



#endif










