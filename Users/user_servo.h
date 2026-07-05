#ifndef _USER_SERVO_H
#define _USER_SERVO_H

#include "Action_Library.h"
#include "user_includes.h"

#define USART_SERVO_TX_SIZE	90
#define USART_SERVO_RX_SIZE	60

#define RUN_MODE 0x21
#define SPEED_KI 0x27
#define SPEED_SET 0x2e

#define BAUDRATE 0x06
#define TORQUESWITCH 0x28
#define POS_KP 0x15
#define maxtorque 0x10
#define tempLimit 0x0D
#define RUNMODE 0x21
#define PWMTIME 0x2C
#define ID 0x05


#define servo11_mid -1066
#define servo12_mid 277
#define serco11_max -54 // 顺时针为正，逆时针为负
#define serco11_min -2048
#define serco12_max 560
#define serco12_min 0


typedef struct
{
	UART_HandleTypeDef* p_usart_n;
	DMA_HandleTypeDef*	p_hdma_usart_n_rx;
	uint8_t usart_tx_buf[USART_SERVO_TX_SIZE];
	uint8_t usart_rx_buf[USART_SERVO_RX_SIZE];				
	uint8_t rx_data_len;
}USART_SERVO_TYPEDEF;


typedef struct
{
	int16_t pos_set;
	uint16_t ms_set;
	uint16_t speed_set;
	int16_t pos_read;
	uint16_t speed_read;
	uint16_t temper_read;
	uint16_t volt_read;
	uint16_t power_read;
	int16_t zero_ang;
	uint8_t AsynchronousWriteFlag;
	uint8_t servoStatus;
}SERVO_INFO_TYPEDEF;

extern SERVO_INFO_TYPEDEF SERVO[14];
extern int16_t servo_pos[14];	// 舵机角度值镜像数组，方便watch窗口查看

extern uint8_t SERVO_COMM_BUSY;

void User_ServoInit(void);
void User_ServoLegLEFT_IRQHandler(void);
void User_ServoNECK_IRQHandler(void);
void User_ServoLegRIGHT_IRQHandler(void);
void User_ServoHead_IRQHandler(void);
void User_ServoHeadIRQHandler(void);
void User_UsartServoDataParas(USART_SERVO_TYPEDEF* p_usart_servo_x);
void User_AllSetAngTime(void);

//FEETECH MOTOR
void FEETECH_UsartSetServoPos(uint8_t servo_id,int16_t pos,uint16_t ms,int16_t speed);
void FEETECH_LEGSYNCWRITE(uint8_t leg_id,int16_t pos[5],int16_t ms[5],int16_t speed[5]);
void FEETECH_HEADSYNCWRITE(int16_t pos[5],int16_t ms[5],int16_t speed[5]);
void FEETECH_NECKSYNCWRITE(int16_t pos[5], int16_t ms[5], int16_t speed[5]);
void FEETECH_ReadServoPos(uint8_t servo_id);
void FEETECH_LEGSYNCRead(uint8_t servo_id);
void sevroSetMode(uint8_t id,uint8_t mode);
extern int16_t goal_pos[15];//FEETECH POS GOAL
extern int16_t goal_speed[15];//FEETECH SPEED GOAL
extern int16_t goal_ms[15];//FEETECH MS GOAL
void FEETECH_HEADMODEWRITE(uint8_t mode);
void FEETECH_MODEWRITE(uint8_t mode, uint8_t leg_id);
void FEETECH_UsartSetServo(uint8_t servo_id, uint8_t address,uint8_t len,int value);
void User_BezierCurve(int stepping, ServoActionSeries_ram* Action_analyze);
void sevroSetZero(void);
void hand_angle(int angle_11,int angle_12);

#endif










