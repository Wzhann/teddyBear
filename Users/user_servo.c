// claude
#include "user_servo.h"
#include "stdio.h"
#include "string.h"

// 控制板上面的左腿、右腿是相反的
USART_SERVO_TYPEDEF USART_LEG1 = {0};	   // usart1
USART_SERVO_TYPEDEF USART_LEG2 = {0};	   // usart1
USART_SERVO_TYPEDEF USART_LEG3 = {0};	   // usart2
USART_SERVO_TYPEDEF USART_LEG4 = {0};	   // usart2
USART_SERVO_TYPEDEF USART_RIGHT_LEG = {0}; // 右边腿 ID:1 - 5 usart1
USART_SERVO_TYPEDEF USART_LEFT_LEG = {0};  // 左边腿 ID: 6 - 10 usart2
USART_SERVO_TYPEDEF USART_HEAD = {0};	   // 头 ID:12
USART_SERVO_TYPEDEF USART_NECK = {0};	   // 脖子 ID:11

SERVO_INFO_TYPEDEF SERVO[14] = {0};
int16_t servo_pos[14] = {0};	// 舵机角度值镜像数组，方便watch窗口查看

uint8_t SERVO_COMM_BUSY = 0;
extern int16_t ang_goal[15];

int16_t goal_pos[15] = {0};	  // FEETECH POS GOAL
int16_t goal_speed[15] = {0}; // FEETECH SPEED GOAL
int16_t goal_ms[15] = {0};	  // FEETECH MS GOAL
// 伺服舵机初始化
void User_ServoInit(void)
{
	// 各组舵机对应串口
	USART_LEG1.p_usart_n = &huart1;
	USART_LEG1.p_hdma_usart_n_rx = &hdma_usart1_rx;
	USART_LEG2.p_usart_n = &huart1;
	USART_LEG2.p_hdma_usart_n_rx = &hdma_usart1_rx;
	USART_LEG3.p_usart_n = &huart2;
	USART_LEG3.p_hdma_usart_n_rx = &hdma_usart2_rx;
	USART_LEG4.p_usart_n = &huart2;
	USART_LEG4.p_hdma_usart_n_rx = &hdma_usart2_rx;

	USART_LEFT_LEG.p_usart_n = &huart1;
	USART_LEFT_LEG.p_hdma_usart_n_rx = &hdma_usart1_rx;
	USART_RIGHT_LEG.p_usart_n = &huart2;
	USART_RIGHT_LEG.p_hdma_usart_n_rx = &hdma_usart2_rx;
	USART_HEAD.p_usart_n = &huart3;
	USART_HEAD.p_hdma_usart_n_rx = &hdma_usart3_rx;
	USART_NECK.p_usart_n = &huart5;
	USART_NECK.p_hdma_usart_n_rx = &hdma_uart5_rx;

	// 各个伺服舵机的初始信息（速度、位置、时间）
	for (int i = 0; i <= 12; i++)
	{
		goal_speed[i] = 2000;
		goal_pos[i] = 0;
		goal_ms[i] = 0;
	}

	goal_speed[11] = 3000;
	goal_pos[1] = 0;
	goal_pos[2] = 0;
	goal_pos[3] = 0;
	goal_pos[4] = 0;
	goal_pos[5] = 0;

	goal_pos[6] = 0;
	goal_pos[7] = 0;
	goal_pos[8] = 0;
	goal_pos[9] = 0;
	goal_pos[10] = 0;

	goal_pos[11] = 0;
	goal_pos[12] = 0;

	for (int i = 0; i <= 14; i++)
		SERVO[i].zero_ang = 0;

	// 启动空闲中断接收
	__HAL_UART_ENABLE_IT(USART_LEFT_LEG.p_usart_n, UART_IT_IDLE);
	HAL_UART_Receive_DMA(USART_LEFT_LEG.p_usart_n, (uint8_t *)USART_LEFT_LEG.usart_rx_buf, USART_SERVO_RX_SIZE);

	__HAL_UART_ENABLE_IT(USART_RIGHT_LEG.p_usart_n, UART_IT_IDLE);
	HAL_UART_Receive_DMA(USART_RIGHT_LEG.p_usart_n, (uint8_t *)USART_RIGHT_LEG.usart_rx_buf, USART_SERVO_RX_SIZE);

	__HAL_UART_ENABLE_IT(USART_HEAD.p_usart_n, UART_IT_IDLE);
	HAL_UART_Receive_DMA(USART_HEAD.p_usart_n, (uint8_t *)USART_HEAD.usart_rx_buf, USART_SERVO_RX_SIZE);

	__HAL_UART_ENABLE_IT(USART_NECK.p_usart_n, UART_IT_IDLE);
	HAL_UART_Receive_DMA(USART_NECK.p_usart_n, (uint8_t *)USART_NECK.usart_rx_buf, USART_SERVO_RX_SIZE);
	//	HAL_Delay(100);
}
// 串口空闲中断接收
void User_ServoLegRIGHT_IRQHandler(void)
{
	if (RESET != __HAL_UART_GET_FLAG(USART_RIGHT_LEG.p_usart_n, UART_FLAG_IDLE)) // 检查UART的空闲中断标志位是否被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(USART_RIGHT_LEG.p_usart_n);														   // 清除中断标志位，防止重复触发中断
		HAL_UART_DMAStop(USART_RIGHT_LEG.p_usart_n);																   // 终止当前DMA传输，确保后续操作（如计算数据长度）的准确性
		USART_RIGHT_LEG.rx_data_len = USART_SERVO_RX_SIZE - __HAL_DMA_GET_COUNTER(USART_RIGHT_LEG.p_hdma_usart_n_rx);  // 计算实际接收长度
		User_UsartServoDataParas(&USART_RIGHT_LEG);																	   // 解析数据
		HAL_UART_Receive_DMA(USART_RIGHT_LEG.p_usart_n, (uint8_t *)USART_RIGHT_LEG.usart_rx_buf, USART_SERVO_RX_SIZE); // 重启DMA接收
	}
}
void User_ServoLegLEFT_IRQHandler(void)
{
	if (RESET != __HAL_UART_GET_FLAG(USART_LEFT_LEG.p_usart_n, UART_FLAG_IDLE)) // 检查UART的空闲中断标志位是否被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(USART_LEFT_LEG.p_usart_n);														 // 清除中断标志位，防止重复触发中断
		HAL_UART_DMAStop(USART_LEFT_LEG.p_usart_n);																	 // 终止当前DMA传输，确保后续操作（如计算数据长度）的准确性
		USART_LEFT_LEG.rx_data_len = USART_SERVO_RX_SIZE - __HAL_DMA_GET_COUNTER(USART_LEFT_LEG.p_hdma_usart_n_rx);	 // 计算实际接收长度
		User_UsartServoDataParas(&USART_LEFT_LEG);																	 // 解析数据
		HAL_UART_Receive_DMA(USART_LEFT_LEG.p_usart_n, (uint8_t *)USART_LEFT_LEG.usart_rx_buf, USART_SERVO_RX_SIZE); // 重启DMA接收
	}
}
void User_ServoHead_IRQHandler(void)
{
	if (RESET != __HAL_UART_GET_FLAG(USART_HEAD.p_usart_n, UART_FLAG_IDLE)) // 检查UART的空闲中断标志位是否被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(USART_HEAD.p_usart_n);													 // 清除中断标志位
		HAL_UART_DMAStop(USART_HEAD.p_usart_n);																 // 终止当前DMA传输，确保后续操作（如计算数据长度）的准确性
		USART_HEAD.rx_data_len = USART_SERVO_RX_SIZE - __HAL_DMA_GET_COUNTER(USART_HEAD.p_hdma_usart_n_rx);	 // 计算实际接收长度
		User_UsartServoDataParas(&USART_HEAD);																 // 解析数据
		HAL_UART_Receive_DMA(USART_HEAD.p_usart_n, (uint8_t *)USART_HEAD.usart_rx_buf, USART_SERVO_RX_SIZE); // 重启DMA接收
	}
}
void User_ServoNECK_IRQHandler(void)
{
	if (RESET != __HAL_UART_GET_FLAG(USART_NECK.p_usart_n, UART_FLAG_IDLE)) // 检查UART的空闲中断标志位是否被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(USART_NECK.p_usart_n);													 // 清除中断标志位
		HAL_UART_DMAStop(USART_NECK.p_usart_n);																 // 终止当前DMA传输，确保后续操作（如计算数据长度）的准确性
		USART_NECK.rx_data_len = USART_SERVO_RX_SIZE - __HAL_DMA_GET_COUNTER(USART_NECK.p_hdma_usart_n_rx);	 // 计算实际接收长度
		User_UsartServoDataParas(&USART_NECK);																 // 解析数据
		HAL_UART_Receive_DMA(USART_NECK.p_usart_n, (uint8_t *)USART_NECK.usart_rx_buf, USART_SERVO_RX_SIZE); // 重启DMA接收
	}
}

// 舵机返回数据解析
void User_UsartServoDataParas(USART_SERVO_TYPEDEF *p_usart_servo_x)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	uint8_t servo_id;

	// 最小有效帧长: 帧头(2) + ID(1) + 长度(1) + 错误(1) + 校验和(1) = 6字节
	if (p_usart_servo_x->rx_data_len < 6)
		return;

	for (i = 2; i < p_usart_servo_x->rx_data_len - 1; i++)
	{
		sum += p_usart_servo_x->usart_rx_buf[i];
	}
	sum = ~sum;

	servo_id = p_usart_servo_x->usart_rx_buf[2];
	// 舵机ID边界检查: 有效范围 0-13 (SERVO数组大小14)
	if (servo_id >= 14)
		return;

	//	if ((p_usart_servo_x->usart_rx_buf[0] == 0xFF) && (p_usart_servo_x->usart_rx_buf[1] == 0xFF)
	//		&& (p_usart_servo_x->usart_rx_buf[4] == 0x00) && (p_usart_servo_x->usart_rx_buf[i] == sum))
	if ((p_usart_servo_x->usart_rx_buf[0] == 0xFF) && (p_usart_servo_x->usart_rx_buf[1] == 0xFF) && (p_usart_servo_x->usart_rx_buf[3] == 12) && (p_usart_servo_x->usart_rx_buf[i] == sum))
	{
		SERVO[servo_id].pos_read = (p_usart_servo_x->usart_rx_buf[5] + (((int16_t)p_usart_servo_x->usart_rx_buf[6]) << 8)) - SERVO[servo_id].zero_ang - 2048;
		servo_pos[servo_id] = SERVO[servo_id].pos_read;	// 同步更新镜像数组
		SERVO[servo_id].speed_read = p_usart_servo_x->usart_rx_buf[7] + (((int16_t)p_usart_servo_x->usart_rx_buf[8]) << 8);
		SERVO[servo_id].power_read = p_usart_servo_x->usart_rx_buf[9] + (((int16_t)p_usart_servo_x->usart_rx_buf[10]) << 8);
		SERVO[servo_id].volt_read = p_usart_servo_x->usart_rx_buf[11];
		SERVO[servo_id].temper_read = p_usart_servo_x->usart_rx_buf[12];
		SERVO[servo_id].AsynchronousWriteFlag = p_usart_servo_x->usart_rx_buf[13];
		SERVO[servo_id].servoStatus = p_usart_servo_x->usart_rx_buf[14];

		//		SERVO[servo_id].temper_read = p_usart_servo_x->usart_rx_buf[5];
	}
}

void RECOVERY_0_FEETECH(uint8_t servo_id)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	USART_SERVO_TYPEDEF *p_usart_servo_x;
	switch (servo_id)
	{
	case 1:
	case 2:
	case 3:
		p_usart_servo_x = &USART_LEG1;
		break;
	case 4:
	case 5:
		p_usart_servo_x = &USART_LEG2;
		break;
	case 6:
	case 7:
	case 8:
		p_usart_servo_x = &USART_LEG3;
		break;
	case 9:
	case 10:
		p_usart_servo_x = &USART_LEG4;
		break;
	case 11:
		p_usart_servo_x = &USART_NECK;
		break;
	case 12:
		p_usart_servo_x = &USART_HEAD;
		break;
	default:
		break;
	}
	p_usart_servo_x->usart_tx_buf[0] = 0xFF;	 // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF;	 // 帧头
	p_usart_servo_x->usart_tx_buf[2] = servo_id; // 舵机ID号
	p_usart_servo_x->usart_tx_buf[3] = 02;		 // 数据包有效数据长度
	p_usart_servo_x->usart_tx_buf[4] = 0x06;	 // 写指令
												 //	p_usart_servo_x->usart_tx_buf[5] = 0x00;
												 //	p_usart_servo_x->usart_tx_buf[6] = 0x00;
	sum = 0;
	for (i = 2; i <= 4; i++)
	{
		sum += p_usart_servo_x->usart_tx_buf[i];
	}
	sum %= 256;
	sum = ~sum;
	p_usart_servo_x->usart_tx_buf[5] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）
	if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
		HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 6);
}

// 舵机角度写入函数（WRITE DATA）
void FEETECH_UsartSetServoPos(uint8_t servo_id, int16_t pos, uint16_t ms, int16_t speed)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	USART_SERVO_TYPEDEF *p_usart_servo_x;

	switch (servo_id)
	{
	case 1:
	case 2:
	case 3:
		p_usart_servo_x = &USART_LEG1;
		break;
	case 4:
	case 5:
	case 6:
		p_usart_servo_x = &USART_LEG2;
		break;
	case 7:
	case 8:
	case 9:
		p_usart_servo_x = &USART_LEG3;
		break;
	case 10:
	case 11:
	case 12:
		p_usart_servo_x = &USART_LEG4;
		break;
	case 13:
		p_usart_servo_x = &USART_HEAD;
		break;
	case 14:
		p_usart_servo_x = &USART_NECK;
		break;
	default:
		break;
	}

	p_usart_servo_x->usart_tx_buf[0] = 0xFF;		  // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF;		  // 帧头
	p_usart_servo_x->usart_tx_buf[2] = servo_id;	  // 舵机ID号
	p_usart_servo_x->usart_tx_buf[3] = 0x09;		  // 数据包有效数据长度
	p_usart_servo_x->usart_tx_buf[4] = 0x03;		  // 写指令
	p_usart_servo_x->usart_tx_buf[5] = 0x2A;		  // 控制表里目标位置的首地址
	p_usart_servo_x->usart_tx_buf[6] = pos & 0xff;	  // 舵机目标位置 低8位
	p_usart_servo_x->usart_tx_buf[7] = pos >> 8;	  // 舵机目标位置 高8位
	p_usart_servo_x->usart_tx_buf[8] = ms & 0xff;	  // 舵机目标时间 低8位
	p_usart_servo_x->usart_tx_buf[9] = ms >> 8;		  // 舵机目标时间 高8位
	p_usart_servo_x->usart_tx_buf[10] = speed & 0xff; // 舵机目标速度 高8位
	p_usart_servo_x->usart_tx_buf[11] = speed >> 8;	  // 舵机目标速度 低8位

	sum = 0;
	for (i = 2; i <= 11; i++)
	{
		sum += p_usart_servo_x->usart_tx_buf[i];
	}
	sum %= 256;
	sum = ~sum;
	p_usart_servo_x->usart_tx_buf[12] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

	if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
		HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 13);
}

void FEETECH_UsartSetServo(uint8_t servo_id, uint8_t address, uint8_t len, int value)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	USART_SERVO_TYPEDEF *p_usart_servo_x;
	switch (servo_id)
	{
	case 1:
	case 2:
	case 3:
		p_usart_servo_x = &USART_LEG1;
		break;
	case 4:
	case 5:
		p_usart_servo_x = &USART_LEG2;
		break;
	case 6:
	case 7:
	case 8:
		p_usart_servo_x = &USART_LEG3;
		break;
	case 9:
	case 10:
		p_usart_servo_x = &USART_LEG4;
		break;
	case 11:
		p_usart_servo_x = &USART_NECK;
		break;
	case 12:
		p_usart_servo_x = &USART_HEAD;
		break;
	default:
		break;
	}
	// p_usart_servo_x = &USART_LEG3;
	p_usart_servo_x->usart_tx_buf[0] = 0xFF;	 // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF;	 // 帧头
	p_usart_servo_x->usart_tx_buf[2] = servo_id; // 舵机ID号
	p_usart_servo_x->usart_tx_buf[3] = len + 3;	 // 数据包有效数据长度
	p_usart_servo_x->usart_tx_buf[4] = 0x03;	 // 写指令
	p_usart_servo_x->usart_tx_buf[5] = address;	 // 控制表里目标位置的首地址
	if (len == 1)
	{
		p_usart_servo_x->usart_tx_buf[6] = value; // 舵机目标位置 低8位
		sum = 0;
		for (i = 2; i <= 6; i++)
		{
			sum += p_usart_servo_x->usart_tx_buf[i];
		}
		sum %= 256;
		sum = ~sum;
		p_usart_servo_x->usart_tx_buf[7] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

		if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
			HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 8);
	}
	else if (len == 2)
	{
		p_usart_servo_x->usart_tx_buf[6] = value & 0xff; // 舵机目标位置 低8位
		p_usart_servo_x->usart_tx_buf[7] = value >> 8;	 // 舵机目标位置 高8位
		sum = 0;
		for (i = 2; i <= 7; i++)
		{
			sum += p_usart_servo_x->usart_tx_buf[i];
		}
		sum %= 256;
		sum = ~sum;
		p_usart_servo_x->usart_tx_buf[8] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

		if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
			HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 9);
	}
}
// mode 0;2
void sevroSetMode(uint8_t id, uint8_t mode)
{
	FEETECH_UsartSetServo(id, 0x37, 1, 0);
	osDelay(10);
	FEETECH_UsartSetServo(id, RUNMODE, 1, mode);
	osDelay(10);
	//	if(mode == 2)
	//	{
	//		FEETECH_UsartSetServo(id,PWMTIME,2,0);
	//		osDelay(10);
	//	}
}
void sevroSetZero(void)
{
	for (uint8_t i = 1; i <= 12; i++)
	{
		FEETECH_UsartSetServo(i, 0x37, 1, 0);
		osDelay(10);
		FEETECH_UsartSetServo(i, TORQUESWITCH, 1, 128);
		osDelay(10);
	}
	//
	//		FEETECH_UsartSetServo(11,0x37,1 ,0);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(11,POS_KP,1,15);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(12,0x37,1 ,0);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(12,POS_KP,1,10);
	//		osDelay(10);
	////
	//		FEETECH_UsartSetServo(9,0x37,1 ,0);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(9,TORQUESWITCH,1,128);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(10,0x37,1 ,0);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(10,TORQUESWITCH,1,128);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(4,0x37,1 ,0);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(4,TORQUESWITCH,1,128);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(2,0x37,1 ,0);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(2,TORQUESWITCH,1,128);
	//		osDelay(10);

	//		FEETECH_UsartSetServo(5,0x37,1 ,0);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(5,ID,1,2);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(5,0x37,1 ,0);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(5,ID,1,6);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(2,0x37,1 ,0);
	//		osDelay(10);
	//		FEETECH_UsartSetServo(2,TORQUESWITCH,1,128);
	//		osDelay(10);
}

// 舵机角度同步写入函数（SYNC WRITE）——HEAD
void FEETECH_HEADSYNCWRITE(int16_t pos[5], int16_t ms[5], int16_t speed[5])
{
	uint8_t i = 0;
	uint8_t sum = 0;
	USART_SERVO_TYPEDEF *p_usart_servo_x;
	p_usart_servo_x = &USART_HEAD;

	p_usart_servo_x->usart_tx_buf[0] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[2] = 0xFE; // 舵机ID号
	p_usart_servo_x->usart_tx_buf[3] = 0x0B; // 数据包有效数据长度
	p_usart_servo_x->usart_tx_buf[4] = 0x83; // 同步写指令
	p_usart_servo_x->usart_tx_buf[5] = 0x2A; // 控制表里目标位置的首地址
	p_usart_servo_x->usart_tx_buf[6] = 0x06; // 写入参数的长度

	p_usart_servo_x->usart_tx_buf[7] = 12;				 // 舵机ID
	p_usart_servo_x->usart_tx_buf[8] = pos[0] & 0xff;	 // 舵机目标位置 低8位
	p_usart_servo_x->usart_tx_buf[9] = pos[0] >> 8;		 // 舵机目标位置 高8位
	p_usart_servo_x->usart_tx_buf[10] = ms[0] & 0xff;	 // 舵机目标时间 低8位
	p_usart_servo_x->usart_tx_buf[11] = ms[0] >> 8;		 // 舵机目标时间 高8位
	p_usart_servo_x->usart_tx_buf[12] = speed[0] & 0xff; // 舵机目标速度 高8位
	p_usart_servo_x->usart_tx_buf[13] = speed[0] >> 8;	 // 舵机目标速度 低8位

	sum = 0;
	for (i = 2; i <= 13; i++)
	{
		sum += p_usart_servo_x->usart_tx_buf[i];
	}
	sum %= 256;
	sum = ~sum;
	p_usart_servo_x->usart_tx_buf[14] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

	if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
		HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 15);
	// HAL_UART_Transmit(p_usart_servo_x->p_usart_n,p_usart_servo_x->usart_tx_buf,13,100);
}
// 舵机角度同步写入函数（SYNC WRITE）——NECK
void FEETECH_NECKSYNCWRITE(int16_t pos[5], int16_t ms[5], int16_t speed[5])
{
	uint8_t i = 0;
	uint8_t sum = 0;
	USART_SERVO_TYPEDEF *p_usart_servo_x;
	p_usart_servo_x = &USART_NECK;

	p_usart_servo_x->usart_tx_buf[0] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[2] = 0xFE; // 舵机ID号
	p_usart_servo_x->usart_tx_buf[3] = 0x0B; // 数据包有效数据长度
	p_usart_servo_x->usart_tx_buf[4] = 0x83; // 同步写指令
	p_usart_servo_x->usart_tx_buf[5] = 0x2A; // 控制表里目标位置的首地址
	p_usart_servo_x->usart_tx_buf[6] = 0x06; // 写入参数的长度

	p_usart_servo_x->usart_tx_buf[7] = 11;				 // 舵机ID
	p_usart_servo_x->usart_tx_buf[8] = pos[0] & 0xff;	 // 舵机目标位置 低8位
	p_usart_servo_x->usart_tx_buf[9] = pos[0] >> 8;		 // 舵机目标位置 高8位
	p_usart_servo_x->usart_tx_buf[10] = ms[0] & 0xff;	 // 舵机目标时间 低8位
	p_usart_servo_x->usart_tx_buf[11] = ms[0] >> 8;		 // 舵机目标时间 高8位
	p_usart_servo_x->usart_tx_buf[12] = speed[0] & 0xff; // 舵机目标速度 高8位
	p_usart_servo_x->usart_tx_buf[13] = speed[0] >> 8;	 // 舵机目标速度 低8位

	sum = 0;
	for (i = 2; i <= 13; i++)
	{
		sum += p_usart_servo_x->usart_tx_buf[i];
	}
	sum %= 256;
	sum = ~sum;
	p_usart_servo_x->usart_tx_buf[14] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

	if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
		HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 15);
	// HAL_UART_Transmit(p_usart_servo_x->p_usart_n,p_usart_servo_x->usart_tx_buf,13,100);
}

// 舵机角度同步写入函数（SYNC WRITE）——LEG
void FEETECH_LEGSYNCWRITE(uint8_t leg_id, int16_t pos[5], int16_t ms[5], int16_t speed[5])
{
	uint8_t i = 0;
	uint8_t sum = 0;
	USART_SERVO_TYPEDEF *p_usart_servo_x;

	switch (leg_id)
	{
	case 1:
		p_usart_servo_x = &USART_LEFT_LEG;
		break;
	case 2:
		p_usart_servo_x = &USART_RIGHT_LEG;
		break;
	default:
		break;
	}

	p_usart_servo_x->usart_tx_buf[0] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[2] = 0xFE; // 舵机ID号
	p_usart_servo_x->usart_tx_buf[3] = 0x27; // 数据包有效数据长度
	p_usart_servo_x->usart_tx_buf[4] = 0x83; // 同步写指令
	p_usart_servo_x->usart_tx_buf[5] = 0x2A; // 控制表里目标位置的首地址
	p_usart_servo_x->usart_tx_buf[6] = 0x06; // 写入参数的长度

	p_usart_servo_x->usart_tx_buf[7] = leg_id * 5 - 4;	 // 舵机ID
	p_usart_servo_x->usart_tx_buf[8] = pos[0] & 0xff;	 // 舵机目标位置 低8位
	p_usart_servo_x->usart_tx_buf[9] = pos[0] >> 8;		 // 舵机目标位置 高8位
	p_usart_servo_x->usart_tx_buf[10] = ms[0] & 0xff;	 // 舵机目标时间 低8位
	p_usart_servo_x->usart_tx_buf[11] = ms[0] >> 8;		 // 舵机目标时间 高8位
	p_usart_servo_x->usart_tx_buf[12] = speed[0] & 0xff; // 舵机目标速度 高8位
	p_usart_servo_x->usart_tx_buf[13] = speed[0] >> 8;	 // 舵机目标速度 低8位

	p_usart_servo_x->usart_tx_buf[14] = leg_id * 5 - 3;	 // 舵机ID
	p_usart_servo_x->usart_tx_buf[15] = pos[1] & 0xff;	 // 舵机目标位置 低8位
	p_usart_servo_x->usart_tx_buf[16] = pos[1] >> 8;	 // 舵机目标位置 高8位
	p_usart_servo_x->usart_tx_buf[17] = ms[1] & 0xff;	 // 舵机目标时间 低8位
	p_usart_servo_x->usart_tx_buf[18] = ms[1] >> 8;		 // 舵机目标时间 高8位
	p_usart_servo_x->usart_tx_buf[19] = speed[1] & 0xff; // 舵机目标速度 高8位
	p_usart_servo_x->usart_tx_buf[20] = speed[1] >> 8;	 // 舵机目标速度 低8位

	p_usart_servo_x->usart_tx_buf[21] = leg_id * 5 - 2;	 // 舵机ID
	p_usart_servo_x->usart_tx_buf[22] = pos[2] & 0xff;	 // 舵机目标位置 低8位
	p_usart_servo_x->usart_tx_buf[23] = pos[2] >> 8;	 // 舵机目标位置 高8位
	p_usart_servo_x->usart_tx_buf[24] = ms[2] & 0xff;	 // 舵机目标时间 低8位
	p_usart_servo_x->usart_tx_buf[25] = ms[2] >> 8;		 // 舵机目标时间 高8位
	p_usart_servo_x->usart_tx_buf[26] = speed[2] & 0xff; // 舵机目标速度 高8位
	p_usart_servo_x->usart_tx_buf[27] = speed[2] >> 8;	 // 舵机目标速度 低8位

	p_usart_servo_x->usart_tx_buf[28] = leg_id * 5 - 1;	 // 舵机ID
	p_usart_servo_x->usart_tx_buf[29] = pos[3] & 0xff;	 // 舵机目标位置 低8位
	p_usart_servo_x->usart_tx_buf[30] = pos[3] >> 8;	 // 舵机目标位置 高8位
	p_usart_servo_x->usart_tx_buf[31] = ms[3] & 0xff;	 // 舵机目标时间 低8位
	p_usart_servo_x->usart_tx_buf[32] = ms[3] >> 8;		 // 舵机目标时间 高8位
	p_usart_servo_x->usart_tx_buf[33] = speed[3] & 0xff; // 舵机目标速度 高8位
	p_usart_servo_x->usart_tx_buf[34] = speed[3] >> 8;	 // 舵机目标速度 低8位

	p_usart_servo_x->usart_tx_buf[35] = leg_id * 5;		 // 舵机ID
	p_usart_servo_x->usart_tx_buf[36] = pos[4] & 0xff;	 // 舵机目标位置 低8位
	p_usart_servo_x->usart_tx_buf[37] = pos[4] >> 8;	 // 舵机目标位置 高8位
	p_usart_servo_x->usart_tx_buf[38] = ms[4] & 0xff;	 // 舵机目标时间 低8位
	p_usart_servo_x->usart_tx_buf[39] = ms[4] >> 8;		 // 舵机目标时间 高8位
	p_usart_servo_x->usart_tx_buf[40] = speed[4] & 0xff; // 舵机目标速度 高8位
	p_usart_servo_x->usart_tx_buf[41] = speed[4] >> 8;	 // 舵机目标速度 低8位

	sum = 0;
	for (i = 2; i <= 41; i++)
	{
		sum += p_usart_servo_x->usart_tx_buf[i];
	}
	sum %= 256;
	sum = ~sum;
	p_usart_servo_x->usart_tx_buf[42] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

	if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
		HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 43);
	// HAL_UART_Transmit(p_usart_servo_x->p_usart_n,p_usart_servo_x->usart_tx_buf,13,100);
}

// 舵机模式同步写入函数（SYNC WRITE）——LEG
void FEETECH_MODEWRITE(uint8_t mode, uint8_t leg_id)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	USART_SERVO_TYPEDEF *p_usart_servo_x;

	switch (leg_id)
	{
	case 1:
		p_usart_servo_x = &USART_LEFT_LEG;
		break;
	case 2:
		p_usart_servo_x = &USART_RIGHT_LEG;
		break;
	default:
		break;
	}

	p_usart_servo_x->usart_tx_buf[0] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[2] = 0xFE; // 舵机ID号
	p_usart_servo_x->usart_tx_buf[3] = 0x0e; // 数据包有效数据长度
	p_usart_servo_x->usart_tx_buf[4] = 0x83; // 同步写指令
	p_usart_servo_x->usart_tx_buf[5] = 0x21; // 控制表里目标位置的首地址
	p_usart_servo_x->usart_tx_buf[6] = 0x01; // 写入参数的长度

	p_usart_servo_x->usart_tx_buf[7] = leg_id * 5 - 4; // 舵机ID
	p_usart_servo_x->usart_tx_buf[8] = mode;

	p_usart_servo_x->usart_tx_buf[9] = leg_id * 5 - 3; // 舵机ID
	p_usart_servo_x->usart_tx_buf[10] = mode;

	p_usart_servo_x->usart_tx_buf[11] = leg_id * 5 - 2; // 舵机ID
	p_usart_servo_x->usart_tx_buf[12] = mode;

	p_usart_servo_x->usart_tx_buf[13] = leg_id * 5 - 1; // 舵机ID
	p_usart_servo_x->usart_tx_buf[14] = mode;

	p_usart_servo_x->usart_tx_buf[15] = leg_id * 5; // 舵机ID
	p_usart_servo_x->usart_tx_buf[16] = mode;

	sum = 0;
	for (i = 2; i <= 16; i++)
	{
		sum += p_usart_servo_x->usart_tx_buf[i];
	}
	sum %= 256;
	sum = ~sum;
	p_usart_servo_x->usart_tx_buf[17] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

	if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
		HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 18);
	// HAL_UART_Transmit(p_usart_servo_x->p_usart_n,p_usart_servo_x->usart_tx_buf,13,100);

	if (mode == 1)
	{
		p_usart_servo_x->usart_tx_buf[0] = 0xFF; // 帧头
		p_usart_servo_x->usart_tx_buf[1] = 0xFF; // 帧头
		p_usart_servo_x->usart_tx_buf[2] = 0xFE; // 舵机ID号
		p_usart_servo_x->usart_tx_buf[3] = 0x13; // 数据包有效数据长度
		p_usart_servo_x->usart_tx_buf[4] = 0x83; // 同步写指令
		p_usart_servo_x->usart_tx_buf[5] = 0x2e; // 控制表里目标位置的首地址
		p_usart_servo_x->usart_tx_buf[6] = 0x02; // 写入参数的长度

		p_usart_servo_x->usart_tx_buf[7] = leg_id * 5 - 4; // 舵机ID
		p_usart_servo_x->usart_tx_buf[8] = 0;
		p_usart_servo_x->usart_tx_buf[9] = 0;

		p_usart_servo_x->usart_tx_buf[10] = leg_id * 5 - 3; // 舵机ID
		p_usart_servo_x->usart_tx_buf[11] = 0;
		p_usart_servo_x->usart_tx_buf[12] = 0;

		p_usart_servo_x->usart_tx_buf[13] = leg_id * 5 - 2; // 舵机ID
		p_usart_servo_x->usart_tx_buf[14] = 0;
		p_usart_servo_x->usart_tx_buf[15] = 0;

		p_usart_servo_x->usart_tx_buf[16] = leg_id * 5 - 1; // 舵机ID
		p_usart_servo_x->usart_tx_buf[17] = 0;
		p_usart_servo_x->usart_tx_buf[18] = 0;

		p_usart_servo_x->usart_tx_buf[19] = leg_id * 5; // 舵机ID
		p_usart_servo_x->usart_tx_buf[20] = 0;
		p_usart_servo_x->usart_tx_buf[21] = 0;

		sum = 0;
		for (i = 2; i <= 21; i++)
		{
			sum += p_usart_servo_x->usart_tx_buf[i];
		}
		sum %= 256;
		sum = ~sum;
		p_usart_servo_x->usart_tx_buf[22] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

		if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
			HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 23);
	}
}

// 舵机模式同步写入函数（SYNC WRITE）——HEAD
void FEETECH_HEADMODEWRITE(uint8_t mode)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	USART_SERVO_TYPEDEF *p_usart_servo_x;
	p_usart_servo_x = &USART_HEAD;

	p_usart_servo_x->usart_tx_buf[0] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[2] = 0xFE; // 舵机ID号
	p_usart_servo_x->usart_tx_buf[3] = 0x08; // 数据包有效数据长度
	p_usart_servo_x->usart_tx_buf[4] = 0x83; // 同步写指令
	p_usart_servo_x->usart_tx_buf[5] = 0x21; // 控制表里目标位置的首地址
	p_usart_servo_x->usart_tx_buf[6] = 0x01; // 写入参数的长度

	p_usart_servo_x->usart_tx_buf[7] = 11; // 舵机ID
	p_usart_servo_x->usart_tx_buf[8] = mode;

	p_usart_servo_x->usart_tx_buf[9] = 12; // 舵机ID
	p_usart_servo_x->usart_tx_buf[10] = mode;

	sum = 0;
	for (i = 2; i <= 10; i++)
	{
		sum += p_usart_servo_x->usart_tx_buf[i];
	}
	sum %= 256;
	sum = ~sum;
	p_usart_servo_x->usart_tx_buf[11] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

	if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
		HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 12);
	// HAL_UART_Transmit(p_usart_servo_x->p_usart_n,p_usart_servo_x->usart_tx_buf,13,100);
	if (mode == 1)
	{
		p_usart_servo_x->usart_tx_buf[0] = 0xFF; // 帧头
		p_usart_servo_x->usart_tx_buf[1] = 0xFF; // 帧头
		p_usart_servo_x->usart_tx_buf[2] = 0xFE; // 舵机ID号
		p_usart_servo_x->usart_tx_buf[3] = 0x0a; // 数据包有效数据长度
		p_usart_servo_x->usart_tx_buf[4] = 0x83; // 同步写指令
		p_usart_servo_x->usart_tx_buf[5] = 0x2e; // 控制表里目标位置的首地址
		p_usart_servo_x->usart_tx_buf[6] = 0x02; // 写入参数的长度

		p_usart_servo_x->usart_tx_buf[7] = 11; // 舵机ID
		p_usart_servo_x->usart_tx_buf[8] = 0;
		p_usart_servo_x->usart_tx_buf[9] = 0;

		p_usart_servo_x->usart_tx_buf[10] = 12; // 舵机ID
		p_usart_servo_x->usart_tx_buf[11] = 0;
		p_usart_servo_x->usart_tx_buf[12] = 0;

		sum = 0;
		for (i = 2; i <= 12; i++)
		{
			sum += p_usart_servo_x->usart_tx_buf[i];
		}
		sum %= 256;
		sum = ~sum;
		p_usart_servo_x->usart_tx_buf[13] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数）

		if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
			HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 14);
	}
}
// 指定舵机角度读取函数
void FEETECH_ReadServoPos(uint8_t servo_id)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	static USART_SERVO_TYPEDEF *p_usart_servo_x;

	switch (servo_id)
	{
	case 1:
	case 2:
	case 3:
		p_usart_servo_x = &USART_LEG1;
		break;
	case 4:
	case 5:
		p_usart_servo_x = &USART_LEG2;
		break;
	case 6:
	case 7:
	case 8:
		p_usart_servo_x = &USART_LEG3;
		break;
	case 9:
	case 10:
		p_usart_servo_x = &USART_LEG4;
		break;
	case 11:
		p_usart_servo_x = &USART_NECK;
		break;
	case 12:
		p_usart_servo_x = &USART_HEAD;
		break;
	default:
		break;
	}

	// p_usart_servo_x = &USART_LEG1;

	p_usart_servo_x->usart_tx_buf[0] = 0xFF;	 // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF;	 // 帧头
	p_usart_servo_x->usart_tx_buf[2] = servo_id; // 舵机ID
	p_usart_servo_x->usart_tx_buf[3] = 0x04;	 // 数据包内容的字节长度
	p_usart_servo_x->usart_tx_buf[4] = 0x02;	 // 数据包内容
	p_usart_servo_x->usart_tx_buf[5] = 0x38;	 // 数据包地址
	p_usart_servo_x->usart_tx_buf[6] = 0x0A;	 // 数据包长度
												 //	p_usart_servo_x->usart_tx_buf[5] = 0x0d;	 // 数据包地址
												 //	p_usart_servo_x->usart_tx_buf[6] = 0x01;	 // 数据包长度

	sum = 0;
	for (i = 2; i <= 6; i++)
	{
		sum += p_usart_servo_x->usart_tx_buf[i];
	}
	sum %= 256;
	sum = ~sum;
	p_usart_servo_x->usart_tx_buf[7] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数后取反）
	if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
		HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 8);
}
// 舵机角度同步读取函数
void FEETECH_LEGSYNCRead(uint8_t servo_id)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	USART_SERVO_TYPEDEF *p_usart_servo_x;

	switch (servo_id)
	{
	case 1:
		p_usart_servo_x = &USART_LEFT_LEG;
		break;
	case 2:
		p_usart_servo_x = &USART_RIGHT_LEG;
		break;
	default:
		break;
	}

	p_usart_servo_x->usart_tx_buf[0] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[1] = 0xFF; // 帧头
	p_usart_servo_x->usart_tx_buf[2] = 0xFE; // 广播
	p_usart_servo_x->usart_tx_buf[3] = 0x06; // 数据包长度
	p_usart_servo_x->usart_tx_buf[4] = 0x82; // 数据包指令
	p_usart_servo_x->usart_tx_buf[5] = 0x38; // 数据包地址
	p_usart_servo_x->usart_tx_buf[6] = 0x08; // 数据包长度
	p_usart_servo_x->usart_tx_buf[7] = 0x01; // 数据包长度
	p_usart_servo_x->usart_tx_buf[8] = 0x02; // 数据包长度

	sum = 0;
	for (i = 2; i <= 8; i++)
	{
		sum += p_usart_servo_x->usart_tx_buf[i];
	}
	sum %= 256;
	sum = ~sum;
	p_usart_servo_x->usart_tx_buf[9] = sum; // 数据包校验和（对0到n-1的字节数据求和，然后跟256取余数后取反）

	if (p_usart_servo_x->p_usart_n->gState == HAL_UART_STATE_READY)
		HAL_UART_Transmit_DMA(p_usart_servo_x->p_usart_n, p_usart_servo_x->usart_tx_buf, 10);
}

void setmode(uint8_t mode)
{
	switch (mode)
	{
	case 0:

		break;

	case 1:

		break;
	}
}
extern uint8_t releaseSevroFlag;
extern uint8_t personTeachFlag;
#define SEVRO12MIN -40
#define SEVRO12MAX 640
// 用来综合控制整个机器人的所有腿部
void User_AllSetAngTime(void)
{
	int16_t tmp_pos[5] = {0};
	int16_t tmp_ms[5] = {0};
	int16_t tmp_speed[5] = {0};

	// 左腿 (leg_id=1): 舵机 ID 1-5
	for (int x = 0; x < 5; x++)
	{
		tmp_ms[x] = goal_ms[x + 1];
		tmp_speed[x] = goal_speed[x + 1];
	}
	tmp_pos[0] = 2048 + goal_pos[1];
	tmp_pos[1] = 2048 + goal_pos[2];
	tmp_pos[2] = 2048 + goal_pos[3];
	tmp_pos[3] = 2048 + goal_pos[4];
	tmp_pos[4] = 2048 + goal_pos[5];
	if (releaseSevroFlag == 0)
	{
		FEETECH_LEGSYNCWRITE(1, tmp_pos, tmp_ms, tmp_speed); // 左腿
	}

	// 右腿 (leg_id=2): 舵机 ID 6-10
	for (int x = 0; x < 5; x++)
	{
		tmp_ms[x] = goal_ms[x + 6];
		tmp_speed[x] = goal_speed[x + 6];
	}
	tmp_pos[0] = 2048 + goal_pos[6];
	tmp_pos[1] = 2048 + goal_pos[7];
	tmp_pos[2] = 2048 + goal_pos[8];
	tmp_pos[3] = 2048 + goal_pos[9];
	tmp_pos[4] = 2048 + goal_pos[10];
	if (releaseSevroFlag == 0)
	{
		FEETECH_LEGSYNCWRITE(2, tmp_pos, tmp_ms, tmp_speed); // 右腿
	}

	//	if(goal_pos[12] < SEVRO12MIN) goal_pos[12] = SEVRO12MIN;
	//	else if(goal_pos[12] > SEVRO12MAX) goal_pos[12] = SEVRO12MAX;
	if (personTeachFlag != 1)
	{
	#if SERVO12_ENABLE
		// 头部舵机 ID 12
		tmp_pos[0] = 2048 + goal_pos[12];
		tmp_ms[0] = goal_ms[12];
		tmp_speed[0] = goal_speed[12];
		FEETECH_HEADSYNCWRITE(tmp_pos, tmp_ms, tmp_speed); // 头部
	#endif

		// 脖子舵机 ID 11
		tmp_pos[0] = 2048 + goal_pos[11];
		tmp_ms[0] = goal_ms[11];
		tmp_speed[0] = goal_speed[11];
		FEETECH_NECKSYNCWRITE(tmp_pos, tmp_ms, tmp_speed); // 脖子
	}
}

int16_t servo11_angle, servo12_angle;

void hand_angle(int angle_11, int angle_12)
{
	int angle_11x = angle_11 * 4096 / 360 + servo11_mid;
	int angle_12x = angle_12 * 4096 / 360 + servo12_mid;
	if ((angle_11x < serco11_max) && (angle_11x > serco11_min))
	{
		servo11_angle = angle_11x;
	}
	else
		servo11_angle = servo11_mid;

	if ((angle_12x > serco12_min) && (angle_12x < serco12_max))
	{
		servo12_angle = angle_12x;
	}
	else
		servo12_angle = servo12_mid;
	goal_pos[11] = servo11_angle;
	goal_pos[12] = servo12_angle;
}

/*用贝塞尔来计算每单个动作的舵机角度*/
void User_BezierCurve(int stepping, ServoActionSeries_ram *Action_analyze)
{
	// 曲线上控制点个数(分度值)
	float count_t = 1.0f / (stepping - 1);
	for (int i = 0; i <= stepping - 1; i++)
	{
		float cur_t = count_t * (float)i;
		float one_minus_t = 1.0f - cur_t;
		for (int j = 1; j <= 12; j++)
		{
			Action_analyze->actions[i].servoAngles[j] = one_minus_t * Action_analyze->startservoAngles[j] + cur_t * Action_analyze->endservoAngles[j];
		}
	}
}
