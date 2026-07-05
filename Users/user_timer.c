#include "user_timer.h"
#include "user_communication.h"
#include "DS18B20.h"

// 设置模式
#define _CMD_TYPE_SET 0
// 读取模式
#define _CMD_TYPE_READ 1

#define _READ_TYPE_ANG 0
#define _READ_TYPE_DATA 1

#define _DATA_TYPE_VOLT 1
#define _DATA_TYPE_CUR 2
#define _DATA_TYPE_POWER 3
#define _DATA_TYPE_TEMP 4

TIMER_TYPEDEF TIMER = {0};
uint8_t flag_act = 0;

void User_TimerInit(void)
{
	HAL_TIM_Base_Start_IT(&USER_htim_servo); //
	
}
void User_TeachTimerInit(void)
{
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim3); //
}

/*
舵机控制：
定时器周期：1ms
控制步骤：
				1ms->各串口DMA发送该串口总线上各舵机（6个）角度和执行时间（1.5ms）
				3ms->各串口DMA发送该串口总线上第1个舵机状态读取请求指令（400us）
				4ms->各串口DMA发送该串口总线上第2个舵机状态读取请求指令（400us）
				5ms->各串口DMA发送该串口总线上第3个舵机状态读取请求指令（400us）
				6ms->各串口DMA发送该串口总线上第4个舵机状态读取请求指令（400us）
				7ms->各串口DMA发送该串口总线上第5个舵机状态读取请求指令（400us）
每个舵机角度控制周期：7ms
每个舵机角度读取周期：7ms
*/
extern uint8_t personTeachFlag;
extern uint8_t OPEN;
extern uint8_t Init_OK;
extern float startPose;
uint16_t countForMPUReset;
uint8_t testinput;
uint8_t flagForMPUReset;
extern uint8_t releaseSevroFlag;
void User_TimerServoIRQ(void)
{
	
//	if(touchTopofHead_Downside == 1 && Init_OK == 1 && ActionNow == IDLE && TEACHMODE == 0)//摸脑袋
//	{
////		ActionNow = ACTION_M65;//M61. 头缓慢左右摇动几下后保持静止。

//	}
//	if(touchChin_Downside == 1 && Init_OK == 1 && ActionNow == IDLE && TEACHMODE == 0)//摸下巴
//	{
////		ActionNow = ACTION_M61;//M65. 抬手摸脸或耳朵，随后缓缓放下。
//	}
//	if(humanDetection_Downside == 1 && Init_OK == 1 && ActionNow == IDLE && TEACHMODE == 0)//人体检测
//	{
////		ActionNow = ACTION_HELLO;//hello
//	}
	/*陀螺仪重启*/
	countForMPUReset++;
	if(countForMPUReset > 8500 && flagForMPUReset == 0)
	{
		countForMPUReset = 8501;
		if(startPose == 0)
		{
			SoftwareReset();
		}
		else flagForMPUReset = 1;
	}
	
	static uint8_t cmd_type = _CMD_TYPE_READ;
	static uint8_t read_type = _READ_TYPE_ANG;
	static uint8_t cnt_servo_id = 0;
	static uint8_t wait_to_set = 0;
	if (__HAL_TIM_GET_IT_SOURCE(&USER_htim_servo, TIM_IT_UPDATE) != RESET && OPEN == 1)
	{
		SERVO_COMM_BUSY = 1;
		switch (cmd_type)
		{
		case _CMD_TYPE_SET: // 总共12个舵机发送一次控制指令，串口通信时间约750us
		{
			wait_to_set++;
			if (wait_to_set == 1)
			{
				if ((TEACHMODE != 1 && flag_act == 1 && actionStop == 0 ) || personTeachFlag == 1) // 示教模式下，电机不动
//					;
					User_AllSetAngTime();
			}
			else if (wait_to_set == 2) // 2
			{
				cmd_type = _CMD_TYPE_READ;
				wait_to_set = 0;
			}
		}
		break;
		case _CMD_TYPE_READ: // 发送一次读指令，从发送该指令到舵机返回数据串口总共通信时间约400us（发送100us，等待响应100us，回传200us）
		{
			if (read_type == _READ_TYPE_ANG) // 读角度
			{
				cnt_servo_id++;
//				FEETECH_ReadServoPos(cnt_servo_id % 2 + 11); // 头部两个舵机轮流读取
//				if((releaseSevroFlag == 1) && (cnt_servo_id == 1|| cnt_servo_id == 2|| cnt_servo_id == 3
//					|| cnt_servo_id == 6|| cnt_servo_id == 7|| cnt_servo_id == 8))
//				{
//				}
//				else
				if((releaseSevroFlag == 0))
				FEETECH_ReadServoPos(cnt_servo_id);
//				FEETECH_ReadServoPos(cnt_servo_id + 5);
			}
			if (cnt_servo_id == 12) // 5个1ms分别读取了5个舵机的数据后，进入新的周期并切换读取内容（读角度/读数据）
			{
				cnt_servo_id = 0;
				cmd_type = _CMD_TYPE_SET;
			}
		}
		break;
		default:
			break;
		}
		SERVO_COMM_BUSY = 0;
	}
	
		static uint16_t cnt = 0;
	static uint8_t first_time = 1;
	
	if(first_time == 1)
	{
		first_time = 0;
		ds18b20_search_ID();
	}
	if(cnt++ == 100)
	{
		ds18b20_get_temperature_multiple();		
		cnt = 0;
	}
	
}
//int vount_action;
// 用于示教的定时器中断，如果不需要示教模式可以关闭
int T_COUNTER = 0;		  // 记录示教过程中进入中断的次数
int __t_count = 0;
int step_record = 0;	  // 记录此次示教模式的初始时刻
int last_step_record = 0; // 记录上次示教模式的结束时刻
extern uint8_t PoweronAction;
int count_power;
void User_TimerTeachIRQ(void)
{
//	
//	if (HAL_GPIO_ReadPin(Power_in_GPIO_Port, Power_in_Pin) == GPIO_PIN_RESET)
//  {
//	  count_power++;
//  }
//  else count_power = 0;
//  if(count_power > 6) HAL_GPIO_WritePin(Power_out_GPIO_Port,Power_out_Pin,GPIO_PIN_RESET);//板子供电
  
//	vount_action++;
//	if(vount_action >= 100)
//	{
//		vount_action = 0;
//		if(PoweronAction == 0)
//			{
//				PoweronAction = 1;
//				ActionNow = ACTION_Yawn;
//			}
//	}
	
//	if (__HAL_TIM_GET_IT_SOURCE(&USER_htim_teach, TIM_IT_UPDATE) != RESET)
//	{
//		if (TEACHMODE == 1) // 如果是示教模式
//		{
//			if(TEACH_OK == 1)
//			{
//				if(T_COUNTER == 1)
//				{
//					__t_count++;
//					// 存储示教过程中的角度数据
//					for (int i = 0; i < 14; i++)
//						_Action_TEACH.motion[0].actions[__t_count - 1].servoAngles[i] = SERVO[i].pos_read;
//					T_COUNTER = 0;
//				}
//			}
//		}
//	}

	if (__HAL_TIM_GET_IT_SOURCE(&USER_htim_teach, TIM_IT_UPDATE) != RESET)
	{
		if (TEACHMODE == 1) // 如果是示教模式
		{
			T_COUNTER++;															   // 进入计数
			if (step_record == last_step_record && TEACH_OK == 1 && TEACH_FINISH != 1) // 如果示教模式刚刚开始
			{
				step_record = T_COUNTER; // 记录此次示教模式的初始时刻
			}
			else if (step_record != last_step_record && TEACH_OK == 1 && TEACH_FINISH != 1)
			{
				if (T_COUNTER - step_record > TEACH_TOTAL_STEP)
				{
					// 结束本次示教
					TEACH_FINISH = 1;
					personTeachFlag = 0;
					last_step_record = step_record;
				}
				else if ((T_COUNTER - step_record) >= 0 && (T_COUNTER - step_record) <= TEACH_TOTAL_STEP)
				{
					// 存储示教过程中的角度数据
					for (int i = 0; i < 14; i++)
						_Action_TEACH.motion[0].actions[T_COUNTER - step_record - 1].servoAngles[i] = SERVO[i].pos_read;
				}
			}
		}
	}
}


uint16_t timerStepForAction;
extern uint16_t timerStepForActionLast;
uint16_t actionSwitchTime = ACTIONTIMESTEP;//0.2s切换一次，和示教一样

uint16_t countTimerStepForAction = 0;
void User_TimerActionIRQ(void)
{
	if(ActionNow != IDLE)
	{
		countTimerStepForAction ++;
//		timerStepForAction ++;
	}
	else 
	{
		countTimerStepForAction = 0;
		timerStepForAction = 0;
		timerStepForActionLast = 0;
	}
	
	if(countTimerStepForAction >= actionSwitchTime)
	{
		countTimerStepForAction = 0;
		timerStepForAction ++;
	}
}

