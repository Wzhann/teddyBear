#include "user_tasks.h"
#include "cmsis_os.h"
#include "user_servo.h"
#include "user_timer.h"
#include "user_states.h"
#include <stdbool.h>
#include "user_communication.h"
#include "user_adc.h"
#include "user_imu_i2c.h"
#include "adc.h"
#include "DS18B20.h"
#include "stdlib.h"
#include "Myiic_IMU.h"
#include "usart.h"

int16_t ang_goal[15] = {0};
uint16_t ms_goal[15] = {0};
uint8_t Action_done[50] = {0};
uint8_t ActionNowFlag = 1;
ACTION_STATE ActionNow = IDLE;  // 当前动作状态
ACTION_STATE ActionLast = IDLE; // 上一个动作状态
uint8_t speed;              // 舵机插补速度（单位0.1度）
int32_t step_counter = 1;       // 当前动作在动作序列中执行到第几步
uint8_t vis[15] = {0};          // 判断舵机是否到位
uint8_t Servo_Reset_Flag = 0;
const uint8_t Init_OK = 1;
int16_t LookPos[14] = {0};
uint8_t actionStandup_getStartAngle = 0; // 执行站立动作时获取当前舵机位置作为初始位置
Motion_t_ram *motion_ram_last = &_Action_TEACH;
Motion_t *motion_last = (Motion_t *)&Motion_Stand_Bow;

uint8_t ifStartAct = 0;
uint8_t PoweronAction; // 上电开始一系列动作
uint8_t flag_sendExecuting = 0;
uint8_t flag_sendCompleted = 0;

uint8_t sendmodework = 0; // 发送工作中的mode

uint8_t actionPoseNext = POSE_SITTING;
uint8_t actionPoseLast = POSE_SITTING;
uint8_t actionNeedReturn = 0;
extern uint16_t actionSwitchTime;
uint8_t releaseSevroFlag = 0;

uint8_t personTeachFlag = 0;
uint8_t indexActionShowMode = 0;
uint8_t indexActionShowModeStart = 0;
extern uint8_t showModeFlag;
extern uint8_t actionINDEXForshow1[13];
void User_Init_HIGH(void)
{
//	osDelay(3000);
#if 0
//    PoweronAction = 0;
    TEACHMODE = 0;
#else
//    PoweronAction = 1; // 上电开始一系列动作
    TEACHMODE = 1;
#endif

	__disable_irq();
	
	MX_UART4_Init();
	User_CommunicationInit(); // 通信协议初始化
	User_TimerInit();      // 统管全局的定时器
	User_TeachTimerInit(); // 示教模式专用定时器
	
	__enable_irq();
	__set_FAULTMASK(0);
	User_ServoInit();      // 舵机消息初始化（含应答等待）
	User_AdcInit();
    Action_init();         // 动作库初始化（加载舵机消息）
    
	for(uint8_t i = 1;i <= 12;i++)
	{
		osDelay(10);
		sevroSetMode(i,0);
	}
	
	osDelay(100);
	ds18b20_init();
	osDelay(100);
	
    
//	osDelay(1000);//等待舵机稳定
//	osDelay(1000);//等待舵机稳定
//	sevroSetZero();
	
	speed = 3;

	//	ActionNow = ACTION_SIT;
//    if (PoweronAction == 1)
//        ActionNow = ACTION_Yawn;
//    else
//        ActionNow = IDLE;
	
	
}

void User_Init_LOW(void)
{
	osDelay(1000);//等待舵机稳定
	MPU6050_Init();
	userImuInit();//上电校准姿态
	
}
int absInt(int num) {
    // 取绝对值：负数直接返回其相反数
    return num < 0 ? -num : num;
}


extern uint16_t timerStepForAction;
uint16_t timerStepForActionLast = 0;
int16_t goal_posOffsetAv[12];
int16_t speedActionUse[12];
int16_t differenceAction[12];
extern uint16_t actionSwitchTime;
// 判断单步动作是否到位（Flash动作库版本）
bool _SingleAction_CheckApproch(ServoActionSeries *action)
{
	int maxValue = 0;
    for (int i = 1; i <= 12; i++)
    {
//		if (goal_pos[i] <= action->actions[step_counter].servoAngles[i] - speed)
//            goal_pos[i] += speed;
//        else if (goal_pos[i] >= action->actions[step_counter].servoAngles[i] + speed)
//            goal_pos[i] -= speed;
		
//		goal_posOffsetAv[i] = action->actions[step_counter + 1].servoAngles[i]-action->actions[step_counter].servoAngles[i];
//		speedActionUse[i] = goal_posOffsetAv[i] /(200/11);// 若定时器200ms step+1，而这里11ms执行一次
//		if (goal_pos[i] <= action->actions[step_counter].servoAngles[i] - speed)
//            goal_pos[i] += speedActionUse[i];
//        else if (goal_pos[i] >= action->actions[step_counter].servoAngles[i] + speed)
//            goal_pos[i] -= speedActionUse[i];
		
//		goal_pos[i] = action->actions[step_counter].servoAngles[i];		
		if (goal_pos[i] <= action->actions[step_counter].servoAngles[i] - speed)
            goal_pos[i] += speed;
        else if (goal_pos[i] >= action->actions[step_counter].servoAngles[i] + speed)
            goal_pos[i] -= speed;
    }
	for(uint8_t i = 1;i<=12;i++)
	{
		differenceAction[i-1] = absInt(action->actions[step_counter].servoAngles[i] - action->actions[step_counter+1].servoAngles[i]);
	}
	
	for (uint8_t i = 1; i <= 12; i++) {
        // 如果当前元素大于当前最大值，更新最大值
        if (differenceAction[i-1] > maxValue) {
            maxValue = differenceAction[i-1];
        }
    }	
	
//	if(ActionNow != 999walk)//除了爬行
	if(1)
	{
//		if(step_counter<action->total_step-1)
//		{
//			 if (maxValue < 100) {
//				actionSwitchTime = 130;
//			}
//			// 逻辑2：差值 >= 100，线性映射（差值越大间隔越大）
//			else {
//				// 线性公式推导得 output = k*input + b（k为斜率，b为截距）
//				// 已知 input=340 时 output=190，满足"差值越小间隔越小"，k为正
//				// 为了线性映射一致性：假设差值=100时对应间隔190（最大值递减趋势）
//				// 斜率k计算：差值从100到340，间隔从250到190（线性递减，数值匹配）
//				const float k = (ACTIONTIMESTEP - 130.0f) / (420 - 100);  // 斜率≈-0.25
//				const float b = ACTIONTIMESTEP - k * 100;  // 截距≈275.0
//				
//				actionSwitchTime = k * maxValue + b;
//			}
//		}
//		else actionSwitchTime = ACTIONTIMESTEP;
		
		/*============== 固定步进时间 ==============*/
		actionSwitchTime = ACTIONTIMESTEP;
	}
	
	if(timerStepForAction != timerStepForActionLast)
	{
		timerStepForActionLast = timerStepForAction;
            // 未执行完序列步进
            if (step_counter < action->total_step - 1)
            {
                step_counter++;
                for (int i = 1; i <= 12; i++)
				{
					vis[i] = 0;
					goal_pos[i] = action->actions[step_counter-1].servoAngles[i];
				}
            }
            // 执行完成
            else if (step_counter == action->total_step - 1)
            {
				for (int i = 1; i <= 12; i++)
				{
						goal_pos[i] = action->actions[step_counter].servoAngles[i];
				}
                // 当前动作完成
                Action_done[action->actionId] = 1;
                // 相关参数复位
                for (int i = 1; i <= 12; i++)
                    vis[i] = 0;
                return true;
            }
	}
    return false;
}

// 判断单步动作是否到位（RAM/贝塞尔版本）
bool _SingleAction_CheckApproch_Bezier(ServoActionSeries_ram *action)
{
	for (int i = 1; i <= 12; i++)
    {
		if (goal_pos[i] <= action->actions[step_counter].servoAngles[i] - speed)
            goal_pos[i] += speed;
        else if (goal_pos[i] >= action->actions[step_counter].servoAngles[i] + speed)
            goal_pos[i] -= speed;
//		goal_pos[i] = action->actions[step_counter].servoAngles[i];
//		goal_posOffsetAv[i] = action->actions[step_counter + 1].servoAngles[i]-action->actions[step_counter].servoAngles[i];
//		speedActionUse[i] = goal_posOffsetAv[i] /(200/11);// 若定时器200ms step+1，而这里11ms执行一次
//		if (goal_pos[i] <= action->actions[step_counter].servoAngles[i] - speed)
//            goal_pos[i] += speedActionUse[i];
//        else if (goal_pos[i] >= action->actions[step_counter].servoAngles[i] + speed)
//            goal_pos[i] -= speedActionUse[i];
    }
	if(timerStepForAction != timerStepForActionLast)
	{
		timerStepForActionLast = timerStepForAction;
            // 未执行完序列步进
            if (step_counter < action->total_step - 1)
            {
                step_counter++;
                for (int i = 1; i <= 12; i++)
				{
					vis[i] = 0;
					goal_pos[i] = action->actions[step_counter-1].servoAngles[i];
				}
                    
                return false;
            }
            // 执行完成
            else if (step_counter == action->total_step - 1)
            {
				for (int i = 1; i <= 12; i++)
				{
						goal_pos[i] = action->actions[step_counter].servoAngles[i];
				}
                // 当前动作完成
                Action_done[action->actionId] = 1;
                // 相关参数复位
                for (int i = 1; i <= 12; i++)
                    vis[i] = 0;
                return true;
            }
	}
	
    return false;
}
extern uint8_t actionFromemotion;

// 从iter复位，让动作序列能再次触发运行
void Motion_Reset(Motion_t *motion_)
{
//	osDelay(10);
//	sevroSetMode(1,2);
//	sevroSetMode(2,2);
//	sevroSetMode(3,2);
//	
//	sevroSetMode(6,2);
//	sevroSetMode(7,2);
//	sevroSetMode(8,2);
	
    for (int i = 0; i < motion_->point_total; i++)
    {
        Action_done[motion_->motion[i].actionId] = 0; // 重置动作完成标志
    }
    step_counter = 1;
	
	if(flag_sendCompleted  == 0 && actionNeedReturn == 0 && motion_ != NULL) 
	{
		if(actionFromemotion == 1)
		ph.current_cmd = 1;
		else ph.current_cmd = 2;
		if(showModeFlag == 0)
		Send_Response(&ph, 0x03); // 发送执行完成应答
		flag_sendCompleted = 1;
		osDelay(1);
		stateRobot.mode = MODE_IDLE;
		sendStateActive(&ph, stateRobot);
		
	}
	if(motion_ != NULL)
		{
			ActionNow = IDLE;
			actionPoseLast = motion_->poseend; // 更新当前姿态为动作结束姿态
		}
		else
		{
			actionPoseLast = POSE_STANDING;
		}
    motion_->point_iter = 0;         // 重置动作迭代器
//	speed = 2;
		actionFromemotion = 0;
	actionSwitchTime = ACTIONTIMESTEP;
}
// 从iter复位，让动作序列能再次触发运行（RAM版本）
void Motion_Reset_Bezier(Motion_t_ram *motion_)
{
    for (int i = 0; i < motion_->point_total; i++)
    {
        Action_done[motion_->motion[i].actionId] = 0; // 重置动作完成标志
    }
    step_counter = 1;
	sendmodework = 0;
	if(flag_sendCompleted  == 0) 
	{
		ph.current_cmd = 2;
		Send_Response(&ph, 0x03); // 发送执行完成应答
		flag_sendCompleted = 1;
		osDelay(1);
		stateRobot.mode = MODE_IDLE;
		sendStateActive(&ph, stateRobot);
	}
	motion_->point_iter = 0;         // 重置动作迭代器
    actionStandup_getStartAngle = 0; // 重置重新获取当前角度从而生成新的贝塞尔曲线逻辑
	actionPoseLast = motion_->poseend; // 更新当前姿态为动作结束姿态
	ActionNow = IDLE;
}

extern uint8_t flag_act;
// 运动序列运行：通过判断iter步进，返回true表示序列执行完毕
bool Motion_Run(Motion_t *motion_)
{
	if(step_counter == 1 && releaseSevroFlag == 0) releaseSevroFlag = 1;
	if(releaseSevroFlag == 1)
	{
		osDelay(10);
		sevroSetMode(1,0);
		sevroSetMode(2,0);
		sevroSetMode(3,0);
		
		sevroSetMode(6,0);
		sevroSetMode(7,0);
		sevroSetMode(8,0);
		releaseSevroFlag = 0;
	}
	if(motion_ != NULL)
	flag_sendCompleted = 0;
	if(flag_sendExecuting == 0) 
	{
		Send_Response(&ph, 0x02); // 发送开始执行应答
		flag_sendExecuting = 1;
		osDelay(1);
	}
	if(sendmodework == 0)
	{
		stateRobot.mode = MODE_ACTION;
		sendStateActive(&ph, stateRobot);
		sendmodework = 1;
	}
    motion_last = motion_;
    if (ifStartAct == 0)
    {
        for (int i = 1; i <= 12; i++)
        {
            goal_pos[i] = motion_->motion[0].actions[1].servoAngles[i];
			//goal_pos[i] = SERVO[i].pos_read;
        }
        flag_act = 1;
        ifStartAct = 1;
    }
    if (_SingleAction_CheckApproch(&motion_->motion[motion_->point_iter]))
    {
        if (motion_->point_iter < motion_->point_total - 1)
        {
            motion_->point_iter++; // 切换到下一个动作点
            return false;
        }

        else if (motion_->point_iter == motion_->point_total - 1)
        {
            return true;
        }
        return false;
    } // 检查是否完成整个序列
    return false;
}

// 运动序列运行（RAM版本）：通过判断iter步进，返回true表示序列执行完毕
bool Motion_Run_Bezier(Motion_t_ram *motion_)
{
	if(step_counter == 1 && releaseSevroFlag == 0) releaseSevroFlag = 1;
	if(releaseSevroFlag == 1)
	{
		osDelay(10);
		sevroSetMode(1,0);
		sevroSetMode(2,0);
		sevroSetMode(3,0);
		
		sevroSetMode(6,0);
		sevroSetMode(7,0);
		sevroSetMode(8,0);
		releaseSevroFlag = 0;
	}
	
    motion_ram_last = motion_;
	if(flag_sendExecuting == 0) 
	{
		Send_Response(&ph, 0x02); // 发送开始执行应答
		flag_sendExecuting = 1;
		flag_sendCompleted = 0;
		osDelay(1);
	}
	if(sendmodework == 0)
	{
		stateRobot.mode = MODE_ACTION;
		sendStateActive(&ph, stateRobot);
		sendmodework = 1;
	}
    if (ifStartAct == 0)
    {
        for (int i = 1; i <= 12; i++)
        {
            goal_pos[i] = _Action_TEACH.motion[0].actions[1].servoAngles[i];
        }
        ifStartAct = 1;
        flag_act = 1;
    }
    if (_SingleAction_CheckApproch_Bezier(&motion_->motion[motion_->point_iter]))
    {
        if (motion_->point_iter < motion_->point_total - 1)
        {
            motion_->point_iter++; // 切换到下一个动作点
            return false;
        }

        else if (motion_->point_iter == motion_->point_total - 1)
        {
            return true;
        }
        return false;
    } // 检查是否完成整个序列
    return false;
}


/*
 * 姿态定义:
 * POSE_SITTING   = 1  // 坐
 * POSE_LYING     = 2  // 趴
 * POSE_STANDING  = 3  // 站立
 *
 * 姿态切换:
 * ACTION_SIT_TO_STAND   // 坐->站立
 * ACTION_STAND_TO_SIT   // 站立->坐
 * ACTION_SIT_TO_LIE     // 坐->趴
 * ACTION_LIE_TO_SIT     // 趴->坐
 * ACTION_LIE_TO_STAND   // 趴->站立
 * ACTION_STAND_TO_LIE   // 站立->趴
 */
void switchPose(uint8_t lastPose, uint8_t nowPose)
{
    uint8_t composeUnit = ((lastPose << 4) | (nowPose & 0x0f));
    // speed = 7;
    switch (composeUnit)
    {
    case 0x12: // 坐->趴
        ActionNow = ACTION_SIT_TO_LIE;
        break;

    case 0x13: // 坐->站立
        ActionNow = ACTION_SIT_TO_STAND;
        break;

    case 0x21: // 趴->坐
        ActionNow = ACTION_LIE_TO_SIT;
        break;

    case 0x23: // 趴->站立
        ActionNow = ACTION_LIE_TO_STAND;
        break;

    case 0x31: // 站立->坐
        ActionNow = ACTION_STAND_TO_SIT;
        break;

    case 0x32: // 站立->趴
        ActionNow = ACTION_STAND_TO_LIE;
        break;
    }
}

void actionDoprepare()
{
	switch(actionPoseLast)
	{
		case POSE_SITTING:
			if (Motion_Run(&MsittingInit) == true)
        {
            Motion_Reset(&MsittingInit); // 复位使能该动作库，让下次再触发
        }
		break;

		case POSE_LYING:
			if (Motion_Run(&MlyingInit) == true)
        {
            Motion_Reset(&MlyingInit); // 复位使能该动作库，让下次再触发
        }
		break;

		case POSE_STANDING:
			if (Motion_Run(&MstandingInit) == true)
        {
            Motion_Reset(&MstandingInit); // 复位使能该动作库，让下次再触发
        }
		break;
	}
	
}

extern uint8_t __t_count;
uint8_t showActionFirstEnd = 0;

// 辅助函数: 根据 ACTION_STATE 获取对应的 Motion_t 指针和目标姿态
static Motion_t *getMotionForAction(ACTION_STATE action, uint8_t *poseNext)
{
    *poseNext = POSE_SITTING; // 默认
    switch (action)
    {
    // ===== 站立姿态动作 =====
    case ACTION_STAND_BOW:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_Stand_Bow;
    case ACTION_STAND_DANCE1:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_Stand_Dance1;
    case ACTION_STAND_DANCE2:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_Stand_Dance2;
    case ACTION_STAND_STEPBACK:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_Stand_StepBack;
    case ACTION_STAND_SALUTE:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_Stand_Salute;
    case ACTION_STAND_BLOWKISS:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_Stand_BlowKiss;
    case ACTION_STAND_DANCE3:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_Stand_Dance3;
    case ACTION_STAND_HANDSHAKE:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_Stand_Handshake;
    case ACTION_STAND_PRAY:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_Stand_Pray;

    // ===== 坐姿态动作 =====
    case ACTION_SIT_HANDSHAKE:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_Sit_Handshake;
    case ACTION_SIT_HELLO:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_Sit_Hello;
    case ACTION_SIT_STRETCH:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_Sit_Stretch;
    case ACTION_SIT_SHAKEHEAD:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_Sit_ShakeHead;
    case ACTION_SIT_CHEER:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_Sit_Cheer;
    case ACTION_SIT_YAWN:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_Sit_Yawn;
    case ACTION_SIT_DRUM:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_Sit_Drum;
    case ACTION_SIT_WASHFACE:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_Sit_WashFace;

    // ===== 趴姿态动作 =====
    case ACTION_LIE_WAGHIPS:
        *poseNext = POSE_LYING;
        return (Motion_t *)&Motion_Lie_WagHips;
    case ACTION_LIE_PUSHUP:
        *poseNext = POSE_LYING;
        return (Motion_t *)&Motion_Lie_PushUp;
    case ACTION_LIE_CRAWL:
        *poseNext = POSE_LYING;
        return (Motion_t *)&Motion_Lie_Crawl;

    // ===== 姿态切换 =====
    case ACTION_SIT_TO_STAND:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_SitToStand;
    case ACTION_STAND_TO_SIT:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_StandToSit;
    case ACTION_SIT_TO_LIE:
        *poseNext = POSE_LYING;
        return (Motion_t *)&Motion_SitToLie;
    case ACTION_LIE_TO_SIT:
        *poseNext = POSE_SITTING;
        return (Motion_t *)&Motion_LieToSit;
    case ACTION_LIE_TO_STAND:
        *poseNext = POSE_STANDING;
        return (Motion_t *)&Motion_LieToStand;
    case ACTION_STAND_TO_LIE:
        *poseNext = POSE_LYING;
        return (Motion_t *)&Motion_StandToLie;

    default:
        return NULL;
    }
}

void robotRun()
{
    // 示教模式单独处理
    if (ActionNow == ACTION_TEACH)
    {
        _Action_TEACH.motion[0].total_step = TEACH_TOTAL_STEP;
        if (Motion_Run_Bezier(&_Action_TEACH) == true)
            Motion_Reset_Bezier(&_Action_TEACH);
        return;
    }

    // 获取当前动作对应的 Motion_t 指针
    Motion_t *motion = getMotionForAction(ActionNow, &actionPoseNext);
    if (motion == NULL)
        return;

    // ===== DEBUG: 注释姿态检测，直接执行动作 =====
//    // 检查是否需要姿态切换
//    if (actionPoseNext != actionPoseLast)
//    {
//        actionNeedReturn = 1;
//        ActionLast = ActionNow;
//        switchPose(actionPoseLast, actionPoseNext);
//        return;
//    }

    // 执行动作
//    if (actionNeedReturn == 0)
//    {
        actionSwitchTime = ACTIONTIMESTEP;
        if (Motion_Run(motion) == true)
            Motion_Reset(motion);
//    }
}
void TeachmodeRUN(void)
{
    // 将上位机发过来的动作复制到RAM动作库（示教模式时使用）
    if (Servo_Reset_Flag == 1)
    {
        Servo_Reset_Flag = 0;
    }
    if (TEACHMODE == 1)
    {
        // 示教模式，控制所有舵机到位
        if (TEACH_FINISH == 1)
        {
            /*示教结束*/
            for (int i = 1; i <= 12; i++)
            {
                goal_pos[i] = _Action_TEACH.motion[0].actions[1].servoAngles[i];
            }

            TEACH_OK = 0;
            TEACH_FINISH = 0;
            TEACHMODE = 0;

            /*通知舵机去执行刚才的动作*/
            ActionNowFlag = 0;
            Action_done[0] = 0;
            step_counter = 1;
            ActionNow = ACTION_TEACH;

            /*设置舵机位置并锁住*/
            Action_Teachmode();
        }
    }
}

buzzerType buzzerWorking;
extern uint8_t workflag;
uint8_t rgbTimes;
uint8_t buzzerTimes;
uint8_t poseNow;

void StartTaskHigh(void const *argument)
{
    User_Init_HIGH();
    for (;;)
    {
        osDelay(1);
    }
}
uint8_t OPEN = 1;
uint8_t systemPowerOn = 0;
void StartTaskMid(void const *argument)
{
    for (;;)
    {
        if (TEACHMODE == 1)
            TeachmodeRUN();
        else if(personTeachFlag == 0)
            robotRun();
//		if(actionStop == 1)
//		{
//			Motion_Reset(motion_last);
//			Motion_Reset_Bezier(motion_ram_last);
//			actionPoseLast = motion_last->poseend;
//		}
		
//        for (int i = 1; i <= 12; i++)
//            LookPos[i] = SERVO[i].pos_read;
//        if (OPEN == 1)
//            HAL_GPIO_WritePin(Servo_Power_12V_GPIO_Port, Servo_Power_12V_Pin, GPIO_PIN_SET); // 舵机上电
//        else
//            HAL_GPIO_WritePin(Servo_Power_12V_GPIO_Port, Servo_Power_12V_Pin, GPIO_PIN_RESET); // 舵机断电
        osDelay(10);
    }
}

float tempTest;
uint8_t writeAddress = 0;

extern int16_t count_peopleTeach;
uint8_t buzzerForPose;
sevroParameter paraSevro_t;
uint8_t countForCharging;
uint8_t buzzerForcharging;
void StartTaskLow(void const *argument)
{
	User_Init_LOW();
	
    for (;;) 
    {
		
//		if(count_peopleTeach > 0) count_peopleTeach--;
//		
//		if(count_peopleTeach == 0) personTeachFlag = 0;
		
		if(systemPowerOn == 0)
		{
			buzzerTimes = 3;
			osDelay(1500);
			systemPowerOn = 1;
		}
		paraSevro_t.headVerticalAng = getHorizontalAng();
		paraSevro_t.headHorizontalAng = getVerticalAng();
		
		if(USER_ADC.bat_charging == 1 && buzzerForcharging == 0)
		{
			buzzerTimes = 5;
			osDelay(2000);
			rgbTimes = 10;
			buzzerForcharging = 1;
		}
				
//		tempTest = ds18b20_get_temperature();
		
		MPU_PoseGet();
		poseNow = poseCheck();
		if(poseNow == 4)
		{
			buzzerWorking.buzzerForpose = 1;
			buzzerTimes = 6;
		}
		else buzzerWorking.buzzerForpose = 0;
		
		if(buzzerWorking.buzzerForcharge == 0 && buzzerWorking.buzzerForpose == 0) buzzerTimes = 0;
		fanSet(ioState.fan);
		
        osDelay(10);
    }
}

uint8_t testReset = 0;
int testangX;
uint8_t uartTest_7 = 0;
uint8_t data[3] = {0x00,0xbb,0xcc};
uint8_t testFlagg = 0;
int random_number_;
uint8_t ttttt1 = 0;
uint8_t ttttt2 = 0;
void StartTask05(void const * argument)
{
//	osDelay(3500);
  for(;;)
  {
	  if(testReset == 1)
	  {
		  SoftwareReset();
	  }
	  
	  if(powerState_t == Shutdown)
	  {
		  osDelay(2000);
		  if(USER_ADC.bat_volt > VOLTPOWERON)
		  {
			  HAL_GPIO_WritePin(upperComputerPower_5V_GPIO_Port,upperComputerPower_5V_Pin,GPIO_PIN_SET);//
			  HAL_GPIO_WritePin(Servo_Power_12V_GPIO_Port,Servo_Power_12V_Pin,GPIO_PIN_SET);// 舵机上电
		  }
		  powerState_t = powerIdle;
	  }
	  if(testFlagg == 1)
	  {
					// 随机生成数
		random_number_ = (rand()/10)%10 + 1;
		  testFlagg = 0;
	  }
//	  printf("APP1!\r\n");
	  //	RGB_Flash_InOneSecond(rgbTimes);
//	BUZZER_Flash_InOneSecond(buzzerTimes);
	  
	  // 测试
	  if(ttttt1 == 1) ledSet(LEDLIGHT_ON);
	  else ledSet(LEDLIGHT_OFF);
	  if(ttttt2 == 1) buzzerSet(BUZZER_ON);
	  else buzzerSet(BUZZER_OFF);
	  
    osDelay(1000);
  }
}
