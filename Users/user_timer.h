#ifndef _user_timer_h
#define _user_timer_h

#include "user_includes.h"


#define USER_htim_servo	htim4
#define USER_htim_teach   htim6
#define USER_TIM_SERVO	TIM4

#define ACTIONTIMESTEP 170

typedef struct
{
	uint8_t i;

}TIMER_TYPEDEF;
extern TIMER_TYPEDEF TIMER;


void User_TimerInit(void);
void User_TeachTimerInit(void);
void User_TimerServoIRQ(void);
void User_TimerTeachIRQ(void);
void User_TimerActionIRQ(void);

#endif










