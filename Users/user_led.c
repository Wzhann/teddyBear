#include "user_led.h"

LED_TYPEDEF LED[3];
LEDA_TYPEDEF LEDA; 

void User_LedInit(void)
{
	LED_R_OFF;
	LED_G_OFF;
	LED_B_OFF;
}

void User_LedShowSet(uint8_t rgb,uint8_t freq,uint8_t time,uint8_t en)
{	
	LED[rgb].flash_freq = freq;
	LED[rgb].flash_time = time;
	LED[rgb].flash_en = en;  
}

void User_LedEn(uint8_t rgpb,uint8_t en)
{
	switch(rgpb)
	{
		case 0: en?LED_R_ON:LED_R_OFF;break;
		case 1: en?LED_G_ON:LED_G_OFF;break;
		case 2: en?LED_B_ON:LED_B_OFF;break;
		default:break;
	}
}

void User_LedTaskShow(void)//
{
	uint8_t i = 0;
 static uint16_t cnt = 0;
	
	for(i=0;i<=2;i++)
	{
		if(LED[i].flash_en == 0)
		{
			User_LedEn(i,0);
		}
		else if(LED[i].flash_freq == 0)
		{
			User_LedEn(i,1);
		}
		else
		{
			LED[i].on_cnt = 1000/_LED_TASK_TIME/LED[i].flash_freq/2;

			LED[i].task_cnt++;
			if(LED[i].remain_time) 
			{
				if(LED[i].task_cnt <= LED[i].on_cnt)
				{
					User_LedEn(i,1);
				}
				else if( LED[i].task_cnt <= 2*LED[i].on_cnt)
				{
					User_LedEn(i,0);			
				}
				if(LED[i].task_cnt == 2*LED[i].on_cnt)
				{
					LED[i].remain_time--;
					LED[i].task_cnt = 0;
				}		
			}
			else
			{
				User_LedEn(i,0);	

				LED[i].task_cnt++;					
				if(LED[i].task_cnt == 2000/_LED_TASK_TIME)
				{
					LED[i].task_cnt = 0;
					LED[i].remain_time = LED[i].flash_time;
				}			
			}
		}	
	}
	cnt++;
	if(USER_ADC.bat_volt > 11.1f)
	{
		if(cnt == 1000/_LED_TASK_TIME)
			User_LedaSetColor(0,0,255);
		else if(cnt == 2000/_LED_TASK_TIME)
			User_LedaSetColor(0,255,0);
		else if(cnt == 3000/_LED_TASK_TIME)
			User_LedaSetColor(255,0,0);
		else if(cnt == 4000/_LED_TASK_TIME)
			User_LedaSetColor(255,255,0);
		else if(cnt == 5000/_LED_TASK_TIME)
			User_LedaSetColor(255,0,255);
		else if(cnt == 6000/_LED_TASK_TIME)
			User_LedaSetColor(0,255,255);
		else if(cnt == 7000/_LED_TASK_TIME)
		{
			User_LedaSetColor(255,255,255);
			cnt = 0;
		}		
	}
	else
	{
		if(cnt == 250/_LED_TASK_TIME)
			User_LedaSetColor(255,0,0);
		else if(cnt >= 500/_LED_TASK_TIME)
		{
			User_LedaSetColor(0,0,0);
			cnt = 0;
		}		
	}
}



void User_LedDelayNs(uint16_t ns)
{
	uint16_t i = ns;
	while(i--)
	{
		__NOP();
	}	
}

uint8_t User_LedaSetColor(uint8_t r,uint8_t g,uint8_t b)
{
	uint8_t i = 0;
	if(LEDA.send_busy == 0)
	{
		LEDA.send_busy = 1;
		__disable_irq();
		LEDA.grb = ((uint32_t)g<<16) | ((uint32_t)r<<8) | ((uint32_t)b<<0);
		for(i=0;i<24;i++)
		{
			if(LEDA.grb & (1<<(23-i)))
			{
				LED_A_H;
				User_LedDelayNs(15);//580ns-1us
				LED_A_L;
				User_LedDelayNs(15);
			}
			else
			{
				LED_A_H;
				User_LedDelayNs(4);//220-380ns
				LED_A_L;
				User_LedDelayNs(15);
			}
		}
		__enable_irq();
		LEDA.send_busy = 0;
		return 1;
	}
	else
		return 0;
}










