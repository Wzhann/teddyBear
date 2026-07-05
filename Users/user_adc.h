#ifndef _user_adc_h
#define _user_adc_h

#include "user_includes.h"

#define ADC1_CH_NUM	2


#define MINVOLT 10
#define MAXVOLT 12.4
#define VOLTPOWERON 11.4

#define H_USER_ADC &hadc1

#define zeroCurrentOutputVoltage 2.50f
#define Sensitivity 0.1f

typedef struct
{
	uint16_t adc1_dma_buf[ADC1_CH_NUM];	
	float bat_volt;
	int8_t bat_power;	
	float bat_current;
	uint8_t bat_charging;
}USER_ADC_TYPE;

extern USER_ADC_TYPE USER_ADC; 

void User_AdcInit(void);
void User_AdcDmaIRQHandler(void);
void User_AdcBatVoltGet(void);
void User_AdcBatCurrentGet(void);
#endif

