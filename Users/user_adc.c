#include "user_adc.h"
#include "adc.h"

USER_ADC_TYPE USER_ADC; 

uint8_t workflag = 0;
float kvolt = 6.077314;
float kcurrent = 1;

//void User_AdcBatVoltGet(void)
//{
////	HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
//	USER_ADC.adc3_dma_buf[0] = HAL_ADC_GetValue(&hadc3);
//	static uint8_t cnt = 0;
//	static float sum = 0;
//	
//	cnt++;
//	if(cnt <= 100)
//	{
//		sum += USER_ADC.adc3_dma_buf[0]*3.3/65535;
//	}
//	if(cnt == 100)
//	{
//		//workflag = 1;
//		USER_ADC.bat_volt = sum/100*kvolt;
//		
//		USER_ADC.bat_power =(int8_t)( (USER_ADC.bat_volt - MINVOLT) / (MAXVOLT-MINVOLT) * 100);
//		if(USER_ADC.bat_power >= 100) USER_ADC.bat_power = 100;
//		if(USER_ADC.bat_power <= 0) USER_ADC.bat_power = 0;
//		cnt = 0;
//		sum = 0;
//	}
//	HAL_ADC_Start(&hadc3);
//}

void User_AdcBatVoltGet(void)
{
//	USER_ADC.adc1_dma_buf[0] = HAL_ADC_GetValue(H_USER_ADC);
	static uint8_t cnt = 0;
	static float sum = 0;
	cnt ++;
	if(cnt <= 150)
	{
		sum += USER_ADC.adc1_dma_buf[0]*3.3/65535;
	}
	if(cnt == 150)
	{
		USER_ADC.bat_volt = sum/150*kvolt;
//		if(USER_ADC.bat_volt < SERVO[1].volt_read * 0.1) USER_ADC.bat_volt = SERVO[1].volt_read * 0.1;
		USER_ADC.bat_power =(int8_t)( (float)((float)(USER_ADC.bat_volt - MINVOLT) / (float)(MAXVOLT-MINVOLT)) * 100);
		if(USER_ADC.bat_power >= 100) USER_ADC.bat_power = 100;
		if(USER_ADC.bat_power <= 0) USER_ADC.bat_power = 0;
		cnt = 0;
		sum = 0;
	}
//	HAL_ADC_Start(H_USER_ADC);
}

void User_AdcBatCurrentGet(void)
{
//	USER_ADC.adc2_dma_buf[0] = HAL_ADC_GetValue(CURRENT_YSERADC);
	static uint8_t cnt = 0;
	static float sum = 0;
	cnt ++;
	if(cnt <= 20)
	{
		sum += USER_ADC.adc1_dma_buf[1]*3.3/65535;
	}
	if(cnt == 20)
	{
		USER_ADC.bat_current = (sum/20 - zeroCurrentOutputVoltage)/Sensitivity * kcurrent;
		cnt = 0;
		sum = 0;
	}
//	HAL_ADC_Start(CURRENT_YSERADC);
}


void User_AdcInit(void)
{
//	HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);//默认是开启DMA中断的

//	HAL_ADC_Start(&hadc3);
	HAL_ADCEx_Calibration_Start(H_USER_ADC,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//F4无需自校准！！！
//	HAL_ADCEx_Calibration_Start(&hadc1);

	HAL_ADC_Start_DMA(H_USER_ADC, (uint32_t*)USER_ADC.adc1_dma_buf, ADC1_CH_NUM);
//	HAL_ADC_Start_DMA(CURRENT_YSERADC, (uint32_t*)USER_ADC.adc2_dma_buf, ADC2_CH_NUM);
//	
//	HAL_ADCEx_Calibration_Start(VOLTAGE_UAERADC,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);//F4无需自校准！！！
//	HAL_ADC_Start(VOLTAGE_UAERADC);
//	HAL_ADCEx_Calibration_Start(CURRENT_YSERADC,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);//F4无需自校准！！！
//	HAL_ADC_Start(CURRENT_YSERADC);
	
}


void User_AdcDmaIRQHandler(void)
{

}


