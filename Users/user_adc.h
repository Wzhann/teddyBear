#ifndef _user_adc_h
#define _user_adc_h

#include "user_includes.h"

#define ADC1_CH_NUM	2

// ============ 改进：电池电压相关参数 ============
#define MINVOLT 10.0f      // 最低工作电压 (V)
#define MAXVOLT 12.4f     // 满电电压 (V)
#define VOLTPOWERON 11.4f // 上电最低电压 (V)

// ============ 改进：电量阈值定义 ============
#define BATTERY_FULL_PERCENT 100      // 满电百分比
#define BATTERY_LOW_PERCENT 20       // 低电量告警百分比
#define BATTERY_CRITICAL_PERCENT 10  // 临界电量百分比

// ============ 改进：充电相关参数 ============
#define CHARGING_CURRENT_THRESHOLD 0.1f  // 充电电流阈值 (A)，大于此值认为在充电
#define ZERO_CURRENT_OUTPUT_VOLTAGE 2.50f  // 电流传感器零偏输出电压 (V)
#define CURRENT_SENSOR_SENSITIVITY 0.1f   // 电流传感器灵敏度 (V/A)

#define H_USER_ADC &hadc1

typedef struct
{
	uint16_t adc1_dma_buf[ADC1_CH_NUM];  // DMA 缓冲（[0]=电压, [1]=电流）
	float bat_volt;                      // 电池电压 (V)
	int8_t bat_power;                     // 电池电量百分比 (0-100)
	float bat_current;                    // 电池电流 (A, 正数=充电, 负数=放电)
	uint8_t bat_charging;                 // 充电状态 (0=未充电, 1=充电中)
	float kalman_volt;                    // 卡尔曼滤波后的电压值
} USER_ADC_TYPE;

extern USER_ADC_TYPE USER_ADC;

// ============ 改进：函数声明 ============
void User_AdcInit(void);
void User_AdcDmaIRQHandler(void);
void User_AdcBatVoltGet(void);
void User_AdcBatCurrentGet(void);

// 新增函数
/**
 * @brief 电池电压卡尔曼滤波
 * @note 通过滤波平滑电压读数，减少抖动
 */
void User_AdcBatVoltFilter(void);

/**
 * @brief 分段线性计算电池电量
 * @param volt 电池电压 (V)
 * @return 电量百分比 (0-100)
 *
 * @note 锂电池放电曲线分段线性近似：
 *   12.4V ~ 12.0V: 100% ~ 85%
 *   12.0V ~ 11.4V: 85% ~ 50%
 *   11.4V ~ 10.8V: 50% ~ 20%
 *   10.8V ~ 10.0V: 20% ~ 0%
 */
int8_t User_AdcCalculateBatteryPercent(float volt);

/**
 * @brief 检测充电状态
 * @note 根据电流方向判断：正数=充电, 负数=放电
 */
void User_AdcCheckChargingStatus(void);

/**
 * @brief 电池状态综合检测
 * @note 同时更新电压、电量、电流、充电状态
 */
void User_AdcBatteryStatusUpdate(void);

/**
 * @brief 判断是否需要低电量告警
 * @return 0=不需要, 1=低电量告警, 2=临界电量告警
 */
uint8_t User_AdcCheckLowBattery(void);

#endif