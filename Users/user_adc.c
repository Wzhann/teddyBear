#include "user_adc.h"
#include "adc.h"

USER_ADC_TYPE USER_ADC;

uint8_t workflag = 0;

// ============ 改进：ADC 校准系数 ============
// 电压校准系数：将 ADC 值转换为实际电压
// 计算方式：实际电压 = ADC值 * (3.3 / 65535) * kvolt
// 其中 kvolt 由分压电阻决定，默认值需根据实际电路调整
static const float KVOLT_CALIBRATION = 6.077314f;
static const float KCURRENT_CALIBRATION = 1.0f;

// ============ 改进：电压卡尔曼滤波参数 ============
// 卡尔曼滤波用于平滑电压读数，减少波动
static float kalman_q = 0.01f;      // 过程噪声协方差
static float kalman_r = 0.1f;       // 测量噪声协方差
static float kalman_p = 1.0f;       // 估计误差协方差
static float kalman_k = 0.0f;       // 卡尔曼增益

// ============ 改进：采样参数 ============
#define VOLT_SAMPLE_COUNT 150       // 电压采样次数
#define CURRENT_SAMPLE_COUNT 20     // 电流采样次数
#define ADC_FULL_SCALE 65535.0f     // ADC 满量程（12位）
#define ADC_REF_VOLTAGE 3.3f        // ADC 参考电压 (V)

/**
 * @brief 电池电压卡尔曼滤波
 * @note 通过滤波平滑电压读数，减少抖动
 */
void User_AdcBatVoltFilter(void)
{
    // 计算当前测量值
    float measured = USER_ADC.adc1_dma_buf[0] * ADC_REF_VOLTAGE / ADC_FULL_SCALE * KVOLT_CALIBRATION;

    // 卡尔曼滤波算法
    // 1. 预测
    kalman_p = kalman_p + kalman_q;

    // 2. 计算卡尔曼增益
    kalman_k = kalman_p / (kalman_p + kalman_r);

    // 3. 更新估计值
    if (USER_ADC.kalman_volt < 1.0f) {
        // 首次测量，直接赋值
        USER_ADC.kalman_volt = measured;
    } else {
        USER_ADC.kalman_volt = USER_ADC.kalman_volt + kalman_k * (measured - USER_ADC.kalman_volt);
    }

    // 4. 更新估计误差协方差
    kalman_p = (1 - kalman_k) * kalman_p;
}

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
int8_t User_AdcCalculateBatteryPercent(float volt)
{
    int8_t percent;

    // 限制电压范围
    if (volt >= MAXVOLT) {
        return BATTERY_FULL_PERCENT;
    }
    if (volt <= MINVOLT) {
        return 0;
    }

    // 分段线性插值
    if (volt >= 12.0f) {
        // 高电压区：12.0V ~ 12.4V → 85% ~ 100%
        // 斜率 = (100-85)/(12.4-12.0) = 37.5
        percent = 85 + (int8_t)((volt - 12.0f) * 37.5f);
    } else if (volt >= 11.4f) {
        // 中电压区：11.4V ~ 12.0V → 50% ~ 85%
        // 斜率 = (85-50)/(12.0-11.4) = 58.33
        percent = 50 + (int8_t)((volt - 11.4f) * 58.33f);
    } else if (volt >= 10.8f) {
        // 中低电压区：10.8V ~ 11.4V → 20% ~ 50%
        // 斜率 = (50-20)/(11.4-10.8) = 50
        percent = 20 + (int8_t)((volt - 10.8f) * 50.0f);
    } else {
        // 低电压区：10.0V ~ 10.8V → 0% ~ 20%
        // 斜率 = (20-0)/(10.8-10.0) = 25
        percent = (int8_t)((volt - MINVOLT) * 25.0f);
    }

    // 限制范围
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;

    return percent;
}

/**
 * @brief 检测充电状态
 * @note 根据电流方向判断：正数=充电, 负数=放电
 */
void User_AdcCheckChargingStatus(void)
{
    if (USER_ADC.bat_current > CHARGING_CURRENT_THRESHOLD) {
        USER_ADC.bat_charging = 1;
    } else {
        USER_ADC.bat_charging = 0;
    }
}

/**
 * @brief 电池状态综合检测
 * @note 只在空闲时更新电压和电量，动作中保持上次值
 *       电流始终读取（用于充电判断）
 */
void User_AdcBatteryStatusUpdate(void)
{
    // 电流始终读取（充电判断需要）
    User_AdcBatCurrentGet();

    // 充电状态判断
    User_AdcCheckChargingStatus();

    // 只在空闲时更新电压和电量，避免动作中大电流拉低电压导致电量不准
    if (ActionNow == IDLE) {
        User_AdcBatVoltGet();
    }
}

/**
 * @brief 判断是否需要低电量告警
 * @return 0=不需要, 1=低电量告警(≤20%), 2=临界电量告警(≤10%)
 */
uint8_t User_AdcCheckLowBattery(void)
{
    if (USER_ADC.bat_power <= BATTERY_CRITICAL_PERCENT) {
        return 2;  // 临界告警
    } else if (USER_ADC.bat_power <= BATTERY_LOW_PERCENT) {
        return 1;  // 低电量告警
    } else {
        return 0;  // 正常
    }
}

// ============================================================
// 保留的原函数（内部使用）
// ============================================================

void User_AdcBatVoltGet(void)
{
	static uint8_t cnt = 0;
	static float sum = 0;

	cnt++;

	if (cnt <= VOLT_SAMPLE_COUNT) {
		sum += USER_ADC.adc1_dma_buf[0] * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
	}

	if (cnt == VOLT_SAMPLE_COUNT) {
		// 计算平均电压
		USER_ADC.bat_volt = sum / VOLT_SAMPLE_COUNT * KVOLT_CALIBRATION;

		// 限制电压范围
		if (USER_ADC.bat_volt > MAXVOLT) USER_ADC.bat_volt = MAXVOLT;
		if (USER_ADC.bat_volt < MINVOLT) USER_ADC.bat_volt = MINVOLT;

		// 使用改进的分段线性计算电量
		USER_ADC.bat_power = User_AdcCalculateBatteryPercent(USER_ADC.bat_volt);

		// 卡尔曼滤波
		User_AdcBatVoltFilter();

		// 重置采样
		cnt = 0;
		sum = 0;
	}
}

void User_AdcBatCurrentGet(void)
{
	static uint8_t cnt = 0;
	static float sum = 0;

	cnt++;

	if (cnt <= CURRENT_SAMPLE_COUNT) {
		sum += USER_ADC.adc1_dma_buf[1] * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
	}

	if (cnt == CURRENT_SAMPLE_COUNT) {
		// 计算平均电压
		float avg_voltage = sum / CURRENT_SAMPLE_COUNT;

		// 转换为电流：(测量电压 - 零偏) / 灵敏度
		USER_ADC.bat_current = (avg_voltage - ZERO_CURRENT_OUTPUT_VOLTAGE) / CURRENT_SENSOR_SENSITIVITY * KCURRENT_CALIBRATION;

		// 限制电流范围（-10A ~ 10A）
		if (USER_ADC.bat_current > 10.0f) USER_ADC.bat_current = 10.0f;
		if (USER_ADC.bat_current < -10.0f) USER_ADC.bat_current = -10.0f;

		// 重置采样
		cnt = 0;
		sum = 0;
	}
}

void User_AdcInit(void)
{
	// 启动 DMA 采集
	HAL_ADC_Start_DMA(H_USER_ADC, (uint32_t*)USER_ADC.adc1_dma_buf, ADC1_CH_NUM);
}