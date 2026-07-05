#ifndef _user_imu_i2c_h
#define _user_imu_i2c_h
#include "Action_library.h"

#include <stdint.h>


#define MPU6050_I2C_ADDR    	(0x68)
#define MPU6050_WRITE_ADDR    (((MPU6050_I2C_ADDR) << 1) | 0x00)
#define MPU6050_READ_ADDR    	(((MPU6050_I2C_ADDR) << 1) | 0x01)

#define    MPU6050_SMPLRT_DIV      0x19
#define    MPU6050_CONFIG          0x1A
#define    MPU6050_GYRO_CONFIG     0x1B
#define    MPU6050_ACCEL_CONFIG    0x1C

#define    MPU6050_ACCEL_XOUT_H    0x3B
#define    MPU6050_ACCEL_XOUT_L    0x3C
#define    MPU6050_ACCEL_YOUT_H    0x3D
#define    MPU6050_ACCEL_YOUT_L    0x3E
#define    MPU6050_ACCEL_ZOUT_H    0x3F
#define    MPU6050_ACCEL_ZOUT_L    0x40
#define    MPU6050_TEMP_OUT_H      0x41
#define    MPU6050_TEMP_OUT_L      0x42
#define    MPU6050_GYRO_XOUT_H     0x43
#define    MPU6050_GYRO_XOUT_L     0x44
#define    MPU6050_GYRO_YOUT_H     0x45
#define    MPU6050_GYRO_YOUT_L     0x46
#define    MPU6050_GYRO_ZOUT_H     0x47
#define    MPU6050_GYRO_ZOUT_L     0x48

#define    MPU6050_PWR_MGMT_1      0x6B
#define    MPU6050_PWR_MGMT_2      0x6C
#define    MPU6050_WHO_AM_I        0x75



// ============ 改进：姿态检测角度范围 ============
// 直立姿态范围（IMU roll 角，用于判断是否直立）
#define UPRIGHT_RANGE_MIN 48
#define UPRIGHT_RANGE_MAX 120
// 趴姿态范围
#define LYING_RANGE_MIN -40
#define LYING_RANGE_MAX 48

// ============ 改进：舵机角度阈值（用于区分坐/站） ============
// 通过膝关节角度区分：
//   站态时膝盖接近直立（servo[3] ≈ -626, servo[9] ≈ -7）
//   坐态时膝盖大角度弯曲（servo[3] ≈ -714, servo[9] ≈ -1017）
#define KNEE_ANGLE_THRESHOLD -500    // 膝盖角度阈值，小于此值认为是坐姿
#define KNEE_SERVO_ID_LEFT 3         // 左腿膝盖舵机 ID
#define KNEE_SERVO_ID_RIGHT 9        // 右腿膝盖舵机 ID

// MPU6050 structure
typedef struct
{
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	float Ax;
	float Ay;
	float Az;

	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	float Gx;
	float Gy;
	float Gz;
	float pitch;
	float roll;
	float yaw;

	int16_t temper_RAW;
	float temperature;

	float KalmanAngleX;
	float KalmanAngleY;

} MPU6050_t;

// Kalman structure
typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

extern MPU6050_t MPU6050;

void MPU6050_Init(void);
void MPU6050_GetData(void);
void MPU6050_GetData_All(void);
uint8_t MPU_Read_Len(uint8_t dev_addr,uint8_t reg_addr,uint8_t len,uint8_t *buf);
uint8_t MPU_Write_Len(uint8_t dev_addr,uint8_t reg_addr,uint8_t len,uint8_t *buf);

void estimatePose(void);
void userImuInit(void);
uint8_t poseCheck(void);
void MPU_PoseGet(void);

// ============ 新增函数：基于舵机角度的姿态判断 ============
/**
 * @brief 通过舵机角度判断坐/站态
 * @note 适用于 IMU 无法区分坐/站的场景，特别是上电初始化
 * @param knee_angle_left 左腿膝盖角度（传入 0 则不使用）
 * @param knee_angle_right 右腿膝盖角度（传入 0 则不使用）
 * @return POSE_SITTING 或 POSE_STANDING
 */
uint8_t poseCheckByServoAngle(int16_t knee_angle_left, int16_t knee_angle_right);

/**
 * @brief 综合判断姿态（IMU + 舵机角度）
 * @note 上电时使用此函数代替 poseCheck() 可更准确区分坐/站
 * @return POSE_SITTING、POSE_STANDING 或 POSE_LYING
 */
uint8_t poseCheckComprehensive(void);

#endif