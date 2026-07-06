#include <math.h>
#include "user_imu_i2c.h"
#include "cmsis_os.h"
#include "user_iic.h"
#include "user_servo.h"  // 新增：用于读取舵机角度
#include "user_tasks.h"    // 新增：用于访问 motion_last
//#include "inv_mpu.h"

#define PI 3.14159265358979323846
#define RAD_TO_DEG 57.295779513082320876798154814105

MPU6050_t MPU6050;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};


uint8_t MPU6050_ReadReg(uint8_t reg_addr)
{
	uint8_t data;
	User_IIC_Start();  //1
	User_IIC_SendByte(MPU6050_WRITE_ADDR);  //2
	User_IIC_WaitAck();  //3
	User_IIC_SendByte(reg_addr);  //4
	User_IIC_WaitAck();  //5

	User_IIC_Start();  //6
	User_IIC_SendByte(MPU6050_READ_ADDR);  //7
	User_IIC_WaitAck();  //8
	data = User_IIC_ReadByte(0);  //9

	User_IIC_Stop();  //11

	return data;
}

uint8_t MPU_Read_Len(uint8_t dev_addr,uint8_t reg_addr,uint8_t len,uint8_t *buf)
{
	User_IIC_Start();
	User_IIC_SendByte(MPU6050_WRITE_ADDR);//发送器件地址+写位
	User_IIC_WaitAck();

  User_IIC_SendByte(reg_addr);	//写寄存器地址
  User_IIC_WaitAck();		//等待从机应答
  User_IIC_Start();
	User_IIC_SendByte(MPU6050_READ_ADDR);//发送器件地址+读位
   User_IIC_WaitAck();		//等待从机应答
	while(len)
	{
		if(len==1)*buf=User_IIC_ReadByte(0);//读最后一个字节,发送nACK
		else *buf=User_IIC_ReadByte(1);		//读非最后一个字节,发送ACK
		len--;
		buf++;
	}
  User_IIC_Stop();	//产生一个停止信号
	return 0;

}

uint8_t MPU_Write_Len(uint8_t dev_addr,uint8_t reg_addr,uint8_t len,uint8_t *buf)
{
	uint8_t i;
  User_IIC_Start();
	User_IIC_SendByte(MPU6050_WRITE_ADDR);//发送器件地址+写位
	User_IIC_WaitAck();
  User_IIC_SendByte(reg_addr);	//写寄存器地址
	User_IIC_WaitAck();		//等待从机应答
	for(i=0;i<len;i++)
	{
		User_IIC_SendByte(buf[i]);		//发送数据
		User_IIC_WaitAck();		//等待从机应答
	}
  User_IIC_Stop();
	return 0;
}

void MPU6050_WriteReg(uint8_t reg_addr,uint8_t data)
{
	User_IIC_Start();  //1
	User_IIC_SendByte(MPU6050_WRITE_ADDR);  //2
	User_IIC_WaitAck();  //3
	User_IIC_SendByte(reg_addr);  //4
	User_IIC_WaitAck();  //5
	User_IIC_SendByte(data);  //6
	User_IIC_WaitAck();  //7
	User_IIC_Stop();  //8
}

void MPU6050_Init(void)
{
	User_IIC_Init();
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x00);        //解除睡眠模式
//	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x07);
//	MPU6050_WriteReg(MPU6050_CONFIG,0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x00);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x00);

}

void MPU6050_GetAccData(void)
{
	uint16_t dataH,dataL;
	//AccX
	dataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	MPU6050.Accel_X_RAW = (dataH << 8)    | dataL;
	//AccY
	dataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	MPU6050.Accel_Y_RAW = (dataH << 8)    | dataL;
	//AccZ
	dataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	MPU6050.Accel_Z_RAW = (dataH << 8)    | dataL;

	MPU6050.Ax = MPU6050.Accel_X_RAW / 16384.0;
	MPU6050.Ay = MPU6050.Accel_Y_RAW / 16384.0;
	MPU6050.Az = MPU6050.Accel_Z_RAW / 16384.0;
}

void MPU6050_GetData(void)
{
	uint16_t dataH,dataL;
	//AccX
	dataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	MPU6050.Accel_X_RAW = (dataH << 8)    | dataL;
	//AccY
	dataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	MPU6050.Accel_Y_RAW = (dataH << 8)    | dataL;
	//AccZ
	dataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	MPU6050.Accel_Z_RAW = (dataH << 8)    | dataL;
	//GyroX
	dataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	MPU6050.Gyro_X_RAW = (dataH << 8)    | dataL;
	//GyroY
	dataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	MPU6050.Gyro_Y_RAW = (dataH << 8)    | dataL;
	//GyroZ
	dataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	MPU6050.Gyro_Z_RAW = (dataH << 8)    | dataL;
	//TEMP
	dataH = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);
	dataL = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);
	MPU6050.temper_RAW = (dataH << 8)    | dataL;

	MPU6050.Ax = MPU6050.Accel_X_RAW / 16384.0;
	MPU6050.Ay = MPU6050.Accel_Y_RAW / 16384.0;
	MPU6050.Az = MPU6050.Accel_Z_RAW / 16384.0;

	MPU6050.Gx = MPU6050.Gyro_X_RAW / 131.0;
	MPU6050.Gy = MPU6050.Gyro_Y_RAW / 131.0;
	MPU6050.Gz = MPU6050.Gyro_Z_RAW / 131.0;

	MPU6050.temperature = MPU6050.temper_RAW/340.0f + 36.53;

}

void MPU6050_GetData_All(void)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
		MPU_Read_Len(MPU6050_I2C_ADDR,MPU6050_ACCEL_XOUT_H,14,Rec_Data);

    MPU6050.Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    MPU6050.Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    MPU6050.Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    MPU6050.Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    MPU6050.Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    MPU6050.Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    MPU6050.Ax = MPU6050.Accel_X_RAW / 16384.0;
    MPU6050.Ay = MPU6050.Accel_Y_RAW / 16384.0;
    MPU6050.Az = MPU6050.Accel_Z_RAW / 16384.0f;

    MPU6050.temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    MPU6050.Gx = MPU6050.Gyro_X_RAW / 131.0;
    MPU6050.Gy = MPU6050.Gyro_Y_RAW / 131.0;
    MPU6050.Gz = MPU6050.Gyro_Z_RAW / 131.0;
}

// 根据加速度计计算pitch角（俯仰角），单位为度
double calculatePitch(double ax, double ay, double az)
{
    double pitch = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0 / PI); // 转换为度
    return pitch;
}

// 根据加速度计计算roll角（横滚角），单位为度
double calculateRoll(double ax, double ay, double az)
{
    double roll = atan2(ay, az) * (180.0 / PI); // 转换为度
    return roll;
}

// 根据加速度计计算yaw角（偏航角），单位为度
double calculateYaw(double ax, double ay)
{
    double yaw = atan2(ax, ay) * RAD_TO_DEG; // 转换为度
    return yaw;
}
// 位姿估计函数
void estimatePose(void)
{
  // 计算pitch角和roll角
  MPU6050.pitch = calculatePitch(MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
  MPU6050.roll = calculateRoll(MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
	MPU6050.yaw = calculateYaw(MPU6050.Ax, MPU6050.Ay);
}

extern uint8_t actionPoseLast;
extern Motion_t *motion_last;
float startPose = 0;

// ============================================================
// 改进：基于舵机角度的姿态判断
// ============================================================

/**
 * @brief 通过舵机角度判断坐/站态
 * @param knee_angle_left 左腿膝盖角度（servo 4，传入 0 则不使用）
 * @param knee_angle_right 右腿膝盖角度（servo 9，传入 0 则不使用）
 * @return POSE_SITTING 或 POSE_STANDING
 *
 * @note 左右膝盖独立判断，任一判定坐姿即返回坐姿：
 *   - 左膝盖(servo4): 站≈10,  坐≈1003  → >500  为坐姿
 *   - 右膝盖(servo9): 站≈-7,  坐≈-1017 → <-500 为坐姿
 */
uint8_t poseCheckByServoAngle(int16_t knee_angle_left, int16_t knee_angle_right)
{
    // 左膝盖: 坐姿时角度很大（正数）
    if (knee_angle_left != 0 && knee_angle_left > KNEE_LEFT_THRESHOLD) {
        return POSE_SITTING;
    }
    // 右膝盖: 坐姿时角度很大（负数）
    if (knee_angle_right != 0 && knee_angle_right < KNEE_RIGHT_THRESHOLD) {
        return POSE_SITTING;
    }
    // 两个膝盖都不满足坐姿条件 → 站姿
    return POSE_STANDING;
}

/**
 * @brief 综合判断姿态（IMU + 舵机角度）
 * @return POSE_SITTING、POSE_STANDING 或 POSE_LYING
 *
 * @note 适用于上电初始化时的准确姿态检测
 * 1. 先用 IMU 判断是否趴下
 * 2. 如果是直立姿态，再用舵机角度区分坐/站
 */
uint8_t poseCheckComprehensive(void)
{
    float imu_roll = MPU6050.roll;

    // 1. 先通过 IMU 判断是否趴下（趴态 IMU 角度范围不同）
    if (imu_roll >= LYING_RANGE_MIN && imu_roll <= LYING_RANGE_MAX) {
        return POSE_LYING;
    }

    // 2. 如果 IMU 角度在直立范围内，用舵机角度区分坐/站
    if (imu_roll >= UPRIGHT_RANGE_MIN && imu_roll <= UPRIGHT_RANGE_MAX) {
        // 读取两个膝盖舵机的当前角度
        int16_t knee_left = SERVO[KNEE_SERVO_ID_LEFT].pos_read;
        int16_t knee_right = SERVO[KNEE_SERVO_ID_RIGHT].pos_read;

        return poseCheckByServoAngle(knee_left, knee_right);
    }

    // 3. 异常情况，默认坐态
    return POSE_SITTING;
}

// ============================================================
// 改进后的 userImuInit（上电初始化）
// ============================================================

/**
 * @brief 上电 IMU 初始化与姿态检测（改进版）
 * @note 使用综合判断（IMU + 舵机角度）准确检测上电姿态
 */
void userImuInit()
{
	float imu_roll_avg = 0;
	float sum_pose = 0;

	// 1. 预热 IMU，获取稳定数据
	for(int i = 0; i < 50; i++)
	{
		MPU6050_GetAccData();
		estimatePose();
		osDelay(1);  // 短暂延时
	}

	// 2. 采样 20 次取平均
	for(int j = 0; j < 20; j++)
	{
		MPU6050_GetAccData();
		estimatePose();
		sum_pose += MPU6050.roll;
		osDelay(2);  // 短暂延时
	}
	imu_roll_avg = sum_pose / 20;

	// 3. 将 IMU 角度存入全局变量供后续使用
	startPose = imu_roll_avg;

	// 4. 使用综合判断（IMU + 舵机角度）确定上电姿态
	// 这是关键改进点：不再默认为坐态，而是准确判断
	actionPoseLast = poseCheckComprehensive();
}

void MPU_PoseGet(void)
{
	MPU6050_GetAccData();
	estimatePose();
}

/**
 * @brief 运行时姿态检测（保持原逻辑）
 * @note 运行时根据上一个动作的结束姿态推断当前姿态
 */
uint8_t poseCheck(void)
{
	startPose = MPU6050.roll;

	// 坐和站使用相同的陀螺仪范围（直立姿态），根据上一个动作的姿态判断当前是坐还是站
	if (startPose >= UPRIGHT_RANGE_MIN && startPose <= UPRIGHT_RANGE_MAX)
	{
		if (motion_last->poseend == POSE_STANDING)
			return POSE_STANDING;
		else
			return POSE_SITTING;
	}
	else if (startPose >= LYING_RANGE_MIN && startPose <= LYING_RANGE_MAX)
	{
		return POSE_LYING;
	}
	else
		return POSE_error;
}