#include <math.h>
#include "user_imu_i2c.h"
#include "cmsis_os.h"
#include "user_iic.h"
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
	User_IIC_SendByte(MPU6050_WRITE_ADDR);//����������ַ+д����	
	User_IIC_WaitAck();
	
  User_IIC_SendByte(reg_addr);	//д�Ĵ�����ַ
  User_IIC_WaitAck();		//�ȴ�Ӧ��
  User_IIC_Start();
	User_IIC_SendByte(MPU6050_READ_ADDR);//����������ַ+������	
   User_IIC_WaitAck();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=User_IIC_ReadByte(0);//������,����nACK 
		else *buf=User_IIC_ReadByte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
  User_IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}

uint8_t MPU_Write_Len(uint8_t dev_addr,uint8_t reg_addr,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
  User_IIC_Start(); 
	User_IIC_SendByte(MPU6050_WRITE_ADDR);//����������ַ+д����	
	User_IIC_WaitAck();
  User_IIC_SendByte(reg_addr);	//д�Ĵ�����ַ
  User_IIC_WaitAck();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		User_IIC_SendByte(buf[i]);	//��������
		User_IIC_WaitAck();		//�ȴ�Ӧ��	
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
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x00);        //�ر�˯��ģʽ
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

// ������ٶ�������ˮƽ��ļнǣ������ǣ�
double calculatePitch(double ax, double ay, double az)
{
    double pitch = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0 / PI); // ת��Ϊ�Ƕ�
    return pitch;
}

// ������ٶ�������ˮƽ��ļнǣ�����ǣ�
double calculateRoll(double ax, double ay, double az)
{
    double roll = atan2(ay, az) * (180.0 / PI); // ת��Ϊ�Ƕ�
    return roll;
}

// ������ٶ�������ˮƽ��ļнǣ�ƫ���ǣ�
double calculateYaw(double ax, double ay)
{
    double yaw = atan2(ax, ay) * RAD_TO_DEG; // ת��Ϊ�Ƕ�
    return yaw;
}
// ��̬���ƺ���
void estimatePose(void)
{
  // ���㸩���Ǻͺ����
  MPU6050.pitch = calculatePitch(MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
  MPU6050.roll = calculateRoll(MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
	MPU6050.yaw = calculateYaw(MPU6050.Ax, MPU6050.Ay);
}

extern uint8_t actionPoseLast;
//float startPose;
void userImuInit()
{
	float startPose = 0;
	float sum_pose = 0;
	for(int i =0;i< 50;i++)
	{
		MPU6050_GetAccData();
		estimatePose();//����ǰ��ٸ�
	}
	
	for(int j =0;j< 20;j++)
	{
		MPU6050_GetAccData();
		estimatePose();
		sum_pose += MPU6050.roll;
	}
	startPose = sum_pose / 20;
	
	// 坐和站使用相同的陀螺仪范围（直立姿态），默认为坐
	if (startPose >= UPRIGHT_RANGE_MIN && startPose <= UPRIGHT_RANGE_MAX)
	{
		actionPoseLast = POSE_SITTING; // 默认坐姿
	}
	else if (startPose >= LYING_RANGE_MIN && startPose <= LYING_RANGE_MAX)
	{
		actionPoseLast = POSE_LYING;
	}
	else
	{
		actionPoseLast = POSE_SITTING; // 异常情况默认坐姿
	}
}



void MPU_PoseGet(void)
{
	MPU6050_GetAccData();
	estimatePose();
}
extern Motion_t *motion_last;
float startPose = 0;
uint8_t poseCheck(void)
{
//	float sum_pose = 0;
//	for(int j =0;j< 100;j++)
//	{
//		MPU6050_GetAccData();
//		estimatePose();
//		sum_pose += MPU6050.roll;
//	}
//	startPose = sum_pose / 100;
	
	
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
//	mpu_dmp_get_data(&MPU6050.pitch,&MPU6050.roll,&MPU6050.yaw);
	return POSE_error; 
}
