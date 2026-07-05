//#include "main.h"
//#include "Myiic_IMU.h"
// #include "DS18B20.h"

/////**
////  * @brief  ?????
////  * @param  xus ????,??:0~233015
////  * @retval ?
////  */
////void delay_us(uint32_t xus)
////{
////	SysTick->LOAD = 72 * xus;				//????????
////	SysTick->VAL = 0x00;					//???????
////	SysTick->CTRL = 0x00000005;				//??????HCLK,?????
////	while(!(SysTick->CTRL & 0x00010000));	//?????0
////	SysTick->CTRL = 0x00000004;				//?????
////}
//// 
///**
//  * @brief  ?????
//  * @param  xms ????,??:0~4294967295
//  * @retval ?
//  */
//void Delay_ms(uint32_t xms)
//{
//	while(xms--)
//	{
//		delay_us(1000);
//	}
//}
// 
///**
//  * @brief  ????
//  * @param  xs ????,??:0~4294967295
//  * @retval ?
//  */
//void Delay_s(uint32_t xs)
//{
//	while(xs--)
//	{
//		Delay_ms(1000);
//	}
//} 
// 
// 
//void MyI2C_W_SCL(uint8_t BitValue)
//{
////	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
//	HAL_GPIO_WritePin(I2C_SCL_GPIO_Port,I2C_SCL_Pin,BitValue);
//	delay_us(10);
//}
// 
//void MyI2C_W_SDA(uint8_t BitValue)
//{
////	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
//	HAL_GPIO_WritePin(I2C_SDA_GPIO_Port,I2C_SDA_Pin,BitValue);
//	delay_us(10);
//}
// 
//uint8_t MyI2C_R_SDA(void)
//{
//	uint8_t BitValue;
////	BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
//	BitValue = HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin);
//	delay_us(10);
//	return BitValue;
//}
// 
//void MyI2C_Start(void)
//{
//	MyI2C_W_SDA(1);
//	MyI2C_W_SCL(1);
//	MyI2C_W_SDA(0);
//	MyI2C_W_SCL(0);
//}
// 
//void MyI2C_Stop(void)
//{
//	MyI2C_W_SDA(0);
//	MyI2C_W_SCL(1);
//	MyI2C_W_SDA(1);
//}
// 
//void MyI2C_SendByte(uint8_t Byte)
//{
//	uint8_t i;
//	for (i = 0; i < 8; i ++)
//	{
//		MyI2C_W_SDA(Byte & (0x80 >> i));
//		MyI2C_W_SCL(1);
//		MyI2C_W_SCL(0);
//	}
//}
// 
//uint8_t MyI2C_ReceiveByte(void)
//{
//	uint8_t i, Byte = 0x00;
//	MyI2C_W_SDA(1);
//	for (i = 0; i < 8; i ++)
//	{
//		MyI2C_W_SCL(1);
//		if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}
//		MyI2C_W_SCL(0);
//	}
//	return Byte;
//}
// 
//void MyI2C_SendAck(uint8_t AckBit)
//{
//	MyI2C_W_SDA(AckBit);
//	MyI2C_W_SCL(1);
//	MyI2C_W_SCL(0);
//}
// 
//uint8_t MyI2C_ReceiveAck(void)
//{
//	uint8_t AckBit;
//	MyI2C_W_SDA(1);
//	MyI2C_W_SCL(1);
//	AckBit = MyI2C_R_SDA();
//	MyI2C_W_SCL(0);
//	return AckBit;
//}
// 
//#define MPU6050_ADDRESS		0xD0

// 
///**
//  * 函    数：MPU6050写寄存器
//  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
//  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
//  * 返 回 值：无
//  */
//void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
//{
//	MyI2C_Start();						//I2C起始
//	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
//	MyI2C_ReceiveAck();					//接收应答
//	MyI2C_SendByte(RegAddress);			//发送寄存器地址,即指定要写入哪个寄存器
//	MyI2C_ReceiveAck();					//接收应答
//	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
//	MyI2C_ReceiveAck();					//接收应答
//	MyI2C_Stop();						//I2C终止
//}
// 
///**
//  * 函    数：MPU6050读寄存器
//  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
//  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
//  */
//uint8_t MPU6050_ReadReg(uint8_t RegAddress)
//{
//	uint8_t Data;
//	
//	MyI2C_Start();						//I2C起始
//	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
//	MyI2C_ReceiveAck();					//接收应答
//	MyI2C_SendByte(RegAddress);			//发送寄存器地址
//	MyI2C_ReceiveAck();					//接收应答
//	
//	MyI2C_Start();						//I2C重复起始
//	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
//	MyI2C_ReceiveAck();					//接收应答
//	Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
//	MyI2C_SendAck(1);					//发送应答，1给从机非应答，终止从机的数据输出
//	MyI2C_Stop();						//I2C终止
//	
//	return Data;
//}
// 
///**
//  * 函    数：MPU6050初始化
//  * 参    数：无
//  * 返 回 值：无
//  */
//void MPU6050_Init_(void)
//{
//	
//	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
//	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
//	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
//	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
//	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
//	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
//	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
//}
// 
///**
//  * 函    数：MPU6050获取ID号
//  * 参    数：无
//  * 返 回 值：MPU6050的ID号
//  */
//uint8_t MPU6050_GetID(void)
//{
//	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
//}
// 
///**
//  * 函    数：MPU6050获取数据
//  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
//  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
//  * 返 回 值：无
//  */
//void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
//						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
//{
//	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
//	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
//	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
//	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
//	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
//	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
//	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//}

