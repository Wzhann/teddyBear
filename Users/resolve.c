#include <math.h>

// 定义机械臂的长度
#define L1 10.0  // 第一段长度
#define L2 10.0  // 第二段长度
#define L3 10.0  // 第三段长度

// 定义PI常量
#define PI 3.14159265358979323846

// 逆运动学解算函数
void inverse_kinematics(double x, double y, double z, double *theta1, double *theta2, double *theta3) 
{
	double alpha,beta,gamma,beta1,beta2;
    // 计算
	alpha = atan2(y,x);//计算底座角（弧度制）
    *theta1 = atan2(y, x);

    // 计算末端在x-y平面上的投影距离
    double Lp = sqrt(x * x + y * y);
	// 计算末端在z轴上的高度
    double Lh = z - L1;
	// 辅助线斜边距离
    double Ld = sqrt(Lp * Lp + Lh * Lh);
	if(Lh < 0)
	{
		beta1 = acos((L2 * L2 + Ld * Ld - L3 * L3)/(2 * L2 * Ld));
		beta2 = atan2(Lp,fabs(Lh));
		beta = PI - (beta1 + beta2);
	}
	else if(Lh == 0)
	{
		beta1 = acos((L2 * L2 + Ld * Ld - L3 * L3)/(2 * L2 * Ld));
		beta = PI / 2 - beta1;
	}
	else if(Lh > 0)
	{
		beta1 = acos((L2 * L2 + Ld * Ld - L3 * L3)/(2 * L2 * Ld));
		beta2 = atan2(fabs(Lh),Lp);
		beta = PI/2 - (beta1 + beta2);
	}
    // 使用余弦定理计算theta3
    double cos_gamma = (L2 * L2 + L3 * L3 - Ld * Ld) / (2 * L2 * L3);
    gamma = PI - acos(cos_gamma);

    // 将角度转换为度
    *theta1 = alpha * 180 / PI;
    *theta2 = beta * 180 / PI;
    *theta3 = gamma * 180 / PI;
}
//正运动学解算函数
void forward_kinematics(double* x, double* y, double* z, double theta1, double theta2, double theta3) 
{
	double alpha = theta3 - (PI/2 - theta2);
	double Lp = L2 * sin(theta2) + L3 * cos(alpha);
	*x = Lp * cos(theta1);
	*y = Lp * sin(theta1);
	*z = L1 + L2 * cos(theta2) - L3 * sin(alpha);
}

















