#include "gpio.h"
#include <math.h>
#include "BMI088driver.h"
#include "imu.h"
#include "Gimbal.h"
#include "FreeRTOS.h"
#include "semphr.h"     // 信号量相关类型和函数声明（包含 SemaphoreHandle_t 定义）

#define kp 				25.00f
#define ki 				0.005f
#define cycle_T 		0.005f//200hz
#define half_T 			0.0025f

int16_t Ax,Ay,Az,Gx,Gy,Gz;
fp32 gyro[3], accel[3], temp;
param_imu imu_data;
param_Angle imu_Angle;
offset imu_offset;
float q[4] = {1.0,0.0,0.0,0.0};//四元数数组
float exInt = 0.0 ,eyInt = 0.0 ,ezInt = 0.0 ;

uint8_t    	 SENSER_OFFSET_FLAG=1;  //传感器校准标志位

//////////////////////////////////////////////////////////////////////////////////
float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);  //0x5f3759df是一个平方根倒数速算法
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//////////////////////////////////////////////////////////////////////////////////

void IMU_GetValues(void)
{
	BMI088_read(gyro, accel, &temp);

	imu_data.AX = ((float)accel[0])/2048;
	imu_data.AY = ((float)accel[1])/2048;
	imu_data.AZ = ((float)accel[2])/2048;

	imu_data.GX = ((float)gyro[0])*0.001064;//将角度转成弧度
	imu_data.GY = ((float)gyro[1])*0.001064;
	imu_data.GZ = ((float)gyro[2])*0.001064;
	}

void IMU_AHRSupdate(param_imu* imu_temp)
{
	float ax,ay,az;
	float gx,gy,gz;
	ax = imu_temp->AX;
	ay = imu_temp->AY;
	az = imu_temp->AZ;
	gx = imu_temp->GX;
	gy = imu_temp->GY;
	gz = imu_temp->GZ;

	float vx, vy, vz;
	float ex, ey, ez;

	float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

	float norm = fast_sqrt(ax*ax+ay*ay+az*az);
    ax = ax * norm;//将加速度归一化，与四元数对应
    ay = ay * norm;
    az = az * norm;

	vx = 2 * (q1*q3 - q0*q2);//提取姿态矩阵中的重力分量
	vy = 2 * (q2*q3 + q0*q1);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    ex = (ay * vz - az * vy);//  |A|*|B|*sin<A,B>，叉乘得出姿态误差向量
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

	exInt += ki * ex;//对误差进行积分
	eyInt += ki * ey;
	ezInt += ki * ez;

	gx += kp * ex + exInt;//修正角速度值
	gy += kp * ey + eyInt;
	gz += kp * ez + ezInt;

	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_T;//更新四元数
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy)  * half_T;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx)  * half_T;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx)  * half_T;

	norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

	q[0] = q0 * norm;
	q[1] = q1 * norm;
	q[2] = q2 * norm;
	q[3] = q3 * norm;

}



uint8_t IMU_Offset(param_imu *value,offset *set){
	static float tempgx=0,tempgy=0,tempgz=0;
	static float tempax=0,tempay=0,tempaz=0;
	static uint16_t cnt_a=0;
	if(cnt_a==0){
		tempgx=0;
		tempgy=0;
		tempgz=0;
		tempax=0;
		tempay=0;
		tempaz=0;
		cnt_a=1;

	}
	tempax+=value->AX;
	tempay+=value->AY;
	tempaz+=value->AZ;
	tempgx+=value->GX;
	tempgy+=value->GY;
	tempgz+=value->GZ;
	
	if(cnt_a==1000){
		set->aX=tempax/cnt_a;
		set->aY=tempay/cnt_a;
		set->aZ=tempaz/cnt_a;
		set->gX=tempgx/cnt_a;
		set->gY=tempgy/cnt_a;
		set->gZ=tempgz/cnt_a;
		cnt_a = 0;
		return 1;
	}
	cnt_a++;
  return 0;
}

void IMU_Calibration(void){
	gyro[0]-=imu_offset.gX;
	gyro[1]-=imu_offset.gY;
	gyro[2]-=imu_offset.gZ;

	accel[0]-=imu_offset.aX;
	accel[1]-=imu_offset.aY;
	accel[2]-=imu_offset.aZ;
	if(GET_FLAG(calibration_OFFSET)){
		if(IMU_Offset(&imu_data,&imu_offset)){
			SENSER_FLAG_RESET(calibration_OFFSET);
		}
	}
}

void INS(void)
{
	//设置信号量
	if (xSemaphoreTake(g_xSemTicks, portMAX_DELAY)) {
	IMU_GetValues();
	IMU_Calibration();
	IMU_AHRSupdate(&imu_data);

	imu_Angle.Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.2957;
	imu_Angle.Roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)* 57.2957;

	imu_Angle.Yaw  += imu_data.GZ* 57.2957* cycle_T* 4;//由于在静止状态下z轴方向本身就有重力加速度，所以不用将加速度计和陀螺仪的结果融合，直接对角速度进行积分（这里只有乘上4后才会变得准一些，具体原因尚不清楚）
}
}
