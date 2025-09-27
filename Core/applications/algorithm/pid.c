#include "pid.h"

float PI=3.1415926;
pid_struct_t gimbal_yaw_speed_pid;
pid_struct_t gimbal_yaw_angle_pid;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)//PID初始化函数
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

float rad_format(float ref, float fdb){
	if(ref-fdb>PI){
			fdb+=2*PI;

	}else if(ref-fdb<-PI){
			fdb=fdb-2*PI;
	}
	return ref-fdb;
}
void LIMIT_MIN_MAX(float be,float nmax,float mmax){
	if(be>=mmax){
		be=mmax;
	}else if(be<=nmax){
		be=nmax;
	}
}
                                         //目标      //实际
float pid_calc_speed(pid_struct_t *pid, float tar, float real)//PID运算函数
{
  pid->ref = tar;
  pid->fdb = real;

  pid->err[0] = pid->ref - pid->fdb;


  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  pid->err[1] = pid->err[0];


  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}
                                         //目标      //实际
float pid_calc_raw(pid_struct_t *pid, float tar, float real)//PID运算函数
{
  pid->ref = tar;
  pid->fdb = real;

  pid->err[0] = rad_format(pid->ref, pid->fdb);


  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  pid->err[1] = pid->err[0];

  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}


void gimbal_PID_init()//角度环和速度环的PID初始化,只是初测出来的数据，具体还需要测试
{
	pid_init(&gimbal_yaw_speed_pid, 5, 0.01, 0.2, 1000, 1000);//P=30,I=0,D=0
	pid_init(&gimbal_yaw_angle_pid, 200, 0.7, 1, 600, 1000);//P=500,I=0,D=1
}
