#include "motor.h"
#include "pid.h"
Motor_send *motor_ready[MOTOR_NUM];
Motor_list *motor_data[MOTOR_NUM];

/*
 * @brief  	设置电机目标值
 * @param	电机结构体指针
 * @param	目标值
 * @param   ABS->absolute target;
 *          INCR->add from perious target
 *          FEED->change feedforward
 * @retval 	无
 */
void MotorSetTar(Motor_send *motor,float val, ValSet_Type_e type)
{
    if ( type == ABS )
		motor -> target = val;
	else if ( type == INCR )
		motor -> target += val;
//         else
// 		sErrorHandel (OUT_OF_ENUM);
}



void Motor_Calc(gimbal_control_t *feedback_update)
{
	static float tar=0,real=0;

	//yaw轴计算
	tar=feedback_update->gimbal_yaw_motor.absolute_angle_set;
	real=feedback_update->gimbal_yaw_motor.absolute_angle;
    //过零点处理
	while(tar-real > PI)
	 	real += 2 * PI ;
	while(tar-real < -PI)
	 	real -= 2 * PI ;
	motor_ready[MOTOR_YAW]->output_Position=pid_calc_raw(&gimbal_yaw_angle_pid,tar,real);
    motor_ready[MOTOR_YAW]->output=pid_calc_speed(&gimbal_yaw_speed_pid,motor_ready[MOTOR_YAW]->output_Position,motor_data[MOTOR_YAW]->speed);

	//pitch轴计算
	tar=feedback_update->gimbal_pitch_motor.absolute_angle_set;
	real=feedback_update->gimbal_pitch_motor.absolute_angle;
    //过零点处理
	while(tar-real > PI)
	 	real += 2 * PI ;
	while(tar-real < -PI)
	 	real -= 2 * PI ;
	motor_ready[MOTOR_PITCH]->output_Position=pid_calc_raw(&gimbal_yaw_angle_pid,tar,real);
    motor_ready[MOTOR_PITCH]->output=pid_calc_speed(&gimbal_yaw_speed_pid,motor_ready[MOTOR_PITCH]->output_Position,motor_data[MOTOR_PITCH]->speed);

}