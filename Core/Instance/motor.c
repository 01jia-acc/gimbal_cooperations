#include "motor.h"
#include "config.h"
Motor_send *motor_ready[MOTOR_NUM];


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


