#include "Gimbal.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include <math.h>

gimbal_control_t gimbal_control;
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
// 先声明函数（告诉编译器函数的签名）
void gimbal_angle_limit(gimbal_control_t *gimbal_motort, float *add);
// 再声明其他函数（如 gimbal_calibration）
void gimbal_calibration(gimbal_control_t *gimbal_motor_t);
float temp=local_rc_ctrl->rc.ch[0];
void Gimbal_task(void){
//等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    while (1)
    {
        gimbal_feedback_update(&gimbal_control);
        gimbal_angle_limit(&gimbal_control,&temp);

    }
    



































    // static float tar_pitch;
    // static float tar_yaw;
    // tar_pitch=local_rc_ctrl->rc.ch[0];
    // tar_yaw=local_rc_ctrl->rc.ch[1];
    // Bounded_angle(tar_pitch,0,0);
    // Bounded_angle(tar_yaw,0,0);
    // MotorSetTar(motor_ready[0],tar_pitch,ABS);
    // MotorSetTar(motor_ready[1],tar_yaw,ABS);

}
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

//云台更新数据
static void gimbal_feedback_update(gimbal_control_t *feedback_update){
    feedback_update->gimbal_pitch_motor.absolute_angle=motor_chassis[0].ecd;
    feedback_update->gimbal_yaw_motor.absolute_angle=motor_chassis[1].ecd;
    feedback_update->gimbal_pitch_motor.absolute_angle_set=feedback_update->gimbal_pitch_motor.absolute_angle+local_rc_ctrl->rc.ch[0];
    feedback_update->gimbal_yaw_motor.absolute_angle_set=feedback_update->gimbal_yaw_motor.absolute_angle+local_rc_ctrl->rc.ch[1];
    //计算云台相对于最大限幅值的相对角度，同时判断此时电机处于左值还是右值
    if(feedback_update->gimbal_pitch_motor.absolute_angle_set-PITCH_Limit_Hight>0)
    feedback_update->gimbal_pitch_motor.relative_angle=fabs(feedback_update->gimbal_pitch_motor.absolute_angle_set-PITCH_Limit_Hight);
    else if(feedback_update->gimbal_pitch_motor.absolute_angle_set-PITCH_Limit_Hight<0)
    feedback_update->gimbal_pitch_motor.relative_angle=fabs(feedback_update->gimbal_pitch_motor.absolute_angle_set-PITCH_Limit_Low);

    if(feedback_update->gimbal_yaw_motor.absolute_angle_set-YAW_Limit_Hight>0)
    feedback_update->gimbal_yaw_motor.relative_angle=fabs(feedback_update->gimbal_yaw_motor.absolute_angle_set-YAW_Limit_Hight);
    else if(feedback_update->gimbal_yaw_motor.absolute_angle_set-YAW_Limit_Hight<0)
    feedback_update->gimbal_yaw_motor.relative_angle=fabs(feedback_update->gimbal_yaw_motor.absolute_angle_set-YAW_Limit_Low);


}
//云台校准中值
void gimbal_detact(gimbal_control_t *gimbal_motort){
    // motor_ecd_to_angle_change(motor_chassis[0].ecd,gimbal_motort->gimbal_pitch_motor.absolute_angle);
    motor_ecd_to_angle_change(motor_chassis[1].ecd,gimbal_motort->gimbal_yaw_motor.absolute_angle);
    MotorSetTar(motor_ready[0],motor_ecd_to_angle_change(motor_chassis[0].ecd,gimbal_motort->gimbal_pitch_motor.absolute_angle),ABS);
    MotorSetTar(motor_ready[1],motor_ecd_to_angle_change(motor_chassis[1].ecd,gimbal_motort->gimbal_yaw_motor.absolute_angle),ABS);

}



//云台限幅
void gimbal_angle_limit(gimbal_control_t *gimbal_motort,float *add){
	 if (gimbal_motort == NULL || add == NULL) {
        return;
    }
	 
    if(*add>0&&gimbal_motort->gimbal_pitch_motor.relative_angle-*add<0){
        *add=gimbal_motort->gimbal_pitch_motor.relative_angle;
    }else if(*add<0&&gimbal_motort->gimbal_pitch_motor.relative_angle+*add<0){
        *add=-gimbal_motort->gimbal_pitch_motor.relative_angle;
    }    

}
//云台回中值
void gimbal_calibration(gimbal_control_t *gimbal_motor_t) {
    // 1. 先判断指针是否为空（避免空指针访问）
    if (gimbal_motor_t == NULL) {
        return;
    }
    
    // 2. 函数调用用 ()，不是 []！参数顺序需与函数声明一致
    MotorSetTar(motor_ready[0], 0.0f, ABS);  // 0 建议写成 0.0f（匹配 float 类型参数）
    MotorSetTar(motor_ready[1], 0.0f, ABS);
}

// void Bounded_angle(float tar,float limit_zheng,float limit_fu){
//     if(tar>limit_zheng){
//         tar=limit_zheng;
//     }else if(tar<limit_fu){
//         tar=limit_fu;
//     }
// }