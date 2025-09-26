#include "Gimbal.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include <math.h>

uint8_t   GIMBAL_OFFSET_FLAG=1; //云台标志位
gimbal_control_t gimbal_control;
static void gimbal_feedback_update(gimbal_control_t *feedback_update,float *add_yaw,float *add_pitch);
// 先声明函数（告诉编译器函数的签名）
void gimbal_angle_limit(gimbal_control_t *gimbal_motort, float *add_yaw,float *add_pitch);
// 再声明其他函数（如 gimbal_calibration）
void gimbal_detact_calibration(gimbal_control_t *gimbal_motor_t);
//float temp=local_rc_ctrl->rc.ch[0];
void Gimbal_task(void){
//等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    while (1)
    {
        gimbal_feedback_update(&gimbal_control,0,0);
        gimbal_detact_calibration(&gimbal_control);
        gimbal_angle_limit(&gimbal_control,0,0);
        //以absolute_angle_set为目标值，absolute_angle（既motor_chassis[0].ecd）为当前值，进行pid串级环的运算，并将值存到motor_ready[0]结构体中

    }
    



}
/**
  * @brief          计算ecd与offset_ecd(中值)之间的相对角度
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
static void gimbal_feedback_update(gimbal_control_t *feedback_update,float *add_yaw,float *add_pitch){
    feedback_update->gimbal_pitch_motor.absolute_angle=motor_chassis[0].ecd;
    feedback_update->gimbal_yaw_motor.absolute_angle=motor_chassis[1].ecd;
    feedback_update->gimbal_pitch_motor.absolute_angle_set=feedback_update->gimbal_pitch_motor.absolute_angle+*add_pitch;
    feedback_update->gimbal_yaw_motor.absolute_angle_set=feedback_update->gimbal_yaw_motor.absolute_angle+*add_yaw;

    //计算设置角度后的目标角度与中值的相对角度，并以此为标准，因为最大和最小限幅值也是根据相对角度来定的，这样避免目标角度出现负值
    feedback_update->gimbal_pitch_motor.relative_angle_set=motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.absolute_angle_set,0);
    feedback_update->gimbal_yaw_motor.relative_angle_set=motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.absolute_angle_set,0);

    //计算云台相对于最大限幅值的相对角度，同时判断此时电机处于左值还是右值
    if(feedback_update->gimbal_pitch_motor.relative_angle_set-PITCH_Limit_Hight>0)
    feedback_update->gimbal_pitch_motor.relative_angle=fabs(feedback_update->gimbal_pitch_motor.relative_angle_set-PITCH_Limit_Hight);
    else if(feedback_update->gimbal_pitch_motor.relative_angle_set-PITCH_Limit_Hight<0)
    feedback_update->gimbal_pitch_motor.relative_angle=fabs(feedback_update->gimbal_pitch_motor.relative_angle_set-PITCH_Limit_Low);

    if(feedback_update->gimbal_yaw_motor.relative_angle_set-YAW_Limit_Hight>0)
    feedback_update->gimbal_yaw_motor.relative_angle=fabs(feedback_update->gimbal_yaw_motor.relative_angle_set-YAW_Limit_Hight);
    else if(feedback_update->gimbal_yaw_motor.relative_angle_set-YAW_Limit_Hight<0)
    feedback_update->gimbal_yaw_motor.relative_angle=fabs(feedback_update->gimbal_yaw_motor.relative_angle_set-YAW_Limit_Low);


}
//云台校准中值并执行归中
void gimbal_detact_calibration(gimbal_control_t *gimbal_motort){
    //添加标志位判断有无执行过归中，如果有，则不再归中
    if(GIMBAL_GET_FLAG(GIMBAL_OFFSET_FLAG)){
        MotorSetTar(motor_ready[0], 0.0f, ABS);  
        MotorSetTar(motor_ready[1], 0.0f, ABS);
        GIMBAL_FLAG_RESET(GIMBAL_OFFSET_FLAG);
    }


    // MotorSetTar(motor_ready[0],motor_ecd_to_angle_change(motor_chassis[0].ecd,gimbal_motort->gimbal_pitch_motor.absolute_angle),ABS);
    // MotorSetTar(motor_ready[1],motor_ecd_to_angle_change(motor_chassis[1].ecd,gimbal_motort->gimbal_yaw_motor.absolute_angle),ABS);

}



//云台限幅
void gimbal_angle_limit(gimbal_control_t *gimbal_motort,float *add_yaw,float *add_pitch){
	 if (gimbal_motort == NULL || add_pitch == NULL||add_yaw==NULL) {
        return;
    }
	 //pitch
    if(*add_pitch>0&&gimbal_motort->gimbal_pitch_motor.relative_angle-*add_pitch<0){
        *add_pitch=gimbal_motort->gimbal_pitch_motor.relative_angle;
        gimbal_motort->gimbal_pitch_motor.absolute_angle_set=gimbal_motort->gimbal_pitch_motor.absolute_angle+*add_pitch;
    }else if(*add_pitch<0&&gimbal_motort->gimbal_pitch_motor.relative_angle+*add_pitch<0){
        *add_pitch=-gimbal_motort->gimbal_pitch_motor.relative_angle;
        gimbal_motort->gimbal_pitch_motor.absolute_angle_set=gimbal_motort->gimbal_pitch_motor.absolute_angle+*add_pitch;
    }    

    //yaw
    if(*add_yaw>0&&gimbal_motort->gimbal_yaw_motor.relative_angle-*add_yaw<0){
        *add_yaw=gimbal_motort->gimbal_yaw_motor.relative_angle;
        gimbal_motort->gimbal_yaw_motor.absolute_angle_set=gimbal_motort->gimbal_yaw_motor.absolute_angle+*add_yaw;
    }else if(*add_yaw<0&&gimbal_motort->gimbal_yaw_motor.relative_angle+*add_yaw<0){
        *add_yaw=-gimbal_motort->gimbal_yaw_motor.relative_angle;
        gimbal_motort->gimbal_yaw_motor.absolute_angle_set=gimbal_motort->gimbal_yaw_motor.absolute_angle+*add_yaw;
    }    


}


//云台回中值

// void gimbal_calibration(gimbal_control_t *gimbal_motor_t) {
//     // 1. 先判断指针是否为空（避免空指针访问）
//     if (gimbal_motor_t == NULL) {
//         return;
//     }
    
 
    // MotorSetTar(motor_ready[0], 0.0f, ABS);  
    // MotorSetTar(motor_ready[1], 0.0f, ABS);
// }

// void Bounded_angle(float tar,float limit_zheng,float limit_fu){
//     if(tar>limit_zheng){
//         tar=limit_zheng;
//     }else if(tar<limit_fu){
//         tar=limit_fu;
//     }
// }