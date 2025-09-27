// 在 Gimbal.h 开头添加
#ifndef GIMBAL_H  // 如果未定义 GIMBAL_H
#define GIMBAL_H  // 定义 GIMBAL_H，防止重复包含

#include "FreeRTOS.h"
#include "semphr.h"     // 信号量相关类型和函数声明（包含 SemaphoreHandle_t 定义）
#include "CAN_receive.h"
#include "struct_typedef.h"
#include "remote_control.h"


//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191


//电机限幅机械角度
#define YAW_Limit_Low 2000
#define YAW_Limit_Hight 6000
#define PITCH_Limit_Low 1000
#define PITCH_Limit_Hight 7000

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

//对 GIMBAL_OFFSET_FLAG 的位的操作
#define GIMBAL_FLAG_SET(FLAG)   GIMBAL_OFFSET_FLAG|=FLAG                //标志位置1
#define GIMBAL_FLAG_RESET(FLAG) GIMBAL_OFFSET_FLAG&=~FLAG               //标志位值0
#define GIMBAL_GET_FLAG(FLAG)   (GIMBAL_OFFSET_FLAG&FLAG)==FLAG ? 1 : 0  //获取标志位状态

#define calibration_gimbal 0x01 //第一位校准标志位
extern uint8_t   GIMBAL_OFFSET_FLAG; //标志位组

//中值
#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f



typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;

    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //相对rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //绝对rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} gimbal_motor_t;


typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
} gimbal_control_t;//云台模式控制

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201

/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
 extern void Gimbal_task(void);

static void gimbal_feedback_update(gimbal_control_t *feedback_update,float *add_yaw,float *add_pitch);
// 先声明函数（告诉编译器函数的签名）
void gimbal_angle_limit(gimbal_control_t *gimbal_motort, float *add_yaw,float *add_pitch);
// 再声明其他函数（如 gimbal_calibration）
void gimbal_detact_calibration(gimbal_control_t *gimbal_motor_t);
//float temp=local_rc_ctrl->rc.ch[0];
extern SemaphoreHandle_t g_xSemTicks;

#endif  // 结束保护宏
