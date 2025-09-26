#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
#include "config.h"
#include "struct_typedef.h"
//接收电机结构体
typedef struct 
{
    uint16_t angle;
    int16_t speed;
    int16_t given_current;
    uint16_t temperate;/* data */
    int16_t last_ecd;
}__attribute__((__packed__)) Motor_list;

//发送电机结构体
typedef struct 
{
    float target;/* data */
    float output;
    float output_Position;
}Motor_send;
extern Motor_send *motor_ready[MOTOR_NUM];

void MotorSetTar(Motor_send *motor,float val, ValSet_Type_e type);

#endif /* __MOTOR_H */
