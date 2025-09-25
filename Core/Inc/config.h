#ifndef CONFIG_H  // 如果没有定义 CONFIG_H
#define CONFIG_H  // 定义 CONFIG_H，防止重复包含
typedef enum
{
	ABS = 0,
	INCR = 1
} ValSet_Type_e;

//--------------------  电机  -------------------------
#define MOTOR_NUM 2 // 电机总数

#endif  // CONFIG_H  // 结束保护
