#include "VPC.h"
#include "Serial.h"
#include "cmsis_os.h"


 void VPC_Receive(void)
 {
   
    UnPack_Data_ROS2(buf_receive_from_nuc, &aim_packet_from_nuc, (sizeof(aim_packet_from_nuc)+1));
    
    //    chassis_cmd.vy=aim_packet_from_nuc.vy;
    //    gimbal_cmd.v_yaw=aim_packet_from_nuc.v_yaw;

    //解包之后要根据我们的控制逻辑来改，调用ROS2传来的结构体的目标数据

 }
 
 

void VPC_Init(void)
{
  aim_packet_to_nuc.detect_color=0;//1-blue 0-red
  Send_Packet_Init(&aim_packet_to_nuc);

}

// void VPC_Task(void *argument)
// {
//    VPC_Init();
//    uint32_t lastWakeTime = osKernelGetTickCount();
//    for(;;)
//     {
//         VPC_Receive();
//         Pack_And_Send_Data_ROS2(&aim_packet_to_nuc);
//         osDelayUntil(&lastWakeTime, VPC_TASK_PERIOD);
//     }

// }