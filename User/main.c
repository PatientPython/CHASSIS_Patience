/**
  ******************************************************************************
  * @file    main.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.20
  * @brief   主函数，初始化各个外设、添加各项任务、启动FreeRTOS任务调度器。
  ******************************************************************************
*/

/****************************测试使用****************************/
/**** FreeRTOS里的头文件 ****/
#include "FreeRTOS.h"
#include "Task.h"

#include "RM_BSP.h"
#include "DebugTask.h"
#include "ReceiverTask.h"
#include "Chassis_Task.h"
#include "TIM_Config.h"
#include "SendDataTask.h"
TaskHandle_t DebugTaskHandle;
uint32_t DebugTaskHighWaterMark;
TaskHandle_t ReceiverTaskHandle;
uint32_t ReceiverTaskHighWaterMark;
TaskHandle_t ChassisTaskHandle;
uint32_t ChassisTaskHighWaterMark;
TaskHandle_t SendDataTaskHandle;
uint32_t SendDataTaskHighWaterMark;
/****************************测试使用****************************/

int main()
{
    /****************************测试使用上边界****************************/
    BSP_All_Init(); //底层外设初始化
    
    xTaskCreate(ReceiverTask,"ReceiverTask",600,NULL,30,&ReceiverTaskHandle);
    xTaskCreate(ChassisTask,"ChassisTask",1000,NULL,25,&ChassisTaskHandle);
    xTaskCreate(SendDataTask,"SendDataTask",600,NULL,20,&SendDataTaskHandle);
    xTaskCreate(DebugTask,"DebugTask",600,NULL,4,&DebugTaskHandle);

    DebugTaskHighWaterMark      = uxTaskGetStackHighWaterMark(DebugTaskHandle);
    ReceiverTaskHighWaterMark   = uxTaskGetStackHighWaterMark(ReceiverTaskHandle);
    ChassisTaskHighWaterMark    = uxTaskGetStackHighWaterMark(ChassisTaskHandle);
    SendDataTaskHighWaterMark   = uxTaskGetStackHighWaterMark(SendDataTaskHandle);

    RunTimeReset(); //任务开始前重置系统计时器
    vTaskStartScheduler(); //开启任务调度器
    /****************************测试使用下边界****************************/

    /* 后续需要干的事情
        1、对于串腿，腿空的一侧为前面。对于2025年的车灯条侧为右（电池侧为后）

       2、 修改各个中断的抢占优先级（见RM_BSP.h)

        3、PID的位置环速度环命名用Speed Pos，其他的一般用Angle Vel Torque之类的命名

       4、可以考虑一下怎么分配变量更加合理，比如说IMU2的数据有关节电机、姿态解算相关数据
          可以考虑把这些数据放到另外的一个结构体里面，比如说xx运动学DataStruct或者xx几何DataStruct之类的
        
        目前考虑：每个数据最多三个结构体或者是相同变量。
                (1)一个是接收时使用（比如接收IMU1、2的数据）
                (2)然后真正用到的结构体，比如关节电机控制结构体、底盘姿态解算结构体
                (3)最后是发送时使用的结构体，比如发送给IMU1、2的数据结构体
        数据的流向必须遵循(1)-(2)-(3)的顺序。且在上层调用代码的时候只能使用(2)结构体，不能直接使用(1)和(3)结构体的数据

        6、报错不一定要用Debug，比如可以用某种灯闪来代替某个CAN线掉了之类的

        7、数据处理过程
            (1) 解析：把接收到的原始数据，通过位操作等方式解析
            (2) 处理：把解析出来的数据进行滤波、计算等处理，得到最终的反馈值
            (3) 分发：把处理好的数据分发到各个正式变量、结构体中

        1、写代码注意要点：写完任务后，可以先跑一遍FreeRTOS，然后用WaterMark那个函数来查看任务的栈剩余多少。
        最后如果空间充足的话，给使用栈的2倍。如果不够就给1.5倍左右。
    */
    
    /*目前中断的优先级如下
    目前的中断：   CAN1的发送0 0，接收1 0. CAN2的发送0 0，接收1 0.
                  WWDG中断0 0，
                  串口1空闲中断0 1，串口2空闲中断0 2，串口3空闲中断0 10，
                  串口4空闲中断0 6，串口5空闲中断0 6，串口6空闲中断0 10
                  
                  
中断服务函数没写如果调用初始化函数会出现卡死状态，猜测可能是因为溢出了或者卡在中断服务函数了
所以先把串口3以外的都注释（因为串口3没有RX，所以不会进接收中断）
    我的考虑：WWDG重要，给0，其他的可以往1后面排。
*/
    
    /*  可能需要使用到的FreeRTOS函数
        xTaskCreate()//动态创建（是否改为静态创建）
        vTaskDelayUntil()//绝对延时！第一个参数是上次时间的指针，第二个参数是延时时间
        xTaskGetTickCount()//获取系统运行时间
        uxTaskGetStackHighWaterMark()//获取任务剩余空间
    */
    
    /*  老代码个人补充说明：
        1、原来的HITOS中，任务优先级从高到低为
            SystemMonitor       1000ms
            SendData            1ms     
            GimbalTask          1ms     
            ChassisTask         1ms     
            SupplyPelletTask    1ms
            FrictionWheelTask   1ms
            DebugTask           10ms
            FreeRTOS任务优先级可以配置为0~31（见configMAX_PRIORITIES）
            各项任务最好使用绝对延时（尤其是Chassis，因为这个需要乘时间来计算位移）
    
        2、
            2025平步的电机：摩擦轮x2，拨盘、Pitch、Yaw、四个关节、两个轮毂

        3、CM1是右电机（灯条侧）
            相对的CM2就是左边
    */
    
    /*  对于代码的一些疑问
        1、为什么要区分PitchAngle和PitchAngleBuff，Yaw和Roll也是

            //LegLeftPosDistribute.Theta,                  //左腿摆角，往前为正，与我的定义相反
            //LegLeftPosDistribute.Theta_dot_fliter.out,   //左腿摆角速度，往前为负，与我的定义相同
            //LegRightPosDistribute.Theta,                 //右腿摆角，往前为负，与我的定义相同
            //LegRightPosDistribute.Theta_dot_fliter.out,  //右腿摆角速度，往前为正，与我的定义相反
    */
   /*老代码移植注意点
        1、通讯相关的结构体里面的变量顺序可能不能随便变，不然可能会导致通讯的数据赋值错误

        2、如何修改主控、云控间通信（未试验猜测版本）
            (1) 修改IMUxData_StructTypeDef里面的成员
            (2) 修改后用sizeof()查看新结构体的大小，根据此修改串口缓冲区的大小（定义在USART_Communication.c里面）
            (3) 

            (最后) 把云控的做镜像修改（发送和接收镜像），注意成员的顺序要严格一一对应
   */
    
    /*实验性玩法：（重构完了之后搞）
        代码优化/调整：1、原来的代码LQR里面的VelFB是用Luenberger观测器估计出来的速度
                      现在可以试试卡尔曼滤波器估计速度（虽然我还不清楚怎么具体实现）
    */

    while(1)
    {
        
    }
}
