/**
  ******************************************************************************
  * @file    ReadMe.txt
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.15
  * @brief   对这份代码、机器人操作、机器人相关的说明
  ******************************************************************************
*/

//待优化：后续考虑做一份Word文件出来，可以放一些图片什么的，更清楚
//待整理
关于2025并腿平步老代码的方向性说明
    1、电池侧为后方。
    2、关节电机，从外侧向内看，逆时针转为正，顺时针为负
    3、底盘Pitch平放时为0，后仰为正，前倾为负。
    4、底盘Yaw，从上往下看，逆时针为正，顺时针为负。
    5、底盘Roll，左侧高为正，右侧高为负
//待整理
五连杆解算的坐标定义：都是从外往内看，倒过来
    1、左侧：关节电机3为A点，也是原点。
    2、右侧：关节电机2为A点，也是原点。
//待整理
    1、五连杆坐标系：指的是五连杆解算时使用的坐标，即以关节电机连线为x轴
    2、地面坐标系：指的是以地面为x轴

一、关于变量的说明
    1-1、缩写说明
        (1)  Coe    Coefficient     系数的意思
        (2)  TH     threshold       阈值的意思
        (3)  CNT    Count           计数器
        (4)  Pos    Position        位置
        (5)  HM     Hub Motor       轮毂电机
        (6)  JM     Joint Motor     关节电机
        (7)  A2R    AngleToRadian   角度制转弧度制
             R2A    RadianToAngle   弧度制转角度制
        (8)  Cal    Calculate       计算
        (9)  Len    Length          长度
        (10) ZP     ZeroPoint       零点
        (11) FB     Feedback        反馈（一般用来表示真实值）
        (12) Des    Destination     目标（一般用来表示目标值）
        (13) Dis    Displacement    位移
        (14) V/Vel  Velocity        速度
        (15) LPF    LowPassFilter   低通滤波器
        (16) Ctrl   Control         控制
        (17) Temp   Temperature     温度（主意与临时变量的temp区分开）
        (18) Aux    Auxiliary       辅助
        (19) Hor    Horizontal      水平的
        (20) Avg    Average         平均
        (21) Comp   Compensation    补偿
        (22) Norm   Normal          正常的
        (23) FF     FeedForward     前馈
        (24) Para   Parameter       参数  
        (25) tmp    Temp            临时的
        (26) dec    decrease        减少
        () EncoderPPR   Pulses Per Revolution     每转一圈的脉冲数（编码器相关数据）
        () ESC          Electronic Speed Control  电调

    1-2、一些翻译
        (1) Thigh       大腿
        (2) Calf        小腿
        (3) Parse       解析
        (4) Rated       额定的
        (5) Ampere      安培
        (6) Manual      手动的

    2-1、变量命名规则-变量前缀
        (0-1) 对于部分变量（如串口发送缓冲区之类的），前面不添加多余命名。
              宏定义、const变量不添加多余命名。

        (0-2)结构体的st分为大小写，
            小写的st代表辅助结构体，一般在接收、发送、辅助计算的时候使用，使用完辅助结构体后要马上将变量更新到正式结构体中
            大写的ST代表正式结构体，真正在代码中的各处调用，尽量保证一个独立变量只存在于一个正式结构体里面，不要重复定义

        命名顺序从(1)到(2)到(3)，比如底盘(CH)安全模式(SafeMode)全局(G)标志位(F)：GFCH_SafeMode
        (1) G：全局变量global       S：静态变量static
        (2) ST：结构体变量Struct    EM：枚举变量enum    F：标志位flag
        (3) GB：云台变量Gimbal      CH：底盘变量chassis


    2-2、变量命名规则-数字序号
        (0) 对于2025的并腿车，电池侧为后边，灯条侧为右边，右边为轮毂1，左边为轮毂2。

        (1) 云台云控IMU1
            底盘云控IMU2

        (2) 定义串腿腿部一侧为后方，关节电机的序号如下
            左前1  右前2  左后3  右后4

    3、关于变量的单位
        所有变量的单位应该在定义的时候，或者在结构体typedef中说明清楚

    4、其他特殊说明
        (1) 给电机发送的数据统一命名为Current，但是实际上的数据是电流的线性映射值（见平步资料整理-军火说明书里面的电调CAN协议），并不是真正的电流值。




二、关于函数的说明
    1、函数编写规则
        (1) 把函数的简介、说明、参数、返回值写好，可以参考模板.txt里面。
        (2) 尽量做到一个函数只做一件事。
        (3) 如果同样的一个判断出现了两次（比如两个地方都有Angle > 60），那就得把这个写成函数，或者是把判断的数值写成宏定义
    
    2、函数命名规则
        (1) 对于一般的辅助函数（比如说只在某个函数中调用一次的函数），以_下划线开头
        (2) 对于更简单的辅助函数（比如说某个函数的if判断函数），以__双下划线开头


三、关于算法的说明
    1、LQR中的变量
        (1) x_Vector状态向量中的元素如下，计算时是目标值-实际值
            s   s_dot   phi   phi_dot   theta_l theat_l_dot      theta_r theta_r_dot    pitch pitch_dot
            位移        偏航yaw         左腿夹角                  右腿夹角                俯仰pitch

        (2) u_Vector的元素如下
            T_wl,   T_wr,    T_bl,     T_br
            左轮扭矩 右轮扭矩 左摆杆扭矩 右摆杆扭矩

    2、五连杆解算中的变量
        (1) 正方向、角度、变量定义等等，请查看：平步资料整理\轮腿前置知识_LuCkY\2_五连杆与VMC\五连杆变量定义.png

    3、其他相关控制算法可以看：平步资料整理\轮腿前置知识_LuCkY\其他控制算法

四、关于操作的说明
    1、遥控器操作
        //待补充
    
    2、键盘操作
        //待补充

五、平步自身说明
//待整理
    1、接线说明

    2、各项正常帧率说明
        (1) CAN1Rx_fps：5000左右（）
            CAN2Rx_fps：2000左右（左轮毂电机1000，右轮毂电机1000）
            HubMotor1Rx_fps：1000左右
            HubMotor2Rx_fps：1000左右
            USART1Rx_fps：70左右（遥控器关闭的时候为0）
            USART2Rx_fps：1000左右
            USART3Rx_fps：100左右
            
        (2) ChassisTask_fps：1000左右
            DebugTask_fps：100左右
            SendDataTask_fps：1000左右

    Last、上场前准备
    //看手机的备忘录

