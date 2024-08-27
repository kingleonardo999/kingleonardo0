#include "stm32f10x.h"                  // Device header
#include <math.h>
#include "MPU6050.h"
#include "OLED.h"

uint32_t timer;//计时器

MPU6050_OriginalData OData;//获取原始数据的结构体
MPU6050_Data Data;//存放处理后的数据结构体

//计算偏移量
float i;                                //计算偏移量时的循环次数
float ax_offset = 0, ay_offset = 0;     //x，y轴加速度偏移量
float gx_offset = 0, gy_offset = 0;     //x，y轴角速度偏移量

//参数
float rad2deg = 57.29578;               //弧度到角度的换算系数
float roll_v = 0, pitch_v = 0;          //换算到x，y轴上的角速度

//定义微分时间                            
float now = 0, lasttime = 0, dt = 0;    //定义微分时间

//三个状态，先验状态，观测状态，最优估计状态
float gyro_roll = 0, gyro_pitch = 0;    //陀螺仪积分计算出的角度，先验状态
float acc_roll = 0, acc_pitch = 0;      //加速度计测出的角度，观测状态
float k_roll = 0, k_pitch = 0;          //卡尔曼滤波后估计出的最优角度，最优估计状态

//误差协方差矩阵P
float e_P[2][2] = {{1,0},{0,1}};        //误差协方差矩阵，这里的e_P既是先验估计的P，也是更新后的P

//卡尔曼增益K
float K_K[2][2] = {{0,0},{0,0}};        //这里卡尔曼增益矩阵K是一个2 x 2的矩阵


void Kalman_Init()
{
    //初始化MPU6050模块
    MPU6050_Init();
    
    //配置毫秒级计数器做时钟
    //开启TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    
    //选择内部时钟源
    TIM_InternalClockConfig(TIM2);
    
    //时基单元初始化
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //不分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;     //向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 100-1;                       //计数100，一次中断周期为1ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 720-1;                    //频率100KHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                //每个周期触发一次中断
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
    
    TIM_ClearFlag(TIM2,TIM_FLAG_Update);        //清除更新标志，确保在中断服务程序执行完毕后，定时器能够在下一次溢出时再次触发中断
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);    //定时器中断使能
    
    //设置中断优先级分组模式为2位抢占优先级 + 2位子优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    //配置特定中断的属性
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//要配置的中断通道为TIM2_IRQn，即TIM2定时器的中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//设置中断通道的抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//设置了中断通道的子优先级
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM2,ENABLE);//时钟使能
    
    //计算偏移量
    for(i = 1; i <= 200; i ++)
    {
        MPU6050_GetOriginalData(&OData);//获取原始数据
        MPU6050_GetData(&OData,&Data);//处理原始数据
        ax_offset += Data.AX;
        ay_offset += Data.AY;
        gx_offset += Data.GX;
        gy_offset += Data.GY;
    }
    ax_offset /= 200;  //计算x轴加速度的偏移量
    ay_offset /= 200;  //计算y轴加速度的偏移量
    gx_offset /= 200;  //计算x轴角速度的偏移量
    gy_offset /= 200;  //计算y轴角速度的偏移量
}

void TIM2_IRQHandler()
{
    if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
    {
        timer ++;
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    }
}

void Kalman_Loop()
{
    //计算微分时间
    now = timer;                        //当前时间
    dt = (now - lasttime) / 1000.0;     //微分时间（s）
    lasttime = now;                     //上一次采样时间（ms）
    
    //获取角速度和加速度
    MPU6050_GetOriginalData(&OData);
    MPU6050_GetData(&OData,&Data);
    
    //step1：计算先验状态
    //计算x，y轴的角速度
    roll_v = (Data.GX-gx_offset)+((sin(k_pitch)*sin(k_roll))/cos(k_pitch))*(Data.GY-gy_offset)+((sin(k_pitch)*cos(k_roll))/cos(k_pitch))*Data.GZ;//w_roll
    pitch_v = cos(k_roll)*(Data.GY-gy_offset) - sin(k_roll)*Data.GZ;//w_pitch
    gyro_roll = k_roll + dt*roll_v;//先验roll角度
    gyro_pitch = k_pitch + dt*pitch_v;//先验pitch角度
    
    //step2：计算先验误差协方差矩阵P
    e_P[0][0] = e_P[0][0] + 0.0025;//这里Q矩阵是一个对角矩阵，对角元为0.003
    e_P[0][1] = e_P[0][1] + 0;
    e_P[1][0] = e_P[1][0] + 0;
    e_P[1][1] = e_P[1][1] + 0.0025;
    
    //step3：更新卡尔曼增益K
    K_K[0][0] = e_P[0][0]/(e_P[0][0]+0.25);
    K_K[0][1] = 0;
    K_K[1][0] = 0;
    K_K[1][1] = e_P[1][1]/(e_P[1][1]+0.25);
    
    //step4：计算最优估计状态
    //观测状态
    //roll角度
    acc_roll = atan((Data.AY-ay_offset)/(Data.AZ))*rad2deg;
    //pitch角度
    acc_pitch = -1*atan((Data.AX-ax_offset)/sqrt((Data.AY-ay_offset)*(Data.AY-ay_offset)+(Data.AZ)*(Data.AZ)))*rad2deg;
    //最优估计状态
    k_roll = gyro_roll + K_K[0][0]*(acc_roll - gyro_roll);
    k_pitch = gyro_pitch + K_K[1][1]*(acc_pitch - gyro_pitch);
    
    //step5：更新协方差矩阵P
    e_P[0][0] = (1 - K_K[0][0])*e_P[0][0];
    e_P[0][1] = 0;
    e_P[1][0] = 0;
    e_P[1][1] = (1 - K_K[1][1])*e_P[1][1];
}

