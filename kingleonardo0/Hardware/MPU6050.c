#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS     0xD0
#define ACCEL_RANGE 2       //g
#define GYRO_RANGE 250      //°/s

typedef struct
{
    double AX;
    double AY;
    double AZ;
    double GX;
    double GY;
    double GZ;
	double KalmanAngleX;
	double KalmanAngleY;
	double KalmanAngleZ;
}MPU6050_Data;

typedef struct
{
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
}MPU6050_OriginalData;

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();
    
    return Data;
}

void MPU6050_Init()
{
    MyI2C_Init();
	
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);//配置电源管理寄存器1，选择X轴陀螺仪时钟
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);//配置电源管理寄存器2，六轴均不待机
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);//配置采样率分频，10分频，值越小越快
    MPU6050_WriteReg(MPU6050_CONFIG,0x06);//配置寄存器，滤波参数最大
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x00);//配置陀螺仪寄存器，最小量程±250°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x00);//配置加速度计寄存器，最小量程±2g
}

uint8_t MPU6050_GetID()
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetOriginalData(MPU6050_OriginalData *OriginalData)
{
    uint8_t DataH,DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    OriginalData->AccX = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    OriginalData->AccY = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    OriginalData->AccZ = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    OriginalData->GyroX = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    OriginalData->GyroY = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    OriginalData->GyroZ = (DataH << 8) | DataL;
}

void MPU6050_GetData(MPU6050_OriginalData *OriginalData, MPU6050_Data *Data)
{
    Data->AX = OriginalData->AccX / 32768.0 * ACCEL_RANGE;
    Data->AY = OriginalData->AccY / 32768.0 * ACCEL_RANGE;
    Data->AZ = OriginalData->AccZ / 32768.0 * ACCEL_RANGE;
    Data->GX = OriginalData->GyroX / 32768.0 * GYRO_RANGE;
    Data->GY = OriginalData->GyroY / 32768.0 * GYRO_RANGE;
    Data->GZ = OriginalData->GyroZ / 32768.0 * GYRO_RANGE;
}
