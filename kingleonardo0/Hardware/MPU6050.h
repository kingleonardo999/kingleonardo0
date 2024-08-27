#ifndef __MPU6050_H__
#define __MPU6050_H__

typedef struct
{
    double AX;
    double AY;
    double AZ;
    double GX;
    double GY;
    double GZ;
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

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_Init();
void MPU6050_GetData(MPU6050_OriginalData *OriginalData, MPU6050_Data *Data);
void MPU6050_GetOriginalData(MPU6050_OriginalData *OriginalData);
uint8_t MPU6050_GetID();


#endif
