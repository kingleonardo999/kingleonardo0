#include "stm32f10x.h"                  // Device header
#include "PWM.h"


void Servo_Init()
{
    PWM_Init();
}

void Servo_SetAngleUp(float Angle)
{
    PWM_SetCompare2(Angle / 180 *2000 +500);
}

void Servo_SetAngleDown(float Angle)
{
    PWM_SetCompare1(Angle / 180 *2000 +500);
}
