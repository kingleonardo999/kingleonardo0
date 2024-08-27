#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Servo.h"
#include "Kalman.h"

int main()
{
    OLED_Init();
    OLED_ShowString(1,1,"Wait!");
    OLED_ShowString(2,1,"INITIALIZING...");
	Kalman_Init();
    Servo_Init();
    
    OLED_Clear();
	OLED_ShowString(2,1,"Pitch:");
	OLED_ShowString(3,1,"Roll :");
	OLED_ShowString(4,1,"Yaw  :");
    
    while(1)
    {
        Kalman_Loop();
        Servo_SetAngleDown(90 + k_pitch);
        OLED_ShowFloatNum(2,7,k_pitch,3,5);
        Servo_SetAngleUp(90 + k_roll);
        OLED_ShowFloatNum(3,7,k_roll,3,5);
        Delay_us(10);
    }
}
