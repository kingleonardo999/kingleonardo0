#ifndef __KALMAN_H__
#define __KALMAN_H__

extern uint32_t timer;
extern float k_roll, k_pitch;

void Kalman_Init();
void Kalman_Loop();

#endif
