#ifndef __BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "KEY.h"
#include "DELAY.h"
#include "MOTOR.h"
#include "SERVO.h"
#include "K230.h"

extern uint8_t Key_Num;
extern uint64_t Systick_Count;
extern Motor_Handle motor_left_front, motor_left_back, motor_right_back, motor_right_front;
uint32_t motor_base_speed;
extern float Kp, Kd;
extern float delta_angle;

void Board_Init(void);

#endif
