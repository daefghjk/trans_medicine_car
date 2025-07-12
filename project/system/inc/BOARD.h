#ifndef __BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "KEY.h"
#include "DELAY.h"
#include "MOTOR.h"
#include "SERVO.h"

extern uint8_t Key_Num;
extern uint64_t Systick_Count;
extern Motor_Handle motor0, motor1, motor2, motor3;

void Board_Init(void);

#endif
