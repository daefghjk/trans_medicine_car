#ifndef __BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "KEY.h"
#include "DELAY.h"
#include "MOTOR.h"
#include "SERVO.h"
#include "K230.h"
#include "ENCODER.h"

#define WHELL_DIAMETER      0.065   //车轮直径
#define ROTARY_SHAFT_DIS    0.105   //转轴距离

extern uint8_t Key_Num;
extern volatile uint64_t Systick_Count;
extern Motor_Handle motor_left_front, motor_left_back, motor_right_back, motor_right_front;
extern uint32_t motor_base_speed;
extern float Kp, Kd;
extern volatile float delta_angle;
extern volatile uint8_t find_line_en;
extern volatile int32_t left_count;
extern volatile int32_t right_count;
extern volatile uint8_t turn_dir;

void Board_Init(void);
void Motor_SetAllDir(Motor_DirectionType dir);

#endif
