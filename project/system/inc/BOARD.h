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
#include "BLE.h"

#define WHELL_DIAMETER      0.065   //车轮直径
#define ROTARY_SHAFT_DIS    0.09   //转轴距离

typedef enum{
    BASE_1,
    BASE_2,
    EXTRA_1,
    EXTRA_2,
    BASE,    //未检测到小车2，已知为基础，但不知第几问
    EXTRA   //检测到小车2，已知为拓展，但不知第几问
} MODE_Type;

extern uint8_t Key_Num;
extern volatile uint64_t Systick_Count;
extern Motor_Handle motor_left, motor_right;
extern const uint32_t motor_base_speed;
extern const float Kp, Kd;
extern volatile float delta_angle;
extern volatile uint8_t find_line_en;
extern volatile int32_t left_count;
extern volatile int32_t right_count;
extern volatile uint8_t turn_dir;
extern volatile uint8_t ble_flag;
extern volatile MODE_Type mode;

void Board_Init(void);
void Motor_SetAllDir(Motor_DirectionType dir);
void Motor_SetAllSpeed(uint16_t percent);

#endif
