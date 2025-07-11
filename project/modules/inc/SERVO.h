#ifndef __SERVO_H__
#define __SERVO_H__

#include "ti_msp_dl_config.h"

typedef struct {
    GPTIMER_Regs *timer_base;           // 定时器基址
    DL_TIMER_CC_INDEX pwm_channel;      // PWM通道
    uint16_t min_pulse;                 // 最小脉宽(us)
    uint16_t max_pulse;                 // 最大脉宽(us)
    uint16_t period;                    // PWM周期(us)
    uint8_t current_angle;              // 当前角度
} SERVO_Handle;

extern SERVO_Handle servo1;

void SERVO_Init(SERVO_Handle *servo);
void SERVO_SetAngle(SERVO_Handle *servo, uint8_t angle);

#endif
