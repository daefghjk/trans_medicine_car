#include "SERVO.h"
#include "ti_msp_dl_config.h"

// 参考 ti_msp_dl_config.h 的 SERVO_INST 和 GPIO_SERVO_C0_IDX 及引脚定义
SERVO_Handle servo1 = {
    .timer_base = SERVO_INST,                  // TIMA1
    .pwm_channel = GPIO_SERVO_C0_IDX,          // DL_TIMER_CC_0_INDEX
    .min_pulse = 500,                          // 500us
    .max_pulse = 2500,                         // 2500us
    .period = 20000,                           // 20ms周期（50Hz）
    .current_angle = 90                        // 默认90度
};

// 角度转脉宽
static uint16_t SERVO_AngleToPulse(SERVO_Handle *servo, uint8_t angle)
{
    if (angle > 180) angle = 180;
    return servo->min_pulse + ((servo->max_pulse - servo->min_pulse) * angle) / 180;
}

void SERVO_Init(SERVO_Handle *servo)
{
    // 这里只做结构体参数初始化，硬件定时器初始化请在主程序或外部完成
    servo->current_angle = 90;
    SERVO_SetAngle(servo, 90); // 默认居中
}

void SERVO_SetAngle(SERVO_Handle *servo, uint8_t angle)
{
    uint16_t pulse = SERVO_AngleToPulse(servo, angle);
    uint32_t load = DL_Timer_getLoadValue(servo->timer_base);

    // 计算占空比
    uint32_t duty = (pulse * (load + 1)) / servo->period;
    if (duty > load) duty = load;

    DL_Timer_setCaptureCompareValue(servo->timer_base, duty, servo->pwm_channel);
    servo->current_angle = angle;
}