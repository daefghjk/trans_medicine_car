#include <ti/driverlib/dl_timera.h>
#include <ti/driverlib/dl_timerg.h>
#include <ti/driverlib/dl_gpio.h>
#include "MOTOR.h"

void PWM_SetDuty(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex, uint32_t duty_percent)
{
    uint32_t load = DL_Timer_getLoadValue(gpt);
    uint32_t outCtl = DL_Timer_getCaptureCompareOutCtl(gpt, ccIndex);
    DL_TIMER_COUNT_MODE mode = DL_Timer_getCounterMode(gpt);

    // 是否翻转，不翻转在cmpx前是高电平，翻转在cmpx前是低电平
    uint8_t isInverted = (outCtl & DL_TIMER_CC_OCTL_INV_OUT_ENABLED);

    if (duty_percent > 100) duty_percent = 100;
    uint32_t highTime = ((load + 1) * duty_percent) / 100;

    uint32_t cmpx;

    switch (mode)
    {
        case DL_TIMER_COUNT_MODE_UP:
            cmpx = isInverted ? (load + 1 - highTime) : highTime;
            break;

        case DL_TIMER_COUNT_MODE_DOWN:
            cmpx = isInverted ? highTime : (load + 1 - highTime);
            break;

        case DL_TIMER_COUNT_MODE_UP_DOWN:
            highTime = (2 * (load + 1) * duty_percent) / 100;
            cmpx = isInverted ? (load + 1 - highTime / 2) : (highTime / 2);
            break;

        default:
            // 默认按 DOWN 模式处理
            cmpx = isInverted ? highTime : (load + 1 - highTime);
            break;
    }

    DL_Timer_setCaptureCompareValue(gpt, cmpx - 1, ccIndex);
}

void Motor_SetSpeed(Motor_Handle *motor, int16_t percent)
{
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    motor->current_speed = percent;

    uint32_t effective_percent;
    switch (motor->current_dir)
    {
        case MOTOR_DIR_FORWARD:
        case MOTOR_DIR_STOP:
            effective_percent = percent;
        break;

        case MOTOR_DIR_BACKWARD:
            effective_percent = 100 - percent;
        break;

        default:
            effective_percent = percent;
        break;
    }

    PWM_SetDuty(motor->timer_base, motor->pwm_channel, effective_percent);
}

void Motor_SetDirection(Motor_Handle *motor, Motor_DirectionType dir)
{
    motor->current_dir = dir;

    switch (dir)
    {
        case MOTOR_DIR_FORWARD:
            DL_GPIO_clearPins(motor->in2_port, motor->in2_pin);
            Motor_SetSpeed(motor, motor->current_speed);
            break;
        case MOTOR_DIR_BACKWARD:
            DL_GPIO_setPins(motor->in2_port, motor->in2_pin);
            Motor_SetSpeed(motor, motor->current_speed);
            break;
        case MOTOR_DIR_STOP:
        default:
            DL_GPIO_clearPins(motor->in2_port, motor->in2_pin);
            Motor_SetSpeed(motor, 0);
            break;
    }
}

void Motor_Init(Motor_Handle *motor)
{
    Motor_SetDirection(motor, motor->current_dir);
    Motor_SetSpeed(motor, motor->current_speed);
}
