#include <ti/driverlib/dl_timera.h>
#include <ti/driverlib/dl_timerg.h>
#include <ti/driverlib/dl_gpio.h>
#include "MOTOR.h"

void PWM_SetDuty(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex, uint32_t duty_percent)
{
    uint32_t load = DL_Timer_getLoadValue(gpt);
    uint32_t outCtl = DL_Timer_getCaptureCompareOutCtl(gpt, ccIndex);
    DL_TIMER_COUNT_MODE mode = DL_Timer_getCounterMode(gpt);

    uint8_t isInverted = (outCtl & DL_TIMER_CC_OCTL_INV_OUT_ENABLED);
    uint8_t initHigh = (outCtl & DL_TIMER_CC_OCTL_INIT_VAL_HIGH);

    uint8_t highMeansOn = (initHigh != isInverted); //高电平是否为有效，即在cmpx前是否输出高电平

    if (duty_percent > 100) duty_percent = 100;
    uint32_t highTime = (load * duty_percent) / 100;

    uint32_t cmpx;

    switch (mode)
    {
        case DL_TIMER_COUNT_MODE_UP:
            cmpx = highMeansOn ? highTime : (load - highTime);
            break;

        case DL_TIMER_COUNT_MODE_DOWN:
            cmpx = highMeansOn ? (load - highTime) : highTime;
            break;

        case DL_TIMER_COUNT_MODE_UP_DOWN:
            highTime = (2 * load * duty_percent) / 100;
            cmpx = highMeansOn ? (highTime / 2) : (load - highTime / 2);
            break;

        default:
            // 默认按 DOWN 模式处理
            cmpx = highMeansOn ? (load - highTime) : highTime;
            break;
    }
}

void Motor_SetSpeed(Motor_Handle *motor, uint32_t percent)
{
    if (percent > 100) percent = 100;

    uint32_t effective_percent;
    switch (motor->current_dir)
    {
        case MOTOR_DIR_FORWARD:
        case MOTOR_DIR_STOP:
            effective_percent = percent;
        break;

        case MOTOR_DIR_BACKWARD:
            effective_percent = 100 - percent;

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
            DL_GPIO_clearPins(motor->in2_port, motor->in2_pin);  // IN2 = 0
            break;
        case MOTOR_DIR_BACKWARD:
            DL_GPIO_setPins(motor->in2_port, motor->in2_pin);   // IN2 = 1
            break;
        case MOTOR_DIR_STOP:
        default:
            DL_GPIO_clearPins(motor->in2_port, motor->in2_pin);  // IN2 = 0
            Motor_SetSpeed(motor, 0);
            break;
    }
}
