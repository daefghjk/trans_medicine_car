#ifndef __MOTOR_H__
#define __MOTOR_H__

typedef enum {
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD,
    MOTOR_DIR_STOP
} Motor_DirectionType;

typedef enum {
    MOTOR_TIMER_TIMG,
    MOTOR_TIMER_TIMA
} Motor_TimerType;

typedef struct {
    GPIO_Regs * in2_port;
    uint32_t in2_pin;

    GPTIMER_Regs * timer_base;
    DL_TIMER_CC_INDEX pwm_channel;
    Motor_TimerType timer_type;

    Motor_DirectionType current_dir;
    uint32_t current_speed;
} Motor_Handle;

void Motor_Init(Motor_Handle *motor);
void Motor_SetDirection(Motor_Handle *motor, Motor_DirectionType dir);
void Motor_SetSpeed(Motor_Handle *motor, int16_t percent);

#endif