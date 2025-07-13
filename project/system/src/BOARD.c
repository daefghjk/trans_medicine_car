#include "BOARD.h"

uint8_t Key_Num = 255;
uint64_t Systick_Count = 0;

Motor_Handle motor0 = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_0_PIN,
    .timer_type = MOTOR_TIMER_TIMA,
    .timer_base = MOTOR_INST,
    .pwm_channel = GPIO_MOTOR_C0_IDX,
    .current_dir = MOTOR_DIR_FORWARD,
    .current_speed = 0
};
Motor_Handle motor1 = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_1_PIN,
    .timer_type = MOTOR_TIMER_TIMA,
    .timer_base = MOTOR_INST,
    .pwm_channel = GPIO_MOTOR_C1_IDX,
    .current_dir = MOTOR_DIR_FORWARD,
    .current_speed = 0
};
Motor_Handle motor2 = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_2_PIN,
    .timer_type = MOTOR_TIMER_TIMA,
    .timer_base = MOTOR_INST,
    .pwm_channel = GPIO_MOTOR_C2_IDX,
    .current_dir = MOTOR_DIR_FORWARD,
    .current_speed = 0
};
Motor_Handle motor3 = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_3_PIN,
    .timer_type = MOTOR_TIMER_TIMA,
    .timer_base = MOTOR_INST,
    .pwm_channel = GPIO_MOTOR_C3_IDX,
    .current_dir = MOTOR_DIR_FORWARD,
    .current_speed = 0
};

void SysTick_Handler(void)
{
    Systick_Count++;
}

void Board_Init(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    Motor_Init(&motor0);
    Motor_Init(&motor1);
    Motor_Init(&motor2);
    Motor_Init(&motor3);
    SysTick_Config(CPUCLK_FREQ / 1000);
    DL_TimerA_startCounter(MOTOR_INST);
    DL_TimerA_startCounter(SERVO_INST);
}
