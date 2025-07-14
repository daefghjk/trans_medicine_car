#include "BOARD.h"

uint8_t Key_Num = 255;
uint64_t Systick_Count = 0;
float Kp = 0.01, Kd = 0.01;
uint32_t motor_base_speed = 50;

Motor_Handle motor_left_front = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_0_PIN,
    .timer_type = MOTOR_TIMER_TIMA,
    .timer_base = MOTOR_INST,
    .pwm_channel = GPIO_MOTOR_C0_IDX,
    .current_dir = MOTOR_DIR_FORWARD,
    .current_speed = 0
};
Motor_Handle motor_left_back = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_1_PIN,
    .timer_type = MOTOR_TIMER_TIMA,
    .timer_base = MOTOR_INST,
    .pwm_channel = GPIO_MOTOR_C1_IDX,
    .current_dir = MOTOR_DIR_FORWARD,
    .current_speed = 0
};
Motor_Handle motor_right_back = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_2_PIN,
    .timer_type = MOTOR_TIMER_TIMA,
    .timer_base = MOTOR_INST,
    .pwm_channel = GPIO_MOTOR_C2_IDX,
    .current_dir = MOTOR_DIR_FORWARD,
    .current_speed = 0
};
Motor_Handle motor_right_front = {
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
    Motor_Init(&motor_left_front);
    Motor_Init(&motor_left_back);
    Motor_Init(&motor_right_back);
    Motor_Init(&motor_right_front);
    SysTick_Config(CPUCLK_FREQ / 1000);
    DL_TimerA_startCounter(MOTOR_INST);
}
