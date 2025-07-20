#include "BOARD.h"

uint8_t Key_Num = 255;
volatile uint64_t Systick_Count = 0;
const float Kp = 0.45, Kd = 1.5;
const uint32_t motor_base_speed = 45;
volatile uint8_t find_line_en = 0;
volatile uint8_t turn_dir = '0';
volatile uint8_t ble_flag = '0'; //从小车1接收到的标志位

Motor_Handle motor_left = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_L_PIN,
    .timer_type = MOTOR_TIMER_TIMA,
    .timer_base = MOTOR_INST,
    .pwm_channel = GPIO_MOTOR_C2_IDX,
    .current_dir = MOTOR_DIR_FORWARD,
    .current_speed = 0
};
Motor_Handle motor_right = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_R_PIN,
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
    Motor_Init(&motor_left);
    Motor_Init(&motor_right);
    SysTick_Config(CPUCLK_FREQ / 1000);
    DL_TimerA_startCounter(MOTOR_INST);
    DL_TimerA_startCounter(SERVO_INST);
    DL_UART_Main_enableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);
}

void Motor_SetAllDir(Motor_DirectionType dir)
{
    Motor_SetDirection(&motor_left, dir);
    Motor_SetDirection(&motor_right, dir);
}

void Motor_SetAllSpeed(uint16_t percent)
{
    Motor_SetSpeed(&motor_left, percent);
    Motor_SetSpeed(&motor_right, percent);
}
