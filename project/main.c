#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "KEY.h"
#include "DELAY.h"
#include "MOTOR.h"

uint8_t Key_Num = 255;
uint64_t Systick_Count = 0;

void SysTick_Handler(void)
{
    Systick_Count++;
}

Motor_Handle motor1 = {
    .in2_port = GPIO_MOTOR_DIR_PORT,
    .in2_pin = GPIO_MOTOR_DIR_MOTOR_0_PIN,
    .timer_type = MOTOR_TIMER_TIMA,
    .timer_base = MOTOR_INST,
    .pwm_channel = GPIO_MOTOR_C0_IDX,
    .current_dir = MOTOR_DIR_FORWARD,
    .current_speed = 0
};

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    SysTick_Config(CPUCLK_FREQ / 1000);
    OLED_ShowChar(1, 1, 's');
    DL_TimerA_startCounter(MOTOR_INST);
    while (1)
    {
        Key_Num = KEY_GetNum();
        if(Key_Num != 255)
        {
            OLED_ShowNum(2,1,Key_Num,3);
        }
    }
}
