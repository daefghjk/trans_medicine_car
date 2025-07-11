#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "KEY.h"

uint8_t Key_Num = 255;
uint64_t Systick_Count = 0;

void SysTick_Handler(void)
{
    Systick_Count++;
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    //启用SysTick中断
    SysTick_Config(CPUCLK_FREQ / 1000); // 1ms

    OLED_ShowFloat(1,1,1,1,1);
    while (1)
    {
        
        uint8_t key_event = KEY_GetNum();
        KEY_Act(key_event);
        OLED_ShowNum(2,1,key_event,10);
        OLED_ShowNum(3,1,Systick_Count,10);

    }
}
