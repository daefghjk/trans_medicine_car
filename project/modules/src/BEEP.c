#include "ti_msp_dl_config.h"
#include "DELAY.h"

extern uint64_t Systick_Count;

#define BEEP_PORT      GPIO_BEEP_PORT
#define BEEP_PIN       GPIO_BEEP_PIN

void BEEP_Init(void);

void BEEP_Shot(void)
{
    static uint64_t beep_start_tick = 0;
    static uint8_t beep_flag = 0;

    if (!beep_flag) {
        DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_BEEP_PIN); // 打开蜂鸣器
        beep_start_tick = Systick_Count;
        beep_flag = 1;
    }

    // 持续100ms
    if (beep_flag && (Systick_Count - beep_start_tick >= 100)) {
        DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_BEEP_PIN); // 关闭蜂鸣器
        beep_flag = 0;
    }
}

