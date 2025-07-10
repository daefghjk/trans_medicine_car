#include "ti_msp_dl_config.h"
#include "KEY.h"
#include "DELAY.h"

void KEY_Init(void);

uint8_t KEY_GetNum(void)
{
    uint8_t Key_Num = 255;

    uint32_t KEY_Pin[6] = {GPIO_KEY_KEY_0_PIN, GPIO_KEY_KEY_1_PIN, GPIO_KEY_KEY_2_PIN, GPIO_KEY_KEY_3_PIN, GPIO_KEY_KEY_4_PIN, GPIO_KEY_KEY_5_PIN};

    for(uint8_t i = 0; i < 6; i++)
    {
        if(DL_GPIO_readPins(GPIO_KEY_PORT, KEY_Pin[i]) == 0)
        {
            Delay_ms(20);
            while(DL_GPIO_readPins(GPIO_KEY_PORT, KEY_Pin[i]) == 0);
            Delay_ms(20);
            Key_Num = i;
        }
    }

    return Key_Num;
}

// void KEY_Act(uint8_t Key_Num)
// {
//     if (Key_Num == 1)
//     {
//         KEY_Act1();
//     }

//     if (Key_Num == 2)
//     {
//         KEY_Act2();
//     }

// }
// void KEY_Act1(void);
// void KEY_Act2(void);
// __weak void KEY_Act1(void);
// __weak void KEY_Act2(void);

