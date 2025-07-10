#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "KEY.h"

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    uint8_t Key_Num = 255;


    OLED_ShowFloat(1,1,1,1,1);
    while (1)
    {
        
        Key_Num = KEY_GetNum();
        if(Key_Num != 255)
        {
            OLED_ShowNum(2,1,Key_Num,3);
        }

    }
}
