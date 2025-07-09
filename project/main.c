#include "ti_msp_dl_config.h"
#include "OLED.h"

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ShowChar(1, 1, '1');


    while (1)
    {
        
    }
}
