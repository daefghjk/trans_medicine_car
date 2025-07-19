#include "ti_msp_dl_config.h"
#include "DELAY.h"


void Delay_ms(int ms)
{
    while (ms--)
        delay_cycles(CPUCLK_FREQ / 1000);
}

