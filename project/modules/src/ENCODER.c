#include "BOARD.h"

volatile int32_t left_count = 0;
volatile int32_t right_count = 0;

void GROUP1_IRQHandler(void)
{
    uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN | GPIO_ENCODER_ENCODER_3_A_PIN);

    if ((gpioB & GPIO_ENCODER_ENCODER_0_A_PIN) == GPIO_ENCODER_ENCODER_0_A_PIN)
    {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_B_PIN))
            left_count--;
        else
            left_count++;
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN);
    }
    else if ((gpioB & GPIO_ENCODER_ENCODER_3_A_PIN) == GPIO_ENCODER_ENCODER_3_A_PIN)
    {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_3_B_PIN))
            right_count--;
        else
            right_count++;
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_3_A_PIN);
    }
}
