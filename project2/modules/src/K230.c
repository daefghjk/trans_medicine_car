#include <string.h>
#include <stdlib.h>
#include "BOARD.h"

typedef enum {
    STATE_WAIT_START,
    STATE_WAIT_FLAG,
    STATE_RECEIVING_DATA
} UART_RxStateType;

uint8_t k230_rx_buffer[K230_MAX_BUFFER_SIZE];
uint8_t buffer_index = 0;
volatile float delta_angle = 0;
char float_str[K230_MAX_BUFFER_SIZE + 1];

void K230_ProcessRxData(uint8_t flag)
{
    switch (flag)
    {
        case K230_FLAG_DIR:
            if (buffer_index > 1) break;
            turn_dir =  k230_rx_buffer[0];
            break;
        
        case K230_FLAG_ANGLE:
            memcpy(float_str, k230_rx_buffer, buffer_index);
            float_str[buffer_index] = '\0';
            delta_angle = atof(float_str);
            break;

        default:
            break;
    }
}

void K230_RxCallback(void)
{
    static UART_RxStateType current_state = STATE_WAIT_START;
    static uint8_t rx_flag = 0;
    uint8_t rx_data = DL_UART_Main_receiveData(K230_INST);

    switch (current_state)
    {
        case STATE_WAIT_START:
            if (rx_data == K230_START_FRAME)
            {
                current_state = STATE_WAIT_FLAG;
                buffer_index = 0;
            }
            break;
        
        case STATE_WAIT_FLAG:
            rx_flag = rx_data;
            current_state = STATE_RECEIVING_DATA;
            break;

        case STATE_RECEIVING_DATA:
            if (rx_data == K230_END_FRAME)
            {
                K230_ProcessRxData(rx_flag);
                current_state = STATE_WAIT_START;
            }
            else
            {
                if (buffer_index < K230_MAX_BUFFER_SIZE)
                    k230_rx_buffer[buffer_index++] = rx_data;
                else
                    current_state = STATE_WAIT_START;
            }
            break;
    }
}

void K230_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(K230_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            K230_RxCallback();
            DL_UART_Main_clearInterruptStatus(K230_INST, DL_UART_MAIN_IIDX_RX);
            break;
        
        default:
            break;
    }
}

void K230_SendBytes(uint8_t buffer[], uint8_t length)
{
    for (uint8_t i = 0; i < length; ++i)
        DL_UART_Main_transmitDataBlocking(K230_INST, buffer[i]);
}

void K230_SendCmd(uint8_t cmd)
{
    uint8_t buffer[4] = {K230_START_FRAME, K230_FLAG_CMD, cmd, K230_END_FRAME};
    K230_SendBytes(buffer, 4);
}
