#include "BOARD.h"

typedef enum {
    STATE_WAIT_START,
    STATE_RECEIVING_DATA
} UART_RxStateType;

uint8_t ble_rx_buffer[BLE_MAX_BUFFER_SIZE];
uint8_t ble_rx_buffer_index = 0;

void BLE_ProcessRxData(void)
{
    if (ble_rx_buffer_index > 1) return;
    ble_flag =  ble_rx_buffer[0];
}

void BLE_RxCallback(void)
{
    static UART_RxStateType current_state = STATE_WAIT_START;
    uint8_t rx_data = DL_UART_Main_receiveData(UART_1_INST);

    switch (current_state)
    {
        case STATE_WAIT_START:
            if (rx_data == BLE_START_FRAME)
            {
                current_state = STATE_RECEIVING_DATA;
                ble_rx_buffer_index = 0;
            }
            break;

        case STATE_RECEIVING_DATA:
            if (rx_data == BLE_END_FRAME)
            {
                BLE_ProcessRxData();
                current_state = STATE_WAIT_START;
            }
            else
            {
                if (ble_rx_buffer_index < BLE_MAX_BUFFER_SIZE)
                    ble_rx_buffer[ble_rx_buffer_index++] = rx_data;
                else
                    current_state = STATE_WAIT_START;
            }
            break;
    }
}

void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            BLE_RxCallback();
            DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_IIDX_RX);
            break;
        
        default:
            break;
    }
}

void BLE_SendBytes(uint8_t buffer[], uint8_t length)
{
    for (uint8_t i = 0; i < length; ++i)
        DL_UART_Main_transmitDataBlocking(UART_1_INST, buffer[i]);
}

void BLE_SendCmd(uint8_t cmd)
{
    uint8_t buffer[3] = {BLE_START_FRAME, cmd, BLE_END_FRAME};
    BLE_SendBytes(buffer, 3);
}

