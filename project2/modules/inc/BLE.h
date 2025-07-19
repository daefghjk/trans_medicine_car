#ifndef __BLE_H__
#define __BLE_H__

#define BLE_START_FRAME 0xff
#define BLE_END_FRAME   0xfe

#define BLE_MAX_BUFFER_SIZE     30

void BLE_SendBytes(uint8_t buffer[], uint8_t length);
void BLE_SendCmd(uint8_t cmd);

#endif
