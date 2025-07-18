#ifndef __K230_H__
#define __K230_H__

#define K230_START_FRAME 0xff
#define K230_END_FRAME   0xfe

#define K230_FLAG_DIR   0x00
#define K230_FLAG_ANGLE 0x01

#define K230_FLAG_CMD   0x00

#define K230_MAX_BUFFER_SIZE 30

void K230_SendBytes(uint8_t buffer[], uint8_t length);
void K230_SendCmd(uint8_t cmd);

#endif
