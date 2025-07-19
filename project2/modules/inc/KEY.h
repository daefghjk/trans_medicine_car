#ifndef __KEY_H_
#define __KEY_H_
#include "ti_msp_dl_config.h"
#include "DELAY.h"

void KEY_Init(void);
uint8_t KEY_GetNum(void);
void KEY_Act(uint8_t Key_Num);

// 单击事件（弱定义，用户可重写）
void KEY_Act0_Click(void)      __attribute__((weak));
void KEY_Act1_Click(void)      __attribute__((weak));
void KEY_Act2_Click(void)      __attribute__((weak));
void KEY_Act3_Click(void)      __attribute__((weak));
void KEY_Act4_Click(void)      __attribute__((weak));
void KEY_Act5_Click(void)      __attribute__((weak));

// 双击事件（弱定义，用户可重写）
void KEY_Act0_DoubleClick(void) __attribute__((weak));
void KEY_Act1_DoubleClick(void) __attribute__((weak));
void KEY_Act2_DoubleClick(void) __attribute__((weak));
void KEY_Act3_DoubleClick(void) __attribute__((weak));
void KEY_Act4_DoubleClick(void) __attribute__((weak));
void KEY_Act5_DoubleClick(void) __attribute__((weak));

// 长按事件（弱定义，用户可重写）
void KEY_Act0_LongPress(void)  __attribute__((weak));
void KEY_Act1_LongPress(void)  __attribute__((weak));
void KEY_Act2_LongPress(void)  __attribute__((weak));
void KEY_Act3_LongPress(void)  __attribute__((weak));
void KEY_Act4_LongPress(void)  __attribute__((weak));
void KEY_Act5_LongPress(void)  __attribute__((weak));

#endif