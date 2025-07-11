#include "ti_msp_dl_config.h"
#include "KEY.h"
#include "DELAY.h"

extern uint64_t Systick_Count;

void KEY_Init(void);



#define KEY_NONE    255

/**
 * @brief   扫描并检测按键事件，支持单击、双击和长按。
 * @note    不同按键和按键类型返回唯一值。
 * @retval  0~5: 单击按键0~5
 *          100~105: 双击按键0~5
 *          200~205: 长按按键0~5
 *          255: 未检测到按键事件
 */
uint8_t KEY_GetNum(void)
{
    static uint8_t key_state[6] = {0};
    static uint64_t key_down_tick[6] = {0};
    static uint64_t key_up_tick[6] = {0};
    static uint8_t click_count[6] = {0};
    uint32_t KEY_Pin[6] = {GPIO_KEY_KEY_0_PIN, GPIO_KEY_KEY_1_PIN, GPIO_KEY_KEY_2_PIN, 
                            GPIO_KEY_KEY_3_PIN, GPIO_KEY_KEY_4_PIN, GPIO_KEY_KEY_5_PIN};
    for(uint8_t i = 0; i < 6; i++)
    {
        uint8_t is_down = (DL_GPIO_readPins(GPIO_KEY_PORT, KEY_Pin[i]) == 0);
        switch(key_state[i])
        {
            case 0: // 空闲
                if(is_down)
                {
                    key_state[i] = 1;
                    key_down_tick[i] = Systick_Count;
                }
                break;
            case 1: // 按下
                if(!is_down)
                {
                    key_state[i] = 2;
                    key_up_tick[i] = Systick_Count;
                    click_count[i]++;
                }
                else if(Systick_Count - key_down_tick[i] > 800) // 长按800ms
                {
                    key_state[i] = 0;
                    click_count[i] = 0;
                    return (i + 200); // 长按返回
                }
                break;
            case 2: // 抬起，等待双击
                if(is_down)
                {
                    key_state[i] = 1;
                    key_down_tick[i] = Systick_Count;
                }
                else if(Systick_Count - key_up_tick[i] > 300) // 300ms内无第二次按下
                {
                    key_state[i] = 0;
                    if(click_count[i] == 1)
                    {
                        click_count[i] = 0;
                        return i; // 单击返回
                    }
                    else if(click_count[i] == 2)
                    {
                        click_count[i] = 0;
                        return (i + 100); // 双击返回
                    }
                    click_count[i] = 0;
                }
                break;
        }
    }
    return KEY_NONE;
}

// 根据 KEY_GetNum 的返回值，区分单击、双击、长按，并分别调用不同的处理函数

void KEY_Act(uint8_t Key_Num)
{
    if (Key_Num == 255) return;

    uint8_t key_index = Key_Num % 100;
    if (Key_Num < 100) {
        // 单击
        switch (key_index) {
            case 0: KEY_Act0_Click(); break;
            case 1: KEY_Act1_Click(); break;
            case 2: KEY_Act2_Click(); break;
            case 3: KEY_Act3_Click(); break;
            case 4: KEY_Act4_Click(); break;
            case 5: KEY_Act5_Click(); break;
            default: break;
        }
    } else if (Key_Num < 200) {
        // 双击
        switch (key_index) {
            case 0: KEY_Act0_DoubleClick(); break;
            case 1: KEY_Act1_DoubleClick(); break;
            case 2: KEY_Act2_DoubleClick(); break;
            case 3: KEY_Act3_DoubleClick(); break;
            case 4: KEY_Act4_DoubleClick(); break;
            case 5: KEY_Act5_DoubleClick(); break;
            default: break;
        }
    } else if (Key_Num < 255) {
        // 长按
        switch (key_index) {
            case 0: KEY_Act0_LongPress(); break;
            case 1: KEY_Act1_LongPress(); break;
            case 2: KEY_Act2_LongPress(); break;
            case 3: KEY_Act3_LongPress(); break;
            case 4: KEY_Act4_LongPress(); break;
            case 5: KEY_Act5_LongPress(); break;
            default: break;
        }
    }
}

// 弱定义，用户可在其他文件重写
void KEY_Act0_Click(void)      __attribute__((weak));
void KEY_Act1_Click(void)      __attribute__((weak));
void KEY_Act2_Click(void)      __attribute__((weak));
void KEY_Act3_Click(void)      __attribute__((weak));
void KEY_Act4_Click(void)      __attribute__((weak));
void KEY_Act5_Click(void)      __attribute__((weak));
void KEY_Act0_DoubleClick(void) __attribute__((weak));
void KEY_Act1_DoubleClick(void) __attribute__((weak));
void KEY_Act2_DoubleClick(void) __attribute__((weak));
void KEY_Act3_DoubleClick(void) __attribute__((weak));
void KEY_Act4_DoubleClick(void) __attribute__((weak));
void KEY_Act5_DoubleClick(void) __attribute__((weak));
void KEY_Act0_LongPress(void)  __attribute__((weak));
void KEY_Act1_LongPress(void)  __attribute__((weak));
void KEY_Act2_LongPress(void)  __attribute__((weak));
void KEY_Act3_LongPress(void)  __attribute__((weak));
void KEY_Act4_LongPress(void)  __attribute__((weak));
void KEY_Act5_LongPress(void)  __attribute__((weak));

/*
示例用法：

// 在主循环中调用
void loop(void)
{
    uint8_t key_event = KEY_GetNum();
    KEY_Act(key_event);
}

// 用户可在其他文件实现具体响应
void KEY_Act0_Click(void) {
    // 按键0单击事件处理
}
void KEY_Act0_DoubleClick(void) {
    // 按键0双击事件处理
}
void KEY_Act0_LongPress(void) {
    // 按键0长按事件处理
}
*/

