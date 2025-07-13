#include "BOARD.h"

int main(void)
{
    Board_Init();

    OLED_ShowChar(1, 1, 'K');
    while (1)
    {
        Key_Num = KEY_GetNum();
        KEY_Act(Key_Num);
        if(Key_Num != 255)
        {
            OLED_ShowNum(2,1,Key_Num,3);
            Servo_SetAngle( Key_Num*20);
            // OLED_ShowNum(3,2,servo1.current_angle,3);
        }
        Key_Num = 255;
    }
}

void KEY_Act0_Click(void)
{
    if (motor0.current_speed)
        Motor_SetDirection(&motor0, MOTOR_DIR_STOP);
    else
    {
        Motor_SetDirection(&motor0, MOTOR_DIR_FORWARD);
        Motor_SetSpeed(&motor0, 50);
    }
}
