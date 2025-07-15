#include "BOARD.h"

uint32_t CalculatePID(float delta)
{
    static float last_delta = 0;
    uint32_t res = Kp * delta + Kd * (delta - last_delta);
    last_delta = delta;
    return res;
}

void Find_Line()
{
    uint32_t delta_speed = CalculatePID(delta_angle);
    Motor_SetSpeed(&motor_left_front, motor_base_speed + delta_speed);
    Motor_SetSpeed(&motor_left_back, motor_base_speed + delta_speed);
    if (motor_base_speed - delta_speed < 0)
    {
        Motor_SetSpeed(&motor_right_front, 0);
        Motor_SetSpeed(&motor_right_back, 0);
    }
    else
    {
        Motor_SetSpeed(&motor_right_front, motor_base_speed - delta_speed);
        Motor_SetSpeed(&motor_right_back, motor_base_speed - delta_speed);
    }
}

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

        OLED_ShowFloat(3, 1, delta_angle, 2, 1);
        if (find_line_en) Find_Line();
    }
}
