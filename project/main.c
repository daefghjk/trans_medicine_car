#include "BOARD.h"

#define abs(x) (x > 0 ? x : -x)

int32_t CalculatePID(float delta)
{
    static float last_delta = 0;
    int32_t res = Kp * delta + Kd * (delta - last_delta);
    last_delta = delta;
    return res;
}

void Find_Line(void)
{
    int32_t delta_speed = CalculatePID(delta_angle);

    int32_t left_speed = motor_base_speed + delta_speed;
    int32_t right_speed = motor_base_speed - delta_speed;
    if (left_speed > 100) left_speed = 100;
    if (left_speed < 0) left_speed = 0;
    if (right_speed > 100) right_speed = 100;
    if (right_speed < 0) right_speed = 0;

    Motor_SetSpeed(&motor_left_front, left_speed);
    Motor_SetSpeed(&motor_left_back, left_speed);
    Motor_SetSpeed(&motor_right_front, right_speed);
    Motor_SetSpeed(&motor_right_back, right_speed);
}

void Move_Meter(float meter)
{
    int32_t current_left_count = 0, current_right_count = 0;
    uint8_t dir = meter > 0 ? 1 : 0;
    if (!dir) meter = -meter;
    uint64_t target_count = meter / (3.14159265 * WHELL_DIAMETER / ENCODER_PPR);

    OLED_ShowNum(2, 1, target_count, 10);

    Motor_SetAllDir(MOTOR_DIR_STOP);

    DL_GPIO_disableInterrupt(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN | GPIO_ENCODER_ENCODER_3_A_PIN);
    left_count = 0;
    right_count = 0;
    DL_GPIO_enableInterrupt(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN | GPIO_ENCODER_ENCODER_3_A_PIN);

    Motor_SetAllDir(MOTOR_DIR_FORWARD);

    while (1)
    {
        DL_GPIO_disableInterrupt(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN | GPIO_ENCODER_ENCODER_3_A_PIN);
        current_left_count = left_count;
        current_right_count = right_count;
        DL_GPIO_enableInterrupt(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN | GPIO_ENCODER_ENCODER_3_A_PIN);

        OLED_ShowSignedNum(3, 1, current_left_count, 10);
        OLED_ShowSignedNum(4, 1, current_right_count, 10);

        if ((abs(current_left_count) + abs(current_right_count)) / 2 >= target_count)
            break;

        int32_t delta = 0.5 * (current_left_count - current_right_count);

        delta = 0;

        int32_t left_speed = motor_base_speed - delta;
        int32_t right_speed = motor_base_speed + delta;
        if (left_speed > 100) left_speed = 100;
        if (left_speed < 0) left_speed = 0;
        if (right_speed > 100) right_speed = 100;
        if (right_speed < 0) right_speed = 0;

        Motor_SetSpeed(&motor_left_front, left_speed);
        Motor_SetSpeed(&motor_left_back, left_speed);
        Motor_SetSpeed(&motor_right_front, right_speed);
        Motor_SetSpeed(&motor_right_back, right_speed);

        Delay_ms(10);
    }

    Motor_SetAllDir(MOTOR_DIR_STOP);
}

// angle_deg 大于0向右转
void Rotate_Angle(float angle_deg)
{
    int32_t current_left_count = 0, current_right_count = 0;
    uint8_t dir = angle_deg > 0 ? 1 : 0; //正为右
    if (!dir) angle_deg = -angle_deg;
    float angle_rad = angle_deg * (3.14159265 / 180);
    uint64_t target_count = angle_rad * ROTARY_SHAFT_DIS / (3.14159265 * WHELL_DIAMETER / ENCODER_PPR);

    OLED_ShowNum(2, 1, target_count, 10);

    Motor_SetAllDir(MOTOR_DIR_STOP);

    DL_GPIO_disableInterrupt(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN | GPIO_ENCODER_ENCODER_3_A_PIN);
    left_count = 0;
    right_count = 0;
    DL_GPIO_enableInterrupt(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN | GPIO_ENCODER_ENCODER_3_A_PIN);

    if (dir)
    {
        Motor_SetDirection(&motor_left_front, MOTOR_DIR_FORWARD);
        Motor_SetDirection(&motor_left_back, MOTOR_DIR_FORWARD);
        Motor_SetDirection(&motor_right_front, MOTOR_DIR_BACKWARD);
        Motor_SetDirection(&motor_right_back, MOTOR_DIR_BACKWARD);
        Motor_SetSpeed(&motor_left_front, 60);
        Motor_SetSpeed(&motor_right_front, 50);
        Motor_SetSpeed(&motor_left_back, 60);
        Motor_SetSpeed(&motor_right_back, 50);
    }
    else
    {
        Motor_SetDirection(&motor_left_front, MOTOR_DIR_BACKWARD);
        Motor_SetDirection(&motor_left_back, MOTOR_DIR_BACKWARD);
        Motor_SetDirection(&motor_right_front, MOTOR_DIR_FORWARD);
        Motor_SetDirection(&motor_right_back, MOTOR_DIR_FORWARD);
        Motor_SetSpeed(&motor_left_front, 50);
        Motor_SetSpeed(&motor_right_front, 60);
        Motor_SetSpeed(&motor_left_back, 50);
        Motor_SetSpeed(&motor_right_back, 60);
    }



    while (1)
    {
        DL_GPIO_disableInterrupt(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN | GPIO_ENCODER_ENCODER_3_A_PIN);
        current_left_count = left_count;
        current_right_count = right_count;
        DL_GPIO_enableInterrupt(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_0_A_PIN | GPIO_ENCODER_ENCODER_3_A_PIN);
        OLED_ShowSignedNum(3, 1, current_left_count, 10);
        OLED_ShowSignedNum(4, 1, current_right_count, 10);

        if ((abs(current_left_count) >= target_count &&  abs(current_right_count) >= target_count))
            break;
    }

    Motor_SetAllDir(MOTOR_DIR_STOP);
}

int main(void)
{
    Board_Init();

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
        
        OLED_ShowChar(1, 1, turn_dir);

        switch (turn_dir)
        {
            case 's':
                Motor_SetAllDir(MOTOR_DIR_STOP);
                find_line_en = 0;
                break;
            case 'f':
                Motor_SetAllDir(MOTOR_DIR_FORWARD);
                find_line_en = 1;
                break;
            case 'l':
                DL_UART_Main_disableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
                find_line_en = 0;
                Move_Meter(0.3);
                Rotate_Angle(-90);
                Motor_SetAllDir(MOTOR_DIR_FORWARD);
                find_line_en = 1;

                DL_UART_Main_enableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
                break;
            case 'r':
                DL_UART_Main_disableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
                find_line_en = 0;
                Move_Meter(0.3);
                Rotate_Angle(90);
                Motor_SetAllDir(MOTOR_DIR_FORWARD);
                turn_dir = 'f';
                find_line_en = 1;
                DL_UART_Main_enableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
                break;
            case 'b':
                DL_UART_Main_disableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
                find_line_en = 0;
                Rotate_Angle(180);
                Motor_SetAllDir(MOTOR_DIR_FORWARD);
                turn_dir = 'f';
                find_line_en = 1;
                DL_UART_Main_enableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
                break;
            default:
                break;
        }

        if (find_line_en) Find_Line();
    }
}
