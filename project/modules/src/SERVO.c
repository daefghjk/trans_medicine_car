#include "ti_msp_dl_config.h"
#include "SERVO.h"

float Servo_Angle_TurnMin;//每�?�变化的角度
float Servo_Angle;


void Servo_SetAngle(float Angle)	//设置角度,0-180
{
    if(Angle >= 180)
    {
        Servo_SetAngle(Angle - 180); 
    }
    else if (Angle < 0) 
    {
        Servo_SetAngle(Angle + 180);
    }
    else
    {
        Servo_Angle = Angle;
        DL_TimerA_setCaptureCompareValue(SERVO_INST, 975 - Angle * 100 / 180, GPIO_SERVO_C1_IDX);
    }
}

void Servo_Angle_Limit(void)		//限制角度��?60-120
{
	if(Servo_Angle >= 120)
	{
		Servo_Angle = 120;
	}
	
	if(Servo_Angle <= 60)
	{
		Servo_Angle = 60;
	}
}

void Servo_Turn_Right(float Angle_Turn)
{
	Servo_Angle += Angle_Turn;
	Servo_Angle_Limit();//限幅
	Servo_SetAngle(Servo_Angle);
}

void Servo_Turn_Left(float Angle_Turn)
{
	Servo_Angle -= Angle_Turn;
	Servo_Angle_Limit();//限幅
	Servo_SetAngle(Servo_Angle);
}

void Servo_Turn_Stright(float Angle_Turn)
{
	if(Servo_Angle > 90)
	{
		Servo_Turn_Left(Angle_Turn);
	}
	
	else if(Servo_Angle < 90)
	{
		Servo_Turn_Right(Angle_Turn);
	}
	
	Servo_SetAngle(Servo_Angle);
}

//ת��
void Servo_Angle_Turn(float Angle_Turn)
{
	if(Servo_Angle < 0)
	{
		Servo_Turn_Left(Angle_Turn);
	}
	
	else if(Servo_Angle > 0)
	{
		Servo_Turn_Right(Angle_Turn);
	}
}

