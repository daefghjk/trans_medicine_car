#ifndef __SERVO_H
#define __SERVO_H

void Servo_SetAngle(float Angle);

void Servo_Angle_Limit(void);

void Servo_Turn_Left(float Angle_Turn);
void Servo_Turn_Right(float Angle_Turn);
void Servo_Turn_Stright(float Angle_Turn);
void Servo_Angle_Turn(float Angle_Turn);

#endif
