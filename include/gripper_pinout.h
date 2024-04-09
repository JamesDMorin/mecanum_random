#ifndef GRIPPER_PINOUT_H
#define GRIPPER_PINOUT_H

#define LEFT_SERVO_PIN 13
#define RIGHT_SERVO_PIN 12

#define CLOSED_POS_BUTTON 5
#define OPEN_POS_BUTTON 4
#define MAGNET_POS_BUTTON 3

#define SERVO_ANGLE_CONVERSION 5/3

#define MIN_US 840
#define MAX_US 2060

#define OPEN_ANGLE 15

void ClosedOnTim();
void OpenForTim();
void MagnetPosition();

#endif // GRIPPER_PINOUT_H