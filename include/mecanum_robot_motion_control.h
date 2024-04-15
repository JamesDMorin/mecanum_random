#ifndef MECANUM_ROBOT_MOTION_CONTROL_H
#define MECANUM_ROBOT_MOTION_CONTROL_H

// wheel radius in meters
#define r 0.06
// distance from back wheel to center in meters
#define b 0.2

void setupIMU();
void followTrajectory();
void updateOdometry();

#endif