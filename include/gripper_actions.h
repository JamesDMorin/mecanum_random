#ifndef GRIPPER_ACTIONS_H
#define GRIPPER_ACTIONS_H

// wheel radius in meters
// #define r 0.06

void setupGripper();
void updateCommand();
void runGripperActions();
void ClosedOnTim();
void OpenForTim();
void MagnetPosition();
void MagnetsOn();
void MagnetsOff();

#endif