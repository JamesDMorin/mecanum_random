#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "gripper_actions.h"

void setup() {
    Serial.begin(115200);
    setupGripper();
    setupWireless();
}

void loop() {
    // Update velocity setpoints based on trajectory at 50Hz
    EVERY_N_MILLIS(50) {
        updateCommand();
        runGripperActions();
    }

    // // Send and print robot values at 20Hz
    // EVERY_N_MILLIS(50) {
    //     updateOdometry();
    //     sendRobotData();

    //     Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
    //                 robotMessage.x, robotMessage.y, robotMessage.theta);
    // }
  
}