#include <Arduino.h>
#include "util.h"
#include "mecanum_robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "mecanum_robot_motion_control.h"
#include "mecanum_robot_pinout.h"

extern ControllerMessage controllerMessage;

void followTrajectory() {

    if (freshWirelessData) {
        // Serial.printf("Joystick1: %.2f, %.2f, Joystick2: %.2f, %.2f, BL: %u, BR:%u\n",
        //                 controllerMessage.joystick1.x, controllerMessage.joystick1.y,
        //                 controllerMessage.joystick2.x, controllerMessage.joystick2.y,
        //                 controllerMessage.buttonL, controllerMessage.buttonR);

        // map magnitude with 10% joystick deadzone
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(abs(controllerMessage.joystick1.y), 0.1, 1, 0, MAX_FORWARD); 
        // negate if needed
        forward = controllerMessage.joystick1.y > 0 ? forward : -forward;

        double sideways = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : -mapDouble(abs(controllerMessage.joystick1.x), 0.1, 1, 0, MAX_FORWARD);
        sideways = controllerMessage.joystick1.x > 0 ? sideways : -sideways;

        double rotation = abs(controllerMessage.joystick2.x) < 0.1 ? 0 : -mapDouble(abs(controllerMessage.joystick2.x), 0.1, 1, 0, MAX_ROTATE);
        rotation = controllerMessage.joystick2.x > 0 ? rotation : -rotation;
        // Serial.printf("forward (x): %.2f, sideways (y): %.2f, rotation (theta): %.2f\n", forward, sideways, rotation);
        updateSetpoints(forward, sideways, rotation);
    }
}

