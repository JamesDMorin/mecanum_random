#include <Arduino.h>
#include "util.h"
#include "mecanum_robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "mecanum_robot_motion_control.h"
#include "mecanum_robot_pinout.h"
#include "EulerAngles.h"

// #define UTURN
// #define CIRCLE
#define JOYSTICK
// #define YOUR_TRAJECTORY

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;

int state = 0;
double robotVelocity = 0; // velocity of robot, in m/s
double k = 0; // k is 1/radius from center of rotation circle

extern EncoderVelocity encoders[NUM_MOTORS];


// Makes robot follow a trajectory
void followTrajectory() {

    #ifdef JOYSTICK
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
    #endif 

    #ifdef CIRCLE
    updateSetpoints(0.0, 0.0, 3.0);
    #endif 

    #ifdef UTURN
    switch (state) {
        case 0: 
            // Until robot has achieved a x translation of 1m
            if (robotMessage.x <= 1.0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        case 1:
            // Until robot has achieved a 180 deg turn in theta
            if (robotMessage.theta <= M_PI) {
                // Turn in a circle with radius 25cm 
                robotVelocity = 0.2;
                k = 1/0.25;
            } else {
                state++;
            }
            break;

        case 2:
            // Until robot has achieved a x translation of -1m
            if (robotMessage.x >= 0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        default: 
            // If none of the states, robot should just stop
            robotVelocity = 0;
            k = 0;
            break;
    }
    #endif

    #ifdef YOUR_TRAJECTORY
    // TODO: Create a state machine to define your custom trajectory!

    #endif 

}

