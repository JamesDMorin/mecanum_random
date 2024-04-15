#include <ESP32Servo.h>
#include <Arduino.h>
#include "util.h"
#include "wireless.h"
#include "gripper_pinout.h"
#include "UMS3.h"

UMS3 ums3;

Servo LeftServo;
Servo RightServo;

int pos = 0;      // position in degrees
ESP32PWM pwm;

extern GripperMessage gripperMessage;
extern GripperControllerMessage gripperControllerMessage;

int magnet_state = 0;
int gripper_state = 1;

// for testing lights
int r = 0;
int g = 0;
int b = 0;

// Setup Gripper Actuators and Sensors
void setupGripper(){
    ums3.begin();
    ums3.setPixelBrightness(255 / 4); // Brightness is 0-255. We set it to 1/4 brightness here

    LeftServo.attach(LEFT_SERVO_PIN, MIN_US, MAX_US);
    RightServo.attach(RIGHT_SERVO_PIN, MIN_US, MAX_US);
    pwm.attachPin(14, 10000); //10kHz
}

void updateCommand(){
    if (freshWirelessData) {
        magnet_state = gripperControllerMessage.magnet_command;
        gripper_state = gripperControllerMessage.gripper_command;
    }
}

void runGripperActions(){
    if (magnet_state) {
        //turn on magnet
        r = 255;

    } else {
        //turn off magnet
        r = 0;

    }

    if (gripper_state == 0) {
        // Gripper EM Mode
        b = 255;
        g = 0;

    } else if (gripper_state == 1) {
        // Gripper Open 
        b = 0;
        g = 255;
        
    } else if (gripper_state == 2) {
        // Gripper Close
        b = 255;
        g = 255;
        
    }

    ums3.setPixelColor(UMS3::color(r,g,b));

}
