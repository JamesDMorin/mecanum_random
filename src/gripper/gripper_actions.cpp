#include <ESP32Servo.h>
#include <Arduino.h>
#include "util.h"
#include "wireless.h"
#include "gripper_pinout.h"
#include "UMS3.h"
#include "gripper_actions.h"

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

    pinMode(RIGHT_MAGNET_PIN, OUTPUT);
    pinMode(LEFT_MAGNET_PIN, OUTPUT);
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
        MagnetsOn();
    } else {
        //turn off magnet
        r = 0;
        MagnetsOff();
    }

    if (gripper_state == 0) {
        // Gripper EM Mode
        b = 255;
        g = 0;
        MagnetPosition();

    } else if (gripper_state == 1) {
        // Gripper Open 
        b = 0;
        g = 255;
        MagnetsOff();
        OpenForTim();
        
    } else if (gripper_state == 2) {
        // Gripper Close
        b = 255;
        g = 255;
        MagnetsOff();
        ClosedOnTim();
        
    }

    ums3.setPixelColor(UMS3::color(r,g,b));

}

void ClosedOnTim() {
    // MagnetsOff();
    gripper_state = 2;
    RightServo.write(0);
    LeftServo.write(180);
    Serial.print("Gripper closing onto Tim (2 sec)");
    delay(1000);
}

void OpenForTim() {
    // MagnetsOff();
    gripper_state = 1;
    RightServo.write(0 + OPEN_ANGLE);
    LeftServo.write(180 - OPEN_ANGLE);
    Serial.print("Gripper Ready to grab Tim (2 sec)");
    delay(1000);
}

void MagnetPosition() {
    gripper_state = 0;
    RightServo.write(180);
    LeftServo.write(0);
    Serial.print("Gripper In Magnet Position (2 sec)");
    delay(1000);
}

void MagnetsOn() {
    magnet_state = 1;
    digitalWrite(LEFT_MAGNET_PIN, HIGH);
    digitalWrite(RIGHT_MAGNET_PIN, HIGH);
}

void MagnetsOff() {
    magnet_state = 0;
    digitalWrite(LEFT_MAGNET_PIN, LOW);
    digitalWrite(RIGHT_MAGNET_PIN, LOW);
}