#include <ESP32Servo.h>
#include <Arduino.h>
#include <Adafruit_AMG88xx.h>
#include "util.h"
#include "wireless.h"
#include "gripper_pinout.h"
#include "UMS3.h"
#include "gripper_actions.h"

UMS3 ums3;
Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

Servo LeftServo;
Servo RightServo;

int pos = 0;      // position in degrees
ESP32PWM pwm;

extern GripperMessage gripperMessage;
extern GripperControllerMessage gripperControllerMessage;

int magnet_state = 0;
int gripper_state = 1;
int old_magnet_state = 0;
int old_gripper_state = 1;

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

    // Serial.println(F("AMG88xx pixels"));
    bool amg_status;
    amg_status = amg.begin();
    if (!amg_status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        // while (1);
    }
    delay(100); // let amg boot up
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
        ums3.setPixelColor(UMS3::color(r,g,b));
        MagnetsOn();
    } else {
        //turn off magnet
        r = 0;
        ums3.setPixelColor(UMS3::color(r,g,b));
        MagnetsOff();
    }

    if (gripper_state == 0) {
        // Gripper EM Mode
        b = 255;
        g = 0;
        ums3.setPixelColor(UMS3::color(r,g,b));
        MagnetPosition();

    } else if (gripper_state == 1) {
        // Gripper Open 
        b = 0;
        g = 255;
        ums3.setPixelColor(UMS3::color(r,g,b));
        MagnetsOff();
        OpenForTim();
        
    } else if (gripper_state == 2) {
        // Gripper Close
        b = 255;
        g = 255;
        ums3.setPixelColor(UMS3::color(r,g,b));
        MagnetsOff();
        ClosedOnTim();
        
    }

    ums3.setPixelColor(UMS3::color(r,g,b));

    amg.readPixels(pixels);
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      gripperMessage.pixels[i-1] = pixels[i-1];
    }
    gripperMessage.gripper_state = gripper_state;
    gripperMessage.magnet_state = magnet_state;

    // Serial.print("[");
    // for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    //   Serial.print(gripperMessage.pixels[i-1]);
    //   Serial.print(", ");
    //   if( i%8 == 0 ) Serial.println();
    // }
    // Serial.println("]");
    
    old_magnet_state = magnet_state;
    old_gripper_state = gripper_state;

}

void ClosedOnTim() {
    // MagnetsOff();
    gripper_state = 2;
    RightServo.write(0);
    LeftServo.write(180);
    // Serial.print("Gripper closing onto Tim (2 sec)");
    if (old_gripper_state != gripper_state) {
        delay(1000);
    }
}

void OpenForTim() {
    // MagnetsOff();
    gripper_state = 1;
    RightServo.write(0 + OPEN_ANGLE);
    LeftServo.write(180 - OPEN_ANGLE);
    // Serial.print("Gripper Ready to grab Tim (2 sec)");
    if (old_gripper_state != gripper_state) {
        delay(1000);
    }
}

void MagnetPosition() {
    gripper_state = 0;
    RightServo.write(180);
    LeftServo.write(0);
    // Serial.print("Gripper In Magnet Position (2 sec)");
    if (old_gripper_state != gripper_state) {
        delay(1000);
    }
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