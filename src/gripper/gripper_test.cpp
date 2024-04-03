#include <ESP32Servo.h>
#include <Arduino.h>
#include "util.h"
#include "wireless.h"
#include "gripper_pinout.h"
#include "UMS3.h"

UMS3 ums3;
// Create a servo object
Servo LeftServo;
Servo RightServo;

int pos = 0;      // position in degrees
ESP32PWM pwm;

void setup() {
  ums3.begin();

  // Brightness is 0-255. We set it to 1/3 brightness here
  ums3.setPixelBrightness(255 / 3);
  ums3.setPixelColor(UMS3::colorWheel(30));
  // Initialize serial communication
    Serial.begin(115200);
  
  // Attach the servo to the pin
  LeftServo.attach(LEFT_SERVO_PIN, MIN_US, MAX_US);
  RightServo.attach(RIGHT_SERVO_PIN, MIN_US, MAX_US);
  pwm.attachPin(14, 10000); //10kHz
  ums3.setPixelColor(UMS3::colorWheel(100));
}



void loop() {
  
  // while (Serial.available() == 0) {

  // }

  // int left_angle = input("left angle: ");
  // int right_angle = Serial.parseInt();

  // LeftServo.write(180 - open_angle);
  ums3.setPixelColor(UMS3::colorWheel(200));
  OpenForTim();
  // RightServo.write(0 + open_angle);
  // LeftServo.write(180 - open_angle);
  // Serial.print("Gripper Ready to grab Tim (1 sec)");
  // delay(1000);
    
  delay(1000);
  ums3.setPixelColor(UMS3::colorWheel(150));
  ClosedOnTim();
  // RightServo.write(0);
  // LeftServo.write(180);
  // Serial.print("Gripper closing onto Tim (1 sec)");
  // delay(1000);
  delay(1000);
  

}


void ClosedOnTim() {
    RightServo.write(0);
    LeftServo.write(180);
    Serial.print("Gripper closing onto Tim (1 sec)");
    delay(1000);
}

void OpenForTim() {
    RightServo.write(0 + OPEN_ANGLE);
    LeftServo.write(180 - OPEN_ANGLE);
    Serial.print("Gripper Ready to grab Tim (1 sec)");
    delay(1000);
}