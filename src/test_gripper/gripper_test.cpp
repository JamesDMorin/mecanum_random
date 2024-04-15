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
  pinMode(OPEN_POS_BUTTON, INPUT_PULLUP);
  pinMode(CLOSED_POS_BUTTON, INPUT_PULLUP);
  pinMode(MAGNET_POS_BUTTON, INPUT_PULLUP);
}



void loop() {
  
  if (digitalRead(OPEN_POS_BUTTON) == LOW) {
    OpenForTim();
    ums3.setPixelColor(UMS3::colorWheel(200));
  } else if (digitalRead(CLOSED_POS_BUTTON) == LOW) {
    ClosedOnTim();
    ums3.setPixelColor(UMS3::colorWheel(150));
  } else if (digitalRead(MAGNET_POS_BUTTON) == LOW) {
    MagnetPosition();
    ums3.setPixelColor(UMS3::colorWheel(100));
  }
  // LeftServo.write(180 - open_angle);
  // ums3.setPixelColor(UMS3::colorWheel(200));
  // MagnetPosition();
  // OpenForTim();
  // digitalWrite(IN1, HIGH);
  // digitalWrite(IN2, LOW);
  // delay(5000);
  // Motor A
  // digitalWrite(IN1, HIGH);
  // digitalWrite(IN2, HIGH);

  // delay(1000);

  // ums3.setPixelColor(UMS3::colorWheel(150));
  // ClosedOnTim();
  // Rotates the Motor A counter-clockwise
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // delay(2000);
  // Motor A
  // digitalWrite(IN1, HIGH);
  // digitalWrite(IN2, HIGH);
  // delay(500);

  // delay(4000);
  

}


void ClosedOnTim() {
    RightServo.write(0);
    LeftServo.write(180);
    Serial.print("Gripper closing onto Tim (2 sec)");
    delay(2000);
}

void OpenForTim() {
    RightServo.write(0 + OPEN_ANGLE);
    LeftServo.write(180 - OPEN_ANGLE);
    Serial.print("Gripper Ready to grab Tim (2 sec)");
    delay(2000);
}

void MagnetPosition() {
    RightServo.write(180);
    LeftServo.write(0);
    Serial.print("Gripper In Magnet Position (2 sec)");
    delay(2000);
}