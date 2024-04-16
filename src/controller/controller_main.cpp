#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "joystick.h"
#include "dpad.h"
#include "display.h"
#include "controller_pinout.h"

ControllerMessage prevControllerMessage;

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN);
Joystick joystick2(JOYSTICK2_X_PIN, JOYSTICK2_Y_PIN);

void setup() {
    Serial.begin(115200);

    setupWireless();

    joystick1.setup();
    joystick2.setup();
    pinMode(BUTTON_L_PIN, INPUT_PULLUP);
    pinMode(BUTTON_R_PIN, INPUT_PULLUP);    

    Serial.println("Setup complete.");
}

void loop() {
    // Read and send controller sensors
    EVERY_N_MILLIS(50) {
        controllerMessage.millis = millis();
        controllerMessage.joystick1 = joystick1.read();
        controllerMessage.joystick2 = joystick2.read();
        controllerMessage.buttonL = digitalRead(BUTTON_L_PIN) == LOW;
        controllerMessage.buttonR = digitalRead(BUTTON_R_PIN) == LOW;
        
        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
        // Serial.printf("Joystick1: %.2f, %.2f, Joystick2: %.2f, %.2f, BL: %u, BR:%u\n",
        //                 controllerMessage.joystick1.x, controllerMessage.joystick1.y,
        //                 controllerMessage.joystick2.x, controllerMessage.joystick2.y,
        //                 controllerMessage.buttonL, controllerMessage.buttonR);
    }
}