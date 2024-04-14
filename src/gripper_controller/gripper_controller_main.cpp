#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "dpad.h"
#include "display.h"
#include "gripper_controller_pinout.h"

GripperControllerMessage prevGripperControllerMessage;

Bounce bounce0 = Bounce(); // INSTANTIATE A Bounce OBJECT
Bounce bounce1 = Bounce(); // INSTANTIATE A Bounce OBJECT
Bounce bounce2 = Bounce(); // INSTANTIATE A Bounce OBJECT
Bounce bounce3 = Bounce(); // INSTANTIATE A Bounce OBJECT
Bounce bounce4 = Bounce(); // INSTANTIATE A Bounce OBJECT

void setup() {
    Serial.begin(115200);

    setupWireless();

    // BUTTONS SETUP
    bounce0.attach( MAG_ON_PIN ,  INPUT_PULLUP ); // BOUNCE SETUP (USE INTERNAL PULL-UP)
    bounce1.attach( MAG_OFF_PIN ,  INPUT_PULLUP );
    bounce2.attach( MAG_MODE_PIN ,  INPUT_PULLUP );
    bounce3.attach( GRIP_OFF_PIN ,  INPUT_PULLUP );
    bounce4.attach( GRIP_ON_PIN ,  INPUT_PULLUP );
    bounce0.interval(5); // DEBOUNCE INTERVAL IN MILLISECONDS
    bounce1.interval(5);
    bounce2.interval(5);
    bounce3.interval(5);
    bounce4.interval(5);

    Serial.println("Setup complete.");
}

void loop() {
    gripperControllerMessage.millis = millis();

    //Read and send controller sensors
    bounce0.update(); // Update the Bounce instance (YOU MUST DO THIS EVERY LOOP)
    bounce1.update();
    bounce2.update();
    bounce3.update();
    bounce4.update();

    // <Bounce>.changed() RETURNS true IF THE STATE CHANGED (FROM HIGH TO LOW OR LOW TO HIGH)
    if ( bounce0.changed() ) {
        int deboucedInput = bounce0.read();
        if ( deboucedInput == LOW ) {
            gripperControllerMessage.command = 0;
            sendControllerData();
        }
    }
    if ( bounce1.changed() ) {
        int deboucedInput = bounce1.read();
        if ( deboucedInput == LOW ) {
            gripperControllerMessage.command = 1;
            sendControllerData();
        }
    }
    if ( bounce2.changed() ) {
        int deboucedInput = bounce2.read();
        if ( deboucedInput == LOW ) {
            gripperControllerMessage.command = 2;
            sendControllerData();
        }
    }
    if ( bounce3.changed() ) {
        int deboucedInput = bounce3.read();
        if ( deboucedInput == LOW ) {
            gripperControllerMessage.command = 3;
            sendControllerData();
        }
    }    if ( bounce4.changed() ) {
        int deboucedInput = bounce4.read();
        if ( deboucedInput == LOW ) {
            gripperControllerMessage.command = 4;
            sendControllerData();
        }
    }
    

    // if (!(prevControllerMessage == controllerMessage)) {
    //     sendControllerData();
    //     prevControllerMessage = controllerMessage;
    // }

}