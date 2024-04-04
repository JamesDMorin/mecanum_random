#include "L298N.h"

L298N::L298N(int in1Pin, int in2Pin, int enablePin, int ledcChannel, int pwmFreq, int pwmBits)
: _in1Pin(in1Pin), _in2Pin(in2Pin), _enablePin(enablePin), _ledcChannel(ledcChannel), _pwmFreq(pwmFreq), _pwmBits(pwmBits), _currentDutyCycle(0) {
    // Constructor initializes member variables with provided values
}

void L298N::setup() {
    // Configure motor control pins as outputs
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    pinMode(_enablePin, OUTPUT);

    // Setup PWM on the specified channel, frequency, and resolution
    ledcSetup(_ledcChannel, _pwmFreq, _pwmBits);

    // Attach the PWM pin to the LEDC channel
    ledcAttachPin(_enablePin, _ledcChannel);
}

void L298N::drive(double dutyCycle) {
    // Constrain the duty cycle to valid range (-1.0 to 1.0)
    dutyCycle = constrain(dutyCycle, -0.999, 0.999);

    // Set the motor direction based on the sign of duty cycle
    if (dutyCycle > 0){
        digitalWrite(_in1Pin, dutyCycle > 0);
        digitalWrite(_in2Pin, dutyCycle > 0);
        ledcAttachPin(_enablePin, 1);
    } else if ( dutyCycle < 0 ){
        digitalWrite(_in1Pin, dutyCycle > 0);
        digitalWrite(_in2Pin, dutyCycle > 0); 
        ledcAttachPin(_enablePin, 0);
    } else {
        digitalWrite(_in1Pin, dutyCycle > 0);
        digitalWrite(_in2Pin, dutyCycle > 0);
        ledcAttachPin(_enablePin, 1);
    }

    // Calculate and set the PWM duty cycle
    _currentDutyCycle = abs(dutyCycle) * (pow(2, _pwmBits) - 1);
    ledcWrite(_ledcChannel, _currentDutyCycle);
}

double L298N::getCurrentDutyCycle() {
    // Return the current duty cycle value
    return _currentDutyCycle;
}
