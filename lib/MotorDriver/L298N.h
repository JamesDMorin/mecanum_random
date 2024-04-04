#ifndef L298N_H
#define L298N_H

#include <Arduino.h>

/**
 * @class MotorDriver
 * @brief Class to control a motor using PWM.
 */
class L298N {
public:
    /**
     * @brief Constructor for MotorDriver class.
     * 
     * @param in1Pin Digital pin number connected to the motor's direction control.
     * @param in2Pin Digital pin number connected to the motor's direction control.
     * @param enablePin Digital pin number connected to the motor's PWM control.
     * @param ledcChannel LEDC channel used for PWM.
     * @param pwmFreq Frequency of PWM signal (default 20kHz).
     * @param pwmBits Resolution of PWM signal (default 10 bits).
     */
    L298N(int in1Pin, int in2Pin, int enablePin, int ledcChannel, int pwmFreq = 20000, int pwmBits = 10);
    
    /**
     * @brief Sets up the motor driver by initializing pins and PWM.
     */
    void setup();

    /**
     * @brief Drives the motor by setting the PWM duty cycle.
     * 
     * @param dutyCycle A value between -1.0 (full reverse) and 1.0 (full forward).
     */
    void drive(double dutyCycle);

    /**
     * @brief Gets the current PWM duty cycle value.
     * 
     * @return double The duty cycle last set by the drive method.
     */
    double getCurrentDutyCycle();

private:
    int _in1Pin;          ///< Pin for motor direction control
    int _in2Pin;          ///< Pin for motor direction control
    int _enablePin;       ///< Pin for motor PWM control
    int _ledcChannel;     ///< LEDC channel for PWM
    int _pwmFreq;         ///< Frequency of PWM signal
    int _pwmBits;         ///< Resolution of PWM signal
    double _currentDutyCycle; ///< Current duty cycle for the motor
};

#endif // MOTORDRIVER_H
