#include <Arduino.h>
#include "mecanum_robot_pinout.h"
#include "MotorDriver.h"
#include "PID.h"
#include "EncoderVelocity.h"
#include "mecanum_robot_drive.h"

MotorDriver motors[NUM_MOTORS] = { {A_DIR1, A_PWM1, 0}, {A_DIR2, A_PWM2, 1},
                                   {B_DIR1, B_PWM1, 2}, {B_DIR2, B_PWM2, 3} };

EncoderVelocity encoders[NUM_MOTORS] = { {ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER3_A_PIN, ENCODER3_B_PIN, CPR_312_RPM, 0.2}, 
                                         {ENCODER4_A_PIN, ENCODER4_B_PIN, CPR_312_RPM, 0.2} };

PID pids[NUM_MOTORS] = { {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false}, 
                         {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false} };

double setpoints[NUM_MOTORS] = {0, 0, 0, 0};
double velocities[NUM_MOTORS] = {0, 0, 0, 0};
double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};

void setupDrive(){
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}

void updateSetpoints(double forward, double sideways) {
    
    double angle = PI/2 * (1 - 2*signbit(sideways));
    if (forward != 0) {
        angle = atan2(sideways, forward);
    }     
    double magnitude = sqrt(pow(forward, 2) + pow(sideways, 2));
    // Serial.printf("                   m: %.2f, a: %.2f\n", magnitude, angle);
    setpoints[0] = magnitude * sin(-angle + PI/4);
    setpoints[1] = magnitude * sin(angle + PI/4);
    setpoints[2] = magnitude * sin(angle + PI/4);
    setpoints[3] = -magnitude * sin(angle + PI/4);
}

void updatePIDs() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        velocities[i] = pow(-1, i) * encoders[i].getVelocity();
        controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);
        motors[i].drive(controlEfforts[i]);
        Serial.printf("Velocity %u: %.2f\n", i, velocities[i]);
        
    }
}