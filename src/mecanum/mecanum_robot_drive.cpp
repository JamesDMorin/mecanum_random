// Credit to James D. Morin for Mecanum Wheel kinematics

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

double velocities[NUM_MOTORS] = {0, 0, 0, 0};
double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};
double setpoints[NUM_MOTORS] = {0, 0, 0, 0};       // angular velocity to turn the motor
double v_desired[NUM_MOTORS] = {0, 0, 0, 0};       // spped of the wheel in the direction of φ_i

// x (m), y (m), φ_i (rad). Conversion included from inches to meters for x and y
double motor_poses[NUM_MOTORS][3] = { {12.2483*25.4/1000, 12.7232*25.4/1000,  2.9214}, 
                                      { 2.9900*25.4/1000,  7.8362*25.4/1000, -1.5767},
                                      { 7.1298*25.4/1000, 10.3410*25.4/1000,  0.5002}, 
                                      {10.4816*25.4/1000,  2.6018*25.4/1000, -2.1484} };

/**
 * Updates the setpoints for the angular velocity of each wheel's motor given a set of input target velocities.
 *
 * @param forward, sideways Vehicle velocity along vehicle coordinate frame axes in m/s.
 * @param rotation Vehicle angular velocity in rad/s
 */
void updateSetpoints(double forward, double sideways, double rotation) {
    
    double motor_magnitude_angle[NUM_MOTORS][2];
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        
        // Equation 3.1
        double motor_forward = forward - motor_poses[i][1] * rotation;
        double motor_sideways = sideways + motor_poses[i][0] * rotation;
        
        // Equation 3.2
        motor_magnitude_angle[i][0] = sqrt(pow(motor_forward, 2) + pow(motor_sideways, 2));
        // Equation 3.3
        motor_magnitude_angle[i][1] = atan2(motor_sideways, motor_forward);
        
        // Equation 3.5
        v_desired[i] = motor_magnitude_angle[i][0] * cos(motor_magnitude_angle[i][1] - motor_poses[i][2]);
        
        // Equation 3.7 with an alpha low pass filter
        setpoints[i] = M_ALPHA * v_desired[i]/R_EFF + (1-M_ALPHA) * setpoints[i];

        Serial.printf("Motor %u: Setpoint: %.2f, ", i, setpoints[i]);
    }
    Serial.println();
}

void setupDrive(){
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}

void updatePIDs() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        velocities[i] = pow(-1, i) * encoders[i].getVelocity(); // pow(-1, i) * 
        controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);
        motors[i].drive(controlEfforts[i]);
    }
}