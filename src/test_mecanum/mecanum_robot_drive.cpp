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

// x, y, phi
double motor_poses[NUM_MOTORS][3] = { {-0.144, -0.116, -PI/4}, {-0.144,  0.116, PI/4},
                                      { 0.140,  0.048, PI*3/4}, { 0.140, -0.048, -PI*3/4} };

// r, theta
double motor_position_vectors[NUM_MOTORS][2] = {{sqrt(pow(motor_poses[0][0], 2) + pow(motor_poses[0][1], 2)), atan2(motor_poses[0][1], motor_poses[0][0])}, 
                                                {sqrt(pow(motor_poses[1][0], 2) + pow(motor_poses[1][1], 2)), atan2(motor_poses[1][1], motor_poses[1][0])}, 
                                                {sqrt(pow(motor_poses[2][0], 2) + pow(motor_poses[2][1], 2)), atan2(motor_poses[2][1], motor_poses[2][0])}, 
                                                {sqrt(pow(motor_poses[3][0], 2) + pow(motor_poses[3][1], 2)), atan2(motor_poses[3][1], motor_poses[3][0])}, };

double setpoints[NUM_MOTORS] = {0, 0, 0, 0};
double old_setpoints[NUM_MOTORS] = {0, 0, 0, 0};
double velocities[NUM_MOTORS] = {0, 0, 0, 0};
double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};

void setupDrive(){
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}

void updateSetpoints(double forward, double sideways, double rotation) {
    
    double motor_magnitude_angle[NUM_MOTORS][2];
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        double rotational_velocity[2] = {-motor_poses[i][1]*rotation, motor_poses[i][0]*rotation}; // 90 degree ccw rotation matrix
        double motor_forward = forward + rotational_velocity[0];
        double motor_sideways = sideways + rotational_velocity[1];
        
        motor_magnitude_angle[i][1] = atan2(motor_sideways, motor_forward);
        motor_magnitude_angle[i][0] = sqrt(pow(motor_forward, 2) + pow(motor_sideways, 2));
        
        // Serial.printf("Motor %u: mag: %.2f, angle: %.2f\n", i, motor_magnitude_angle[i][0], motor_magnitude_angle[i][1]);

        setpoints[i] = M_ALPHA * motor_magnitude_angle[i][0] * cos(motor_magnitude_angle[i][1] - motor_poses[i][2] + PI) + (1-M_ALPHA) * old_setpoints[i];

        old_setpoints[i] = setpoints[i];
        // Serial.print('hi James: ');
        Serial.printf("Motor %u: Setpoint: %.2f, ", i, setpoints[i]);
    }

    // setpoints[0] = (motor_magnitude_angle[0][0] * sin(motor_magnitude_angle[0][1] - PI/4))*M_ALPHA + old_setpoints[0]*(1-M_ALPHA);
    // setpoints[1] = -(motor_magnitude_angle[1][0] * sin(motor_magnitude_angle[1][1] + PI/4))*M_ALPHA + old_setpoints[1]*(1-M_ALPHA);
    // setpoints[2] = (motor_magnitude_angle[2][0] * sin(-motor_magnitude_angle[2][1] + PI/4))*M_ALPHA + old_setpoints[2]*(1-M_ALPHA);
    // setpoints[3] = (motor_magnitude_angle[3][0] * sin(motor_magnitude_angle[3][1] + PI/4))*M_ALPHA + old_setpoints[3]*(1-M_ALPHA);
    // for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        
    //     old_setpoints[i] = setpoints[i];
    //     // Serial.print('hi James: ');
    //     Serial.printf("Motor %u: Setpoint: %.2f, ", i, setpoints[i]);
    // }
    Serial.println();
}

void updatePIDs() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        velocities[i] = pow(-1, i) * encoders[i].getVelocity(); // pow(-1, i) * 
        controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);
        motors[i].drive(controlEfforts[i]);        
    }
}