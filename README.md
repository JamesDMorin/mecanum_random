# Team 1: Mobile Robot (and gripper) Code

2.12/2.120 Intro to Robotics  
Spring 2024[^1]

## 1 Mobile Robot

### 1.1 Understand `robot/`

Take some time to understand `robot_main.cpp`, `robot_drive.cpp`, `robot_motion_control.cpp`, and `robot_wireless.cpp`. At a high level:
- `robot_main.cpp`: Includes the `setup()` and `loop()` functions, telling the microcontroller exactly what to do and when.
- `robot_drive.cpp`: Sets up the motors and implements a PI controller to follow velocity setpoints.
- `robot_motion_control.cpp`: Calculates odometry and setpoints based on either joystick or a given trajectory.
- `robot_wireless.cpp`: Sets up two-way wireless communication with and sends messages to the microcontroller on your controller.

### 1.2 Understand Odometry

Open `robot_motion_control.cpp` and read through `updateOdometry()`. Make sure that you understand exactly how this function calculates odometry data. For reference:

<p align="center">
<img src="./.images/odom.png" alt="drawing" width="1000"/>
</p>

We will want to do an odometry that depends on IMU, not encoder values. Task for Yuan?

## 2 Joystick Control


### 2.4 Run Controller

Upload `controller_main.cpp` and `controller_wireless.cpp` to the microcontroller on your controller. This will read the joystick and set up two-way wireless communication with the microcontroller on the mobile robot.

### 2.5 Run Joystick Control

In `robot_motion_control.cpp`, comment out `#define CIRCLE` and uncomment `#define JOYSTICK`. This will change the `followTrajectory()` function to follow a joystick instead of a circle. 

Set your PlatformIO environment back to `env:robot`. Upload `robot_main.cpp`, `robot_drive.cpp`, `robot_motion_control.cpp`, and `robot_wireless.cpp` to the microcontroller on your mobile robot. At this point, you should be able to drive your mobile robot around with your joystick!

We will maybe use a logitech controller, not just joysticks and buttons on a breadboard.

## 3 Custom Trajectory

In `robot_motion_control.cpp`, comment out `#define JOYSTICK` and uncomment `#define YOUR_TRAJECTORY`. In the `followTrajectory()` function, make your own path using a state machine, taking `UTURN` as inspiration.

We could kinda do a path following from distance data, more to be seen.

### 5.1 IMU 

Integrate the IMU with your mobile robot! This will probably be very useful for your final project.

### 5.2 Mecanum Wheels

Replace the existing wheels with mecanum wheels! Mecanum wheels allow the robot to move in any direction. However, the odometry and controller will be slightly different.


[^1]: Version 1 - 2016: Peter Yu, Ryan Fish and Kamal Youcef-Toumi  
  Version 2 - 2017: Yingnan Cui and Kamal Youcef-Toumi  
  Version 3 - 2019: Jerry Ng  
  Version 4 - 2023: Joseph Ntaimo, Kentaro Barhydt, Ravi Tejwani, Kamal Youcef-Toumi and Harrison Chin  
  Version 5 - 2024: Jinger Chong, Josh Sohn
