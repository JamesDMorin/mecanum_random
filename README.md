# Randomized Mecanum Wheels Sample Code

James Morin  
May 10, 2024[^1]

## *Readme in progress

## 1 Mobile Robot

### 1.1 Understand `robot/`

Take some time to understand `robot_main.cpp`, `robot_drive.cpp`, `robot_motion_control.cpp`, and `robot_wireless.cpp`. At a high level:
- `robot_main.cpp`: Includes the `setup()` and `loop()` functions, telling the microcontroller exactly what to do and when.
- `robot_drive.cpp`: Sets up the motors and implements a PI controller to follow velocity setpoints.
- `robot_motion_control.cpp`: Calculates odometry and setpoints based on either joystick or a given trajectory.
- `robot_wireless.cpp`: Sets up two-way wireless communication with and sends messages to the microcontroller on your controller.

## 2 Joystick Control




[^1]: This code was built from a framework provided by 2.12/2.120 Introduction to Robotics at MIT  
Version 1 - 2016: Peter Yu, Ryan Fish and Kamal Youcef-Toumi  
  Version 2 - 2017: Yingnan Cui and Kamal Youcef-Toumi  
  Version 3 - 2019: Jerry Ng  
  Version 4 - 2023: Joseph Ntaimo, Kentaro Barhydt, Ravi Tejwani, Kamal Youcef-Toumi and Harrison Chin  
  Version 5 - 2024: Jinger Chong, Josh Sohn
