# FreeRTOS_project
 
## Topic
The aim of this assignment was to self-study and implement a Real Time Operating System (RTOS)
on STM32 using FreeRTOS.

## Motivation
Exploring possible use cases of RTOSes in the area of robotics (especially mobile robots, UAVs and UUVs).

## Hardware components
- STM32F411 micro-controller on the Nucleo-64 board
- HCSR04 Ultrasonic sensor
- Button
- Blue, Green, Red, Yellow leds
- Wires
- Breadboard

## Software tools 
- STM32CubeMX
- STM32CubeIDE
- RealTerm Serial/TCP terminal

## Small description
The main idea of the project was to build the "brain" of a robot that includes a number of sensors (IMU, GPS, Radar) and functionalities (object detection, emergency stop button, I/O user commands, Kalma Filter for fusing GPS/IMU). That said, most of the IMU/GPS data are fake and the Kalman Filter threads doesn't actually do something, as this was not the object of the project. Radar is also replaced with an actual Ultrasonic sensor that detects whether or not something is dangerously close and I/O commands were sent through the UART connection. Finally, as an emergency button, two seperate buttons were used. An external button and the blue one that is integrated on the Nucleo board. In terms of inner-thread communication, Queues/Signals/Semaphores,Mutexes and some global variables were used.

## Note
The main purpose of this project was to study RTOSes and experiment with them on the STM32 microcontroller, so the end product is under no circumstances a realistic RTOS that could or should be used in actual robots.
Feel free to use any parts of this project in any way you want.