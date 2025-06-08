# TENET (SLRC ROBO Competition)

## Introduction

TENET is a robotics system developed for participation in the SLRC ROBO Competition. The project features both autonomous vehicle navigation and a robotic arm, integrating a range of sensors (ultrasonic, color, IR) and actuators (DC motors, servos) for precise movement and object manipulation. The software is written in C++ and optimized for real-time control using techniques such as PID regulation, sensor fusion, and interrupt-driven encoder feedback. The codebase is modular, supporting clear separation of robot movement logic, sensor handling, and arm control. This project demonstrates practical robotics engineering, blending hardware interfacing with advanced algorithms to achieve robust and reliable performance in competitive robotics scenarios.

## Algorithms

- PID Algorithm: Used for motor speed correction, balancing the speed of left and right wheels for straight movement.
- Median Filtering: For sensor data smoothing, especially ultrasonic sensor readings.
- Obstacle Avoidance: Distance thresholds trigger stop and communication routines.
- Servo Angle Mapping: For the robotic arm, servo angles are mapped to PWM duty cycles.

## Main Hardware Components
1. Microcontroller (ESP32 & Arduino Nano)
2. Motors and Motor Drivers
3. Rotary Encoders
4. Ultrasonic Sensor
5. IR Sensor for line & obstacle detection
6. Color Sensors for object color detection
7. Servo Motors for controlling the robotic arm
8. Hardware Serial/UART for module-to-module or controller communication
9. Power Supply
10. Robo arm



## ROBOT   

![WhatsApp Image 2025-05-24 at 20 02 08_5b7e0782](https://github.com/user-attachments/assets/906416ce-1dcd-48fe-aab5-db4c321c01b8)
