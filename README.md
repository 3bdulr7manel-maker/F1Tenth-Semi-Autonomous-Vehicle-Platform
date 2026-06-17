# Design and Implementation of a Semi-Autonomous RC Car

[![F1Tenth Inspired](https://img.shields.io/badge/Platform-F1Tenth_Inspired-blue.svg)]()
[![MCU](https://img.shields.io/badge/MCU-ESP32-orange.svg)]()
[![RTOS](https://img.shields.io/badge/OS-FreeRTOS-green.svg)]()

This repository contains the mechanical designs, electronic schematics, and embedded firmware for a one-tenth scale semi-autonomous vehicle. Inspired by the open-source F1Tenth platform, this project integrates advanced kinematic steering optimization with a highly deterministic, dual-core embedded control system.

*(Please refer to the `graduation thesis.pdf` and `GP Presentation.pptx` files included in this repository for full mathematical analyses, optimization charts, and complete project documentation).*

## 📸 Mechanical CAD Showcase

<div align="center">
<img width="500" height="500" alt="Screenshot 2026-06-17 151951" src="https://github.com/user-attachments/assets/24886edc-6975-4e0f-8062-d973a1eb91ea" />

<img width="500" height="500" alt="Picture1" src="https://github.com/user-attachments/assets/96c38c33-9010-4ae7-8a2d-3ad6aa2bc439" />

</div>

## 🏎️ Project Overview

The primary objective of this project is to build a robust, high-performance robotic vehicle by eliminating mechanical bottlenecks, isolating electronic noise, and distributing embedded processing workloads. 

### Key Features & Achievements
* **Custom 6-Bar Ackerman Steering:** Resolves the physical limitations and servo horn constraints of standard 4-bar trapezoidal mechanisms. The steering linkage is designed to actuate horizontally to accurately execute the optimal kinematics.
* **Kinematic Optimization:** Utilized MATLAB Grid Search and Stochastic Random Search algorithms to find the optimal steering arm length and initial bend angle, successfully minimizing the Root Mean Square Error (RMSE) between the ideal and actual inner/outer wheel angles.
* **Dual-Core Architecture:** Workloads are strictly segregated based on timing constraints to prevent processing bottlenecks on the ESP32. 
* **Advanced Telemetry Filtering:** Implements a 5-stage median filter and a dynamic 10-stage moving average filter to stabilize raw RPM telemetry from the magnetic encoders.

## 🛠️ System Architecture

### 1. Mechanical Layer
* **Chassis:** Constructed from High-Density Polyethylene (HDPE) plastics using subtractive CNC routing, combined with 3D-printed suspension parts. 
* **Drivetrain:** Features a staggered track width for improved stability and a rear-wheel-drive (RWD) system powered by dual 12V DC motors.

### 2. Electrical Layer
* **Custom PCB & Schematics:** The electronics are mounted on a custom, single-sided printed circuit board designed for rapid lab prototyping on a CNC3018 isolation milling machine. 

<div align="center">
  <img width="500" alt="Schematic Diagram" src="https://github.com/user-attachments/assets/57a4f66d-0572-4f09-93c6-d30c3f8da4c7" />
  <img width="500" alt="Code Logic and Software Architecture" src="https://github.com/user-attachments/assets/a7689679-0a7b-427b-97c8-efabd9cb362e" />
</div>

</div>

<br>

* **Power Segregation:** To eliminate electromagnetic interference (EMI) and back-EMF spikes from the actuators, the power delivery is isolated into three distinct domains: a 12V actuator domain, a 5V logic domain, and a 3.3V sensor domain. 

### 3. Software & Control Layer (FreeRTOS)

The system leverages FreeRTOS to split tasks between the two cores of the ESP32, ensuring deterministic execution of critical control loops.

<div align="center">

  <img width="600" alt="PCB Layout" src="https://github.com/user-attachments/assets/eb47402b-5774-40d6-b9f0-7fac5ed36509" />

<br>

* **Core 1 (Decision & Perception):** Handles asynchronous tasks such as RC signal filtering, calculating the Ackermann differential wheel speeds, and managing the Advanced Driver Assistance System (ADAS) state machine using ultrasonic sensors for obstacle avoidance.
* **Core 0 (Deterministic Control):** Locked to a strict 1-millisecond hardware loop to aggregate I2C sensor data via a TCA9548A multiplexer. The right-side AS5600 encoder is registered in reverse compared to the left-side baseline to ensure valid differential RPM tracking. This core executes the Proportional-Integral (PI) closed-loop control to map targeted RPMs to stable PWM outputs.

## 🧰 Hardware Components

| Component | Specification / Role |
| :--- | :--- |
| **Microcontroller** | ESP32 (Dual-Core, FreeRTOS) |
| **Drive Motors** | 2x RS-775 DC Motors (12V, 3000 RPM) |
| **Motor Drivers** | 2x BTS7960 IBT_2 Half-Bridge Drivers |
| **Steering Actuator** | Tower Pro MG996R Servo Motor |
| **Wheel Encoders** | 2x AS5600 Magnetic Rotary Encoders (I2C) |
| **I2C Multiplexer** | TCA9548A (To resolve AS5600 address conflicts) |
| **Obstacle Sensors** | 3x HC-SR04 Ultrasonic Sensors |
| **Receiver** | FS-iA6B 6-Channel RC Receiver |
