# Design and Implementation of a Semi-Autonomous RC Car

[![F1Tenth Inspired](https://img.shields.io/badge/Platform-F1Tenth_Inspired-blue.svg)]()
[![MCU](https://img.shields.io/badge/MCU-ESP32-orange.svg)]()
[![RTOS](https://img.shields.io/badge/OS-FreeRTOS-green.svg)]()

This repository contains the mechanical designs, electronic schematics, and embedded firmware for a one-tenth scale semi-autonomous vehicle[cite: 2]. Inspired by the open-source F1Tenth platform, this project integrates advanced kinematic steering optimization with a highly deterministic, dual-core embedded control system[cite: 1, 2].

*(Please refer to the `graduation thesis.pdf` and `GP Presentation.pptx` files included in this repository for full mathematical analyses, optimization charts, and complete project documentation)*[cite: 1, 2].

## 📸 Project Showcase

<img width="1205" height="705" alt="Screenshot 2026-06-17 151951" src="https://github.com/user-attachments/assets/9f90f6d8-614e-40eb-831d-6e2f19d9c48a" />

<img width="402" height="531" alt="Picture1" src="https://github.com/user-attachments/assets/c02bdf50-a559-46da-b54a-2c87cf60e5d4" />

`![Mechanical CAD Design](images/cad_design.png)`

<img width="617" height="437" alt="Picture2" src="https://github.com/user-attachments/assets/57a4f66d-0572-4f09-93c6-d30c3f8da4c7" />

<img width="756" height="504" alt="Picture3" src="https://github.com/user-attachments/assets/eb47402b-5774-40d6-b9f0-7fac5ed36509" />

<img width="743" height="524" alt="Picture4" src="https://github.com/user-attachments/assets/a7689679-0a7b-427b-97c8-efabd9cb362e" />

`![System Architecture](images/system_architecture.png)`

## 🏎️ Project Overview

The primary objective of this project is to build a robust, high-performance robotic vehicle by eliminating mechanical bottlenecks, isolating electronic noise, and distributing embedded processing workloads[cite: 1, 2]. 

### Key Features & Achievements
*   **Custom 6-Bar Ackerman Steering:** Resolves the physical limitations and servo horn constraints of standard 4-bar trapezoidal mechanisms[cite: 1, 2]. 
*   **Kinematic Optimization:** Utilized MATLAB Grid Search and Stochastic Random Search algorithms to find the optimal steering arm length and initial bend angle, successfully minimizing the Root Mean Square Error (RMSE) between the ideal and actual inner/outer wheel angles[cite: 1, 2].
*   **Dual-Core Architecture:** Workloads are strictly segregated based on timing constraints to prevent processing bottlenecks on the ESP32[cite: 1, 2]. 
*   **Advanced Telemetry Filtering:** Implements a 5-stage median filter and a dynamic 10-stage moving average filter to stabilize raw RPM telemetry from the magnetic encoders[cite: 1, 2].

## 🛠️ System Architecture

### 1. Mechanical Layer
*   **Chassis:** Constructed from High-Density Polyethylene (HDPE) plastics using subtractive CNC routing, combined with 3D-printed suspension parts[cite: 1, 2]. 
*   **Drivetrain:** Features a staggered track width for improved stability and a rear-wheel-drive (RWD) system powered by dual 12V DC motors[cite: 1, 2].

### 2. Electrical Layer
*   **Custom PCB:** The electronics are mounted on a custom, single-sided printed circuit board designed for rapid lab prototyping on a CNC3018 isolation milling machine[cite: 1, 2]. 
*   **Power Segregation:** To eliminate electromagnetic interference (EMI) and back-EMF spikes from the actuators, the power delivery is isolated into three distinct domains: a 12V actuator domain, a 5V logic domain, and a 3.3V sensor domain[cite: 1, 2]. 

### 3. Software & Control Layer (FreeRTOS)
The system leverages FreeRTOS to split tasks between the two cores of the ESP32[cite: 1, 2]:

*   **Core 1 (Decision & Perception):** Handles asynchronous tasks such as RC signal filtering, calculating the Ackermann differential wheel speeds, and managing the Advanced Driver Assistance System (ADAS) state machine using ultrasonic sensors for obstacle avoidance[cite: 2].
*   **Core 0 (Deterministic Control):** Locked to a strict 1-millisecond hardware loop to aggregate I2C sensor data via a TCA9548A multiplexer and execute the Proportional-Integral (PI) closed-loop control to map targeted RPMs to stable PWM outputs[cite: 1, 2].

## 🧰 Hardware Components

| Component | Specification / Role |
| :--- | :--- |
| **Microcontroller** | ESP32 (Dual-Core, FreeRTOS)[cite: 2] |
| **Drive Motors** | 2x RS-775 DC Motors (12V, 3000 RPM)[cite: 2] |
| **Motor Drivers** | 2x BTS7960 IBT_2 Half-Bridge Drivers[cite: 2] |
| **Steering Actuator** | Tower Pro MG996R Servo Motor[cite: 2] |
| **Wheel Encoders** | 2x AS5600 Magnetic Rotary Encoders (I2C)[cite: 2] |
| **I2C Multiplexer** | TCA9548A (To resolve AS5600 address conflicts)[cite: 2] |
| **Obstacle Sensors** | 3x HC-SR04 Ultrasonic Sensors[cite: 2] |
| **Receiver** | FS-iA6B 6-Channel RC Receiver[cite: 1, 2] |
