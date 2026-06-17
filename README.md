# Design and Implementation of a Semi-Autonomous RC Car

[![F1Tenth Inspired](https://img.shields.io/badge/Platform-F1Tenth_Inspired-blue.svg)]()
[![MCU](https://img.shields.io/badge/MCU-ESP32-orange.svg)]()
[![RTOS](https://img.shields.io/badge/OS-FreeRTOS-green.svg)]()

This repository contains the mechanical designs, electronic schematics, and embedded firmware for a one-tenth scale semi-autonomous vehicle. Inspired by the open-source F1Tenth platform, this project integrates advanced kinematic steering optimization with a highly deterministic, dual-core embedded control system.

`graduation thesis.pdf`
`GP_Presentation.pptx` 
## 📸 Mechanical CAD Showcase

<div align="center">
  
<img width="600" height="500" alt="Screenshot 2026-06-17 194733" src="https://github.com/user-attachments/assets/77db05a5-e4d0-4078-839f-56b4442d1e8c" />



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
  <img width="450" height="400" alt="Screenshot 2026-06-17 194955" src="https://github.com/user-attachments/assets/8acab342-0ee0-481d-a3b9-5fd11b434af6" />

 <img width="450" height="400" alt="Screenshot 2026-06-17 195028" src="https://github.com/user-attachments/assets/77a4c6ad-79ba-456b-b74b-6337a85786e2" />
 
<img width="500" height="400" alt="Screenshot 2026-06-17 194612" src="https://github.com/user-attachments/assets/e9da528d-436f-4cf8-88d2-16a88f32b886" />

</div>

</div>

<br>

* **Power Segregation:** To eliminate electromagnetic interference (EMI) and back-EMF spikes from the actuators, the power delivery is isolated into three distinct domains: a 12V actuator domain, a 5V logic domain, and a 3.3V sensor domain. 

### 3. Software & Control Layer (FreeRTOS)

The system leverages FreeRTOS to split tasks between the two cores of the ESP32, ensuring deterministic execution of critical control loops.

<div align="center">
<img width="600" height="500" alt="5879451809368182509" src="https://github.com/user-attachments/assets/83bbe637-798c-4e6d-ab33-50afd31f493a" />


<br>
<div align="left">

* **Core 1 (Decision & Perception):** Handles asynchronous tasks such as RC signal filtering, calculating the Ackermann differential wheel speeds, and managing the Advanced Driver Assistance System (ADAS) state machine using ultrasonic sensors for obstacle avoidance.
* **Core 0 (Deterministic Control):** Locked to a strict 1-millisecond hardware loop to aggregate I2C sensor data via a TCA9548A multiplexer. The right-side AS5600 encoder is registered in reverse compared to the left-side baseline to ensure valid differential RPM tracking. This core executes the Proportional-Integral (PI) closed-loop control to map targeted RPMs to stable PWM outputs.

<div align="center">
  
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


## 🎥 Live Demonstration & Real Prototype


<div align="center">
<img width="300" height="300" alt="58794518093681823922" src="https://github.com/user-attachments/assets/c041b22d-02b4-444b-b971-e87a7c440431" />

<img width="300" height="300" alt="587945180936818239333" src="https://github.com/user-attachments/assets/6f8fcb85-46fd-419d-a4f2-35f8baa7d1d0" />






<p align="center">
  <video src="https://github.com/user-attachments/assets/83665a0f-c02f-4cce-9204-bd650317a64b"
         width="400"
      height="400"
         controls>
  </video>
</p>


</div>

<div align="left">
  
## 💡 Project Constraints & Learning Outcomes

While the current state of the vehicle focuses on establishing a robust, foundational semi-autonomous system, the journey to this baseline involved overcoming significant multi-disciplinary hurdles:

* **Cross-Disciplinary Learning Curve:** Coming from a purely Mechanical Engineering background, the electrical and embedded software domains were approached with zero prior knowledge. A substantial portion of the project's timeline was intentionally dedicated to mastering the fundamentals of PCB design, signal processing, and RTOS architecture from the ground up.
* **Supply Chain & Hardware Reliability:** The prototyping phase was heavily impacted by the acquisition of counterfeit electronic components in the local market. Diagnosing unpredictable hardware failures caused by these fake modules consumed considerable time, ultimately shifting the project's focus toward achieving extreme stability and deterministic control over adding complex software features. 

Ultimately, what may appear as a foundational prototype is the result of rigorous cross-disciplinary learning, troubleshooting, and the successful integration of three distinct engineering fields into a single, functional system.

## 🚀 Future Improvements & Next Steps

To advance this semi-autonomous vehicle from a foundational prototype to a highly robust, deployment-ready robotic platform, several mechanical, electrical, and software upgrades are planned for future iterations:

### 1. Mechanical & Structural Enhancements
* **Advanced Tire Materials:** Transition to high-grip compounds (such as specialized TPU or soft rubber) to significantly increase the friction coefficient and eliminate wheel slippage, ensuring precise execution of the optimized Ackermann steering geometry.
* **Secondary Suspension System:** Integrate a dedicated secondary suspension/shock-absorption mechanism to dampen high-frequency vibrations. This is critical for protecting delicate onboard electronics and maintaining sensor stability, particularly when operating on rigid concrete surfaces.

### 2. Electrical & Power Subsystem Upgrades
* **Pin Mapping & Schematic Auditing:** Conduct a rigorous cross-verification between the hardware PCB schematics and the FreeRTOS codebase to guarantee perfect hardware-software pin alignment prior to any secondary fabrication.
* **PCB Layout Redesign:** Re-engineer the custom PCB layout to optimize trace routing, minimize electromagnetic interference (EMI) loops, and accommodate the expanding electronic component infrastructure.
* **Power Capacity Expansion:** Upgrade the power delivery network by expanding the battery bank or transitioning to high-discharge Lithium-Polymer (LiPo) batteries to safely handle larger dynamic loads and extend operational runtime.

### 3. Sensor Fusion & Advanced Autonomy (Perception & Compute)
* **Ultrasonic Sensor Optimization:** Restructure the proximity sensing array by mounting the current ultrasonic sensors at strategic angles. This includes adding at least two dedicated side-facing sensors and increasing the sensor count across the chassis perimeter for better spatial awareness.
* **360° LiDAR Integration:** As a superior alternative or complement to the ultrasonic array, integrating a 360-degree LiDAR sensor is planned to achieve high-resolution spatial mapping and advanced obstacle avoidance.
* **Migration to ROS 2 & Computer Vision:** Elevate the entire control architecture by migrating from bare-metal microcontroller scripts to **ROS 2 (Robot Operating System)**. Pairing ROS 2 with a LiDAR sensor and a monocular/stereo camera will unlock advanced capabilities, including SLAM (Simultaneous Localization and Mapping), visual odometry, and real-time computer vision processing.



