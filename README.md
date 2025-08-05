# TARS – Tracking Antenna Reception System

TARS (Tracking Antenna Reception System) is a low-cost, modular, GPS-based ground station designed to track high-powered rockets and UAVs in real-time using directional antennas. Developed with ESP32 and stepper motor control, TARS calculates azimuth and elevation angles based on live GPS telemetry from the rocket and rotates the antenna accordingly.

---

## 🚀 Project Overview

### 🎯 Objective
To create a ground-based antenna tracking system that accurately orients a directional antenna toward a moving airborne target using GPS coordinates, enabling robust telemetry reception over long distances.

### 📡 Core Features
- **Real-Time Antenna Tracking**: Uses received GPS coordinates to calculate azimuth and elevation.
- **Mechanically-Coupled Axis Compensation**: Adjusts for azimuth-elevation mechanical linkage.
- **Stepper Motor Control**: Dual NEMA 17 motors with TB6600 drivers.
- **Data Logging**: Stores incoming GPS and IMU data to a microSD card.
- **LoRa + ESP-NOW Telemetry**: Receives GPS + IMU packets via LoRa and logs ESP-NOW packets.
- **Compact Hardware Design**: Based on ESP32 with low-latency control logic.

---

## 🛠️ System Architecture

### Transmitter (Rocket Payload)
- **GPS Module**: Position (lat, lon, alt), velocity, satellite count.
- **IMU (MPU9250)**: Orientation (roll, pitch, yaw), acceleration, gyro.
- **Magnetometer (HMC5883L)**: Compass heading.
- **LoRa + ESP-NOW**: Sends telemetry packets.

### Receiver (Ground Station)
- **ESP32 Controller**: Core logic + decision-making.
- **microSD**: Logs all incoming telemetry data.
- **Stepper Motors**: NEMA 17, controlled by TB6600 drivers.
- **Antenna Rotation Logic**: 
  - Azimuth-first movement (with elevation coupling correction),
  - Elevation-second adjustment.

---

## 🧠 Software Features

### Angle Calculation Algorithm
- Parses incoming GPS coordinates.
- Computes bearing and elevation using:
  - Haversine formula
  - Trigonometric functions
- Applies **azimuth-elevation coupling compensation** due to non-independent axes.

### Telemetry Packet Format
Includes:
- ID, packet count
- GPS: lat, lon, alt, speed, satellite count
- IMU: accel, gyro, magnetometer
- Orientation: yaw, pitch, roll
- Time (from GPS)

---

## 📁 Project Structure (Proposed)

