# STM32 Road Pothole Detection and Localization System

An intelligent road pothole detection and localization system based on **STM32F4, FreeRTOS, IMU/GPS sensor fusion, and MQTT-based IoT communication**.

The system integrates real-time vibration monitoring, adaptive pothole detection, GPS-assisted localization, and cloud-based data visualization to support automated road condition assessment.

## 1. Project Overview

This project implements an embedded edge detection system using an STM32F4 microcontroller and a mobile robotic platform.

By integrating an MPU6050 six-axis IMU, an ATGM336H GPS/BeiDou positioning module, Hall-effect wheel encoders, and an ESP32 Wi-Fi module, the system detects road surface anomalies, estimates pothole locations, and uploads detection results to a cloud platform.

The system is designed to perform real-time sensor acquisition, vibration signal processing, pothole recognition, position estimation, and wireless communication.

### Key Features

- Real-time road vibration monitoring using MPU6050
- Adaptive pothole detection based on acceleration signal features
- GPS/BeiDou positioning with wheel encoder-based distance interpolation
- FreeRTOS-based multitasking architecture
- UART DMA communication for efficient data reception
- MQTT-based wireless data transmission via ESP32
- JSON-formatted pothole event reporting
- Map-based pothole visualization and road quality analysis

## 2. Technology Stack

| Category | Technology |
|---|---|
| Microcontroller | STM32F4 |
| Programming Language | C |
| Real-Time Operating System | FreeRTOS |
| IMU Sensor | MPU6050 |
| Positioning Module | ATGM336H (GPS/BeiDou) |
| Speed Measurement | Hall-effect Wheel Encoder |
| Wireless Communication | ESP32 Wi-Fi |
| Communication Interfaces | UART, I2C, DMA |
| IoT Protocol | MQTT |
| Data Format | JSON |
| Signal Processing | Sliding Window Filtering |
| Detection Algorithm | Adaptive Threshold-Based Detection |
| Localization | GPS and Encoder-Based Interpolation |
| Visualization | Map API, Scatter Plot, Heatmap |

## 3. System Architecture

The system consists of four major functional layers:

**Sensor Acquisition Layer**

- MPU6050 collects three-axis acceleration and angular velocity data.
- ATGM336H provides GPS/BeiDou positioning information.
- Hall-effect encoders measure wheel rotation and estimate vehicle speed.

**Embedded Processing Layer**

- STM32F4 executes sensor acquisition and signal processing tasks.
- FreeRTOS manages concurrent real-time tasks.
- A sliding-window algorithm extracts vibration features.
- Adaptive thresholds identify potential pothole events.
- GPS and encoder data are combined for location estimation.

**Wireless Communication Layer**

- ESP32 provides Wi-Fi connectivity.
- Pothole detection results are formatted as JSON.
- MQTT is used to transmit detection events to the cloud.

**Cloud Visualization Layer**

- Pothole coordinates are displayed on an interactive map.
- Impact intensity is used to represent road surface conditions.
- Historical detection records can be filtered and analyzed.

### Data Processing Pipeline

```text
        +----------------------+
        |     Road Surface     |
        +----------+-----------+
                   |
        +----------v-----------+
        |   Sensor Acquisition |
        | MPU6050 / GPS / Hall |
        +----------+-----------+
                   |
        +----------v-----------+
        |       STM32F4        |
        |       FreeRTOS       |
        +----------+-----------+
                   |
        +----------v-----------+
        |   Signal Processing  |
        | Sliding Window Filter|
        | Feature Extraction   |
        +----------+-----------+
                   |
        +----------v-----------+
        |   Pothole Detection  |
        | Adaptive Thresholds  |
        | Duplicate Suppression|
        +----------+-----------+
                   |
        +----------v-----------+
        | Location Estimation  |
        | GPS + Encoder Data   |
        +----------+-----------+
                   |
        +----------v-----------+
        |    ESP32 Wi-Fi       |
        |    MQTT / JSON       |
        +----------+-----------+
                   |
        +----------v-----------+
        |    Cloud Platform    |
        |  Map Visualization   |
        +----------------------+
```

## 4. Software Architecture

The embedded firmware is organized using FreeRTOS to support modular development and real-time task scheduling.

### FreeRTOS Task Design

| Task | Description |
|---|---|
| IMU Task | Periodically acquires acceleration and gyroscope data |
| GPS Task | Receives and parses GPS/BeiDou positioning data |
| Encoder Task | Calculates wheel speed and traveled distance |
| Detection Task | Processes vibration signals and detects potholes |
| MQTT Task | Packages and uploads pothole detection events |

### Real-Time Communication

- **I2C:** Communicates with the MPU6050 sensor.
- **UART DMA:** Receives GPS and ESP32 communication data with reduced CPU overhead.
- **FreeRTOS:** Coordinates concurrent sensor acquisition, detection, and communication tasks.

The multitasking architecture separates time-sensitive sensor acquisition from wireless communication to improve system responsiveness and maintainability.

## 5. Pothole Detection Algorithm

The pothole detection algorithm analyzes vertical acceleration signals collected by the MPU6050.

### 5.1 Signal Preprocessing

A sliding-window filtering method is applied to the Z-axis acceleration signal to reduce high-frequency noise and improve signal stability.

### 5.2 Feature Extraction

The following features are extracted from the acceleration signal:

- **Peak Acceleration:** Measures the maximum vibration impact.
- **Variance:** Evaluates acceleration fluctuations.
- **Root Mean Square (RMS):** Represents overall vibration intensity.
- **Impact Duration:** Measures the duration of abnormal vibration events.

### 5.3 Adaptive Threshold Detection

Instead of using a fixed acceleration threshold, the detection algorithm adjusts its sensitivity according to vehicle speed.

This approach helps reduce the influence of speed-dependent vibration variations on pothole detection.

### 5.4 Duplicate Detection Suppression

A duplicate suppression mechanism prevents a single pothole event from being reported multiple times due to consecutive acceleration peaks.

## 6. Pothole Localization

The localization algorithm combines GPS positioning with wheel encoder-based distance estimation.

### GPS-Based Positioning

The ATGM336H module provides geographic coordinates and positioning timestamps.

### Encoder-Based Distance Estimation

Hall-effect encoders estimate vehicle speed and traveled distance from wheel rotation measurements.

### Position Interpolation

When a pothole event is detected, the system associates the detection timestamp with the vehicle trajectory.

Encoder-based distance interpolation is used between GPS updates to estimate the relative position of pothole events.

This method is designed to improve relative localization consistency and reduce the impact of short-term GPS position fluctuations.

## 7. MQTT Communication

The ESP32 module provides Wi-Fi connectivity and transmits pothole detection events to the cloud using MQTT.

Each detection event contains the pothole coordinates, vibration intensity, vehicle speed, and timestamp.

### Example JSON Payload

```json
{
  "device_id": "STM32_POTHOLE_001",
  "timestamp": "2026-01-01T10:30:25Z",
  "latitude": 31.2304,
  "longitude": 121.4737,
  "impact_intensity": 2.85,
  "vehicle_speed": 1.25,
  "event_type": "pothole"
}
```

*Note: The values above are illustrative and do not represent actual measurement results.*

## 8. Map Visualization

The cloud visualization interface provides an intuitive way to inspect and analyze detected road surface anomalies.

Main visualization features include:

- **Pothole Scatter Map:** Displays detected pothole locations using geographic coordinates.
- **Road Quality Heatmap:** Visualizes the spatial distribution and severity of road surface anomalies.
- **Historical Data Filtering:** Supports reviewing pothole records by time and other detection attributes.

## 9. Project Highlights

**Real-Time Embedded Design**

Developed a FreeRTOS-based multitasking architecture on STM32F4 to coordinate sensor acquisition, signal processing, positioning, and wireless communication.

**Adaptive Signal Processing**

Implemented sliding-window filtering and multi-feature extraction to identify pothole-induced vibration patterns under different vehicle speeds.

**Multi-Sensor Localization**

Combined GPS positioning with encoder-based distance interpolation to improve the relative localization of nearby pothole events.

**IoT Integration**

Integrated ESP32 Wi-Fi and MQTT communication to enable remote pothole event reporting and map-based visualization.

**Modular System Architecture**

Separated sensor drivers, detection algorithms, localization logic, and communication modules to support future development and maintenance.

## 10. Future Improvements

- Implement Kalman filtering for enhanced GPS/IMU sensor fusion.
- Explore machine learning-based pothole classification.
- Add SD card storage for offline data logging.
- Improve localization accuracy under weak GPS conditions.
- Optimize power consumption for long-term outdoor operation.
- Support multi-device road condition monitoring.

## 11. Applications

- Intelligent road inspection
- Smart city infrastructure monitoring
- Autonomous mobile sensing platforms
- Road maintenance data collection
- Embedded IoT sensor networks

## 12. License

License information will be added based on the project's open-source distribution requirements.
