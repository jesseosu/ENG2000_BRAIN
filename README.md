# ENG2000_BRAIN

**Modular IoT Bridge Control System for ESP32**

## Overview

ENG2000_BRAIN is a modular, web-controlled IoT system designed for ESP32-based hardware, focused on controlling and monitoring a model bridge with sensors, motor, and web interfaces. It features real-time sensor monitoring, motor control, configuration management, and autonomous/override bridge operation—all accessible through modern web UIs served from the ESP32 itself.

## Features

- **Ultrasonic Sensor Monitoring**: Real-time distance measurement, calibration, and status feedback.  
- **Motor Control**: Start/stop, direction, speed control, and encoder feedback for precise bridge movement.
- **Bridge Control**: Autonomous and manual operations, with sensor fusion for safety and detection (car/boat presence, limit switches, hall effect sensors).
- **Web UI**: Responsive HTML/JS interface for all modules (Ultrasonic, Motor, Bridge, Configuration, System Info).
- **Pin Configuration**: Easily assign and manage ESP32 GPIO pins for all subsystems via web.
- **Live Graphing**: Sensor data visualization using Chart.js.
- **Network Management**: ESP32 runs as a WiFi AP (`ESP32-Control`, default password: `password123`), accessible at `192.168.4.1`.
- **SPIFFS Filesystem**: Serves static web assets directly from ESP32 SPIFFS.

## Quick Start

1. **Hardware Requirements**  
   - ESP32 Dev Board (Upesy WROOM)
   - Ultrasonic sensors (HC-SR04 for car & boat detection)
   - DC Motor with encoder and L298N driver
   - Hall effect sensors, RGB LEDs

2. **Build and Flash**  
   - Install [PlatformIO](https://platformio.org/) and [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
   - Clone the repo:
     ```
     git clone https://github.com/finbar001/ENG2000_BRAIN.git
     ```
   - Set up your board (default: `upesy_wroom`) in `platformio.ini`
   - Flash firmware:
     ```
     pio run --target upload
     ```
   - Upload web assets to SPIFFS:
     ```
     pio run --target uploadfs
     ```

3. **Connect and Use**  
   - Connect to WiFi: SSID `ESP32-Control`, password `password123`
   - Open browser: `http://192.168.4.1`
   - Use the web interface to view system status, control modules, calibrate sensors, and configure pins.

## Web Interface Modules

- **Control Center** (`index.html`): Hub for all device modules.
- **Ultrasonic Sensor** (`ultrasonic.html`): View live distance, calibrate, and check sensor status.
- **Motor Control** (`motor.html`): Start/stop motor, set speed/direction, view encoder/pulse count.
- **Bridge Control** (`bridge.html`): Control bridge position manually or autonomously, monitor all bridge sensors, calibrate car/boat detection.
- **Pin Configuration** (`config.html`): Assign GPIO pins for each subsystem, detect conflicts and save settings.
- **System Info** (`system.html`): View uptime, connected WiFi clients, and diagnostics.

## Configuration

- **Pin Assignments**: Use `Configuration` module to set pins for motor, sensors, LEDs, etc. Input-only pins (GPIO 34, 35, 36, 39) are restricted for outputs.
- **Calibration**: Ultrasonic sensors require calibration for car and boat detection. Clear area before calibration and follow web UI prompts.
- **Restart/Defaults**: Configuration changes require ESP32 restart (button in UI). Reset to defaults as needed.

## Firmware Highlights

- WiFi AP settings:  
  ```
  SSID: ESP32-Control
  Password: password123
  Max Connections: 4
  ```
- Ultrasonic sensor parameters:
  - Range: 2–400 cm
  - Accuracy: ±3mm
  - Calibration: 50 samples, baseline-relative threshold
- Motor control: PWM (1 kHz), direction via L298N, encoder feedback
- Bridge servo: 50 Hz PWM, position pulse widths configurable
- Hall effect: ADC threshold adjustable via web UI
- All configuration stored in NVS and SPIFFS

## Development Notes

- Project uses ESP-IDF 5.x and PlatformIO.
- Static web assets served from SPIFFS (see `partitions.csv`).
- Custom libraries should be placed under `lib/` as per PlatformIO conventions.
- Header files for shared code in `include/`.