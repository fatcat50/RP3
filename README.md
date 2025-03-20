# RP3 Power Meter

## Overview
The **RP3 Power Meter** is a performance measurement system for rowing ergometers, based on an **ESP32**. It calculates power, stroke rate, and other metrics in real-time using a magnetic sensor and transmits the data via **Bluetooth Low Energy (BLE)** and **WiFi**.

## Features
- **BLE Transmission**: Compatible with cycling power meter standards (BLE GATT 0x1818).
- **WiFi Connectivity**: Provides a web interface for firmware updates.
- **Power & Stroke Rate Calculation**: Uses a pulse sensor and predefined parameters for precise measurement.
- **Automatic State Detection**: Differentiates between the drive and recovery phases of a rowing stroke.

## Hardware Requirements
- **ESP32 Dev Board**
- **Magnetic Sensor (Hall Effect)**
- **RP3 Rowing Ergometer**
- **Power Source (USB or Battery)**

## Installation & Setup
1. **Flash the Firmware**  
   Use the **Arduino IDE** or **PlatformIO** to compile and upload the firmware to the ESP32.
   
2. **WiFi Configuration**  
   Update the `ssid` and `password` in the source code to connect the ESP32 to a WiFi network.

3. **BLE Connectivity**  
   Scan for the device name **"RP3 Power Meter"** using a BLE-compatible application.

4. **Web Interface (Firmware Update)**  
   - Open a browser and go to `http://<ESP32_IP_ADDRESS>/`.
   - Login with `admin/admin` and upload a new firmware version if needed.

## Usage
- Once powered on, the device starts collecting and transmitting power data automatically.
- The LED indicates connection status:  
  - **Flashing**: Searching for BLE connection  
  - **Solid**: Connected to a BLE central device
