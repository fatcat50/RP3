# RP3 Power Meter  
**ESP32-based rowing power meter** measuring stroke dynamics and transmitting via BLE.  

### Key Features  
- Real-time **power output** & **stroke rate** via Bluetooth (BLE)  
- WiFi-based **OTA firmware updates**  
- Pulse sensor input for flywheel rotation tracking  
- Physics model for rowing-specific power calculations  

### Setup  
1. **Flash ESP32** with Arduino IDE (requires `ArduinoBLE` and `WiFi` libraries)  
2. Set WiFi credentials in `ssid`/`password`  
3. Upload sketch  

### Usage  
- Pair via BLE (**"RP3 Power Meter"**) to receive cycling power profile data  
- Access `http://AIROW-Powermeter.local` for wireless updates  
- Serial monitor: `115200 baud`  

*Adjust mechanical constants in code for your rower.*  
