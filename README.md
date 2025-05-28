# Weather IoT Sensor

This project provides firmware for an Arduino Nano-based IoT weather sensing device. It retrieves GPS data and environmental conditions (temperature, humidity, pressure) and transmits that data to a central receiver or logger upon request.

## Features

- GPS location tracking
- Weather data collection from environmental sensors
- On-demand data transmission to external receivers through UART
- Modular code structure with PlatformIO

## Hardware Requirements

- Arduino Nano
- GPS module (NEO6M)
- Barometric pressure sensor (BMP180)
- Temperature and Humidity sensor (DHT22 - AM2302)
- Optional: LCD screen (1602) with I2C adapter

## Software Requirements

- [PlatformIO](https://platformio.org/) (for building and flashing the firmware)
- Arduino framework (automatically handled by PlatformIO)

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/scuya2050/weather-iot-sensor.git
   cd weather-iot-sensor
   ```

2. Open the project using PlatformIO (e.g., in VS Code).

3. Connect your Arduino Nano via USB.

4. Upload the code to the board:

   ```bash
   pio run --target upload
   ```

## Usage

- The device starts collecting GPS and sensor data on boot.
- Once GPS signal is received and location is determined, it displays on the LCD and starts displaying weather data on the LCD
- Only if the device managed to retrieve GPS data, it transmits the latest readings on-demand.
- The request needs to include a request timestamp since the device does't know the date and time by itself.

## Project Structure

```
weather-iot-sensor/
├── include/            # Header files
├── lib/                # External libraries
├── src/                # Main source code
│   └── main.cpp
├── test/               # Tests (optional)
├── platformio.ini      # PlatformIO project config
└── .vscode/            # VSCode config (optional)
```
