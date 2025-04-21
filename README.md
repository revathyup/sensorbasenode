# Smart Gardening System - Sensor Node + Zephyr Driver

This project implements a Smart Gardening System using two Raspberry Pi Pico boards:
1. **Sensor Node**: Collects data from multiple sensors and communicates with the base node
2. **Base Node**: Implements a Zephyr sensor driver to expose sensor data through the Zephyr Sensors API

## Project Architecture

### System Overview
The Smart Gardening System consists of two main components:

1. **Sensor Node** - Responsible for data collection:
   - Implements sensor interfaces (I2C, ADC) for environmental monitoring
   - Processes sensor readings and detects threshold violations
   - Exposes a UART-based protocol for communication with the base node
   - Built using the Raspberry Pi Pico SDK

2. **Base Node** - Provides the driver interface and user interaction:
   - Implements a Zephyr driver using the standard Sensor API
   - Communicates with the sensor node via UART
   - Provides shell commands for configuration and monitoring
   - Built using Zephyr RTOS

### Communication Protocol
The two nodes communicate using a custom UART protocol defined in `protocol.h`. This protocol supports:
- Sensor data transmission
- Configuration requests/responses
- Interrupt notifications
- Error reporting
- Acknowledgments

The protocol uses structured messages with checksums and sequence numbers for reliability.

## Sensors Supported

### Light Sensor (TSL2591)
- I2C-based light sensor
- Reports light level in lux
- Configurable high and low thresholds

### Temperature Sensor (MCP9700)
- Analog temperature sensor
- Reports temperature in degrees Celsius
- Configurable high threshold

### Soil Moisture Sensor (STEMMA)
- I2C-based capacitive soil moisture sensor
- Reports soil moisture percentage
- Configurable low threshold

## Hardware Setup

### Sensor Node Hardware
- Raspberry Pi Pico
- Adafruit TSL2591 Light Sensor (I2C)
- MCP9700/9700A Temperature Sensor (Analog)
- Adafruit STEMMA Soil Moisture Sensor (I2C)

### Base Node Hardware
- Raspberry Pi Pico (running Zephyr RTOS)
- UART connection to Sensor Node
- USB connection to host computer for monitoring

### Wiring Diagram

#### Sensor Node
- **I2C Configuration**:
  - SDA: GPIO 0 (Pin 1)
  - SCL: GPIO 1 (Pin 2)
  - Connected to TSL2591 (0x29) and STEMMA Soil Sensor (0x36)

- **ADC Configuration**:
  - Temperature Sensor: GPIO 26 (ADC0)

- **UART Configuration**:
  - TX: GPIO 8
  - RX: GPIO 9
  - UART ID: uart1
  - Baud Rate: 115200

#### Base Node
- **UART Configuration**:
  - TX: GPIO 8
  - RX: GPIO 9
  - Connected to Sensor Node

## Building the Project

### Prerequisites
- Raspberry Pi Pico SDK (for Sensor Node)
- Zephyr RTOS (for Base Node)
- CMake (version 3.13 or later)
- Arm GNU Toolchain (gcc-arm-none-eabi)
- Python 3.x (for Zephyr tools)

### Building the Sensor Node
1. Set up the Pico SDK:
   ```bash
   export PICO_SDK_PATH=/path/to/pico-sdk
   ```

2. Build the project:
   ```bash
   cd sensor_node
   mkdir build
   cd build
   cmake ..
   make -j4
   ```

3. Flash to the Pico:
   - Connect the Pico while holding the BOOTSEL button
   - Copy `sensor_node.uf2` to the mounted drive

### Building the Base Node
1. Set up Zephyr environment:
   ```bash
   source /path/to/zephyr/zephyr-env.sh
   ```

2. Build the project:
   ```bash
   cd base_node
   west build -b rpi_pico
   ```

3. Flash to the Pico:
   - Connect the Pico while holding the BOOTSEL button
   - Copy the generated `.uf2` file from `build/zephyr/zephyr.uf2` to the mounted drive

## Using the System

### Sensor Node Operation
- The sensor node automatically starts collecting data upon power-up
- LED blinks to indicate successful operation
- LED blinks rapidly when thresholds are exceeded

### Base Node Commands
The base node provides a shell interface with the following commands:

- Get current sensor readings:
  ```
  sensor get_readings
  ```

- Set high temperature threshold (in °C):
  ```
  sensor set_threshold temp_high <value>
  ```

- Set low soil moisture threshold (in %):
  ```
  sensor set_threshold soil_low <value>
  ```

- Set low light threshold (in lux):
  ```
  sensor set_threshold light_low <value>
  ```

- Set high light threshold (in lux):
  ```
  sensor set_threshold light_high <value>
  ```

- Set sampling rate (in Hz):
  ```
  sensor set_sampling_rate <value>
  ```

## Zephyr Driver Implementation

The base node implements a complete Zephyr sensor driver that exposes the Fetch/Get API:

- `sample_fetch()` - Gets the latest data from the sensor node
- `channel_get()` - Retrieves specific sensor data (temperature, light, soil moisture)
- `attr_set()` - Configures thresholds and parameters
- `attr_get()` - Retrieves current configuration

This driver maps the sensor channels as follows:
- Temperature: `SENSOR_CHAN_AMBIENT_TEMP`
- Soil Moisture: `SENSOR_CHAN_HUMIDITY`
- Light: `SENSOR_CHAN_LIGHT`
