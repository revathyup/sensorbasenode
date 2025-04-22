# Smart Gardening System - Light Sensor Node

## Overview

This project implements a light sensor node for a Smart Gardening System using a Raspberry Pi Pico microcontroller. The system monitors light levels using a TSL2591 high-precision light sensor and provides real-time feedback and alerts when light conditions fall outside optimal ranges.

## Features

- Monitors light intensity using the TSL2591 high-precision light sensor
- Calculates light levels in lux for accurate plant light measurements
- Provides threshold alerts for low and high light conditions
- Outputs all data via USB for easy monitoring in a terminal like PuTTY
- Configurable gain and integration time settings for different light conditions

## Project Structure

- `light_sensor_node/`: Pico SDK implementation of the light sensor node
  - `main.c`: Main application code
  - `tsl2591.c/h`: Driver for the TSL2591 light sensor
  - `CMakeLists.txt`: Build configuration

- `sim_light_sensor/`: Simulated version of the light sensor node for testing
  - `main.c`: Simulated application with interactive commands
  - `tsl2591_sim.c/h`: Simulated sensor driver 
  - `Makefile`: Build configuration for the simulator

## Hardware Components

- Raspberry Pi Pico microcontroller
- TSL2591 light sensor (I2C interface)
- Connecting wires
- Micro USB cable (for power and data)

## Connections

- Connect TSL2591 to Raspberry Pi Pico:
  - VIN → 3.3V
  - GND → GND
  - SCL → GP5
  - SDA → GP4

## Building and Running

### For the Real Hardware

1. Export the Pico SDK path:
   ```
   export PICO_SDK_PATH=/path/to/pico-sdk
   ```

2. Build the project:
   ```
   cd light_sensor_node
   mkdir -p build
   cd build
   cmake ..
   make
   ```

3. Connect the Raspberry Pi Pico to your computer while holding the BOOTSEL button
4. Copy the generated `light_sensor_node.uf2` file to the mounted Pico drive
5. The Pico will restart and run the light sensor application
6. Connect to the Pico using a serial terminal (PuTTY) at 115200 baud rate

### For the Simulator

1. Build the simulator:
   ```
   cd sim_light_sensor
   make
   ```

2. Run the simulator:
   ```
   ./light_sensor_sim
   ```

3. Use the following commands in the simulator:
   - `help`: Show available commands
   - `normal`: Set normal light conditions
   - `low`: Set low light conditions
   - `bright`: Set bright light conditions
   - `random`: Set random light conditions
   - `quit`: Exit the simulator

## Light Thresholds

- Low light threshold: 20 lux (plants need more light)
- High light threshold: 1000 lux (potentially too much light for some plants)

These thresholds can be adjusted in the code by modifying the `LIGHT_LOW_THRESHOLD` and `LIGHT_HIGH_THRESHOLD` constants.

## Future Enhancements

- Add support for soil moisture and temperature sensors
- Implement wireless communication between sensor nodes and a base station
- Develop an automated control system for grow lights and irrigation
- Create a mobile app interface for monitoring and control