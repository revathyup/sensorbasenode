# Smart Gardening System

This project implements a distributed garden monitoring system using multiple Raspberry Pi Pico microcontrollers. The system consists of a sensor node that collects environmental data and a base node that processes and displays this data.

## System Architecture

The Smart Gardening System is built using:

1. **Sensor Node** (Raspberry Pi Pico with Pico SDK)
   - Collects data from multiple sensors:
     - TSL2591 Light Sensor (I2C): measures light intensity
     - MCP9700 Temperature Sensor (ADC): measures ambient temperature
     - STEMMA Soil Moisture Sensor (I2C): measures soil moisture
   - Communicates with base node via UART

2. **Base Node** (Raspberry Pi Pico with Zephyr RTOS)
   - Implements a proper Zephyr sensor driver using the Sensors API
   - Processes and displays sensor data via terminal interface
   - Allows user to set thresholds for alerts
   - Provides user interface via UART/USB console

## Communication Protocol

The nodes communicate using a binary protocol with the following format:
- Start byte (0xAA)
- Command byte
- Length byte
- Data payload (variable length)
- Checksum (XOR of all previous bytes)

## Hardware Connections

### Sensor Node
- TSL2591 Light Sensor: I2C (GPIO 4, 5)
- MCP9700 Temperature Sensor: ADC0 (GPIO 26)
- STEMMA Soil Moisture Sensor: I2C (GPIO 4, 5)
- UART for node communication: TX (GPIO 8), RX (GPIO 9)

### Base Node
- UART for node communication: TX (GPIO 8), RX (GPIO 9)
- UART for console interface: USB/UART0

## Terminal Interface

The base node provides a command-line interface via USB/UART with the following commands:
- `help` - Display help information
- `ping` - Ping the sensor node
- `show light` - Show light sensor data
- `show temp` - Show temperature sensor data
- `show soil` - Show soil moisture sensor data
- `show all` - Show all sensor data
- `show thresholds` - Show all threshold values
- `set light low|high <value>` - Set light threshold (lux)
- `set temp low|high <value>` - Set temperature threshold (°C)
- `set soil dry|wet <value>` - Set soil moisture threshold (0-1000)

## Building and Flashing

### Sensor Node
```bash
cd sensor_node
mkdir -p build && cd build
export PICO_SDK_PATH=/path/to/pico-sdk
cmake ..
make
```

Copy the resulting `sensor_node.uf2` file to the Raspberry Pi Pico.

### Base Node
```bash
cd zephyr_base_node/app
west build -b rpi_pico
```

Copy the resulting `.uf2` file to another Raspberry Pi Pico.

## Connection and Usage

1. Connect the sensors to the sensor node Pico according to the hardware connections
2. Connect the two Picos via UART (GPIO 8, 9)
3. Connect the base node Pico to your computer via USB
4. Open a terminal (PuTTY, screen, etc.) with 115200 baud rate
5. Use the terminal commands to interact with the system