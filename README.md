# Smart Gardening System

A smart sensor node system using Raspberry Pi Pico boards to collect environmental data from multiple sensors.

## Project Structure

The project consists of two main components:

1. **Sensor Node**: A Raspberry Pi Pico that collects data from multiple sensors:
   - TSL2591 Light Sensor (I2C)
   - MCP9700 Temperature Sensor (ADC)
   - STEMMA Soil Moisture Sensor (I2C)

2. **Base Node**: A Raspberry Pi Pico that receives data from the sensor node and allows user interaction through a terminal interface.

## Connection Diagram

### Sensor Node
- **TSL2591 Light Sensor**:
  - VCC → 3.3V
  - GND → GND
  - SDA → GPIO 4
  - SCL → GPIO 5

- **MCP9700 Temperature Sensor**:
  - VCC → 3.3V
  - GND → GND
  - OUT → GPIO 26 (ADC0)

- **STEMMA Soil Moisture Sensor**:
  - VCC → 3.3V
  - GND → GND
  - SDA → GPIO 4
  - SCL → GPIO 5

- **UART Connection to Base Node**:
  - TX → GPIO 8
  - RX → GPIO 9

### Base Node
- **UART Connection to Sensor Node**:
  - TX → GPIO 8
  - RX → GPIO 9

## Building and Flashing

1. Build the sensor node:
   ```
   cd sensor_node
   mkdir -p build
   cd build
   export PICO_SDK_PATH=../../pico-sdk
   cmake ..
   make -j4
   ```

2. Build the base node:
   ```
   cd base_node
   mkdir -p build
   cd build
   export PICO_SDK_PATH=../../pico-sdk
   cmake ..
   make -j4
   ```

3. Flash the UF2 files to the respective Pico boards by:
   - Hold the BOOTSEL button on the Pico while connecting it to your computer
   - Copy the .uf2 file from the build directory to the RPI-RP2 drive that appears
   - Sensor node: `sensor_node/build/sensor_node.uf2`
   - Base node: `base_node/build/base_node.uf2`

## Accessing the Terminal Interface

1. Connect the base node to your computer via USB
2. Open a terminal program (like PuTTY or screen)
3. Connect to the appropriate serial port at 115200 baud rate

## Available Terminal Commands

- `help`: Show available commands
- `ping`: Send ping to sensor node
- `show`: Show current sensor readings
- `set light high <value>`: Set light high threshold (lux)
- `set light low <value>`: Set light low threshold (lux)
- `set temp high <value>`: Set temperature high threshold (°C)
- `set temp low <value>`: Set temperature low threshold (°C)
- `set soil dry <value>`: Set soil dry threshold (0-1000)
- `set soil wet <value>`: Set soil wet threshold (0-1000)

## Communication Protocol

The sensor and base nodes communicate using a custom binary protocol with the following structure:

- Start byte (0xAA)
- Command byte
- Data length byte
- Data payload (variable length)
- Checksum byte (XOR of all previous bytes)

This ensures reliable and error-checked communication between the nodes.