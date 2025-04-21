# Smart Gardening System

This project implements a smart sensor node system using two Raspberry Pi Pico boards to collect environmental data from multiple sensors and expose it through a Zephyr driver.

## Architecture

The system consists of:

1. **Sensor Node** - A Raspberry Pi Pico running Pico SDK that interfaces with:
   - Adafruit TSL2591 light sensor (I2C)
   - MCP9700/9700A temperature sensor (Analog)
   - Adafruit STEMMA soil moisture sensor (I2C)

2. **Base Node** - A Raspberry Pi Pico running Zephyr RTOS that:
   - Implements a Zephyr sensor driver
   - Communicates with the sensor node via UART
   - Processes sensor data and handles interrupts
   - Displays data to a terminal via UART

## Hardware Requirements

- 2 × Raspberry Pi Pico boards
- Adafruit TSL2591 light sensor
- MCP9700/9700A temperature sensor
- Adafruit STEMMA soil moisture sensor
- Breadboard and jumper wires
- USB cables for power and programming

## Wiring Diagram

### Sensor Node (Raspberry Pi Pico)
- **Light Sensor (TSL2591)**:
  - VIN → 3.3V
  - GND → GND
  - SCL → GPIO 5 (I2C0 SCL)
  - SDA → GPIO 4 (I2C0 SDA)

- **Soil Moisture Sensor (STEMMA)**:
  - VIN → 3.3V
  - GND → GND
  - SCL → GPIO 7 (I2C1 SCL)
  - SDA → GPIO 6 (I2C1 SDA)

- **Temperature Sensor (MCP9700/9700A)**:
  - VDD → 3.3V
  - GND → GND
  - OUT → GPIO 26 (ADC0)

- **UART Connection to Base Node**:
  - TX → GPIO 0 (UART0 TX)
  - RX → GPIO 1 (UART0 RX)
  - GPIO 2 → Interrupt line to Base Node

### Base Node (Raspberry Pi Pico)
- **UART Connection to Sensor Node**:
  - RX → GPIO 0 (UART0 RX)
  - TX → GPIO 1 (UART0 TX)
  - GPIO 2 → Interrupt line from Sensor Node

- **UART Connection to Computer**:
  - TX → GPIO 4 (UART1 TX)
  - RX → GPIO 5 (UART1 RX)

## Building and Running

### Sensor Node
1. Navigate to the `sensor_node` directory
2. Build the project with:
   ```
   mkdir build
   cd build
   cmake ..
   make
   ```
3. Flash the resulting `.uf2` file to the Raspberry Pi Pico

### Base Node
1. Set up Zephyr development environment
2. Navigate to the `base_node` directory
3. Build the project with:
   ```
   west build -b rpi_pico
   ```
4. Flash the resulting binary to the Raspberry Pi Pico

## Communication Protocol

The sensor node and base node communicate using a simple binary protocol over UART:
- Start byte (0xAA)
- Command byte
- Data length byte
- Data bytes
- Checksum byte (XOR of all previous bytes)

## Interrupt System

The system implements an interrupt line between the sensor node and base node. Interrupts are triggered when:
- Soil moisture falls below a configurable threshold
- Temperature rises above a configurable threshold
- Light intensity exceeds a configurable threshold

## License

This project is licensed under the MIT License - see the LICENSE file for details.
