# pmw3389-rpl4-lib
Library for communicating with the PMW3389 optical mouse sensor

## Overview
This library provides a C++ interface for controlling the PMW3389 optical mouse sensor via SPI communication. The library uses a hardware abstraction layer, making it compatible with various peripheral libraries including:
- Raspberry Pi 4 (using [RPL4](https://github.com/yutatech/RPL4))
- STM32 HAL
- Arduino
- Custom implementations

## Features
- Read sensor motion data (delta X/Y)
- Motion burst read for efficient data acquisition
- Configure CPI (Counts Per Inch) resolution (50-16000 in steps of 50)
- Upload firmware (SROM) to the sensor
- Read product ID, revision ID, and SROM ID
- Power management modes (Run, Rest1, Rest2, Rest3)
- SPI communication with proper timing delays
- **Hardware abstraction layer for portability**

## Architecture

The library is split into two parts:

1. **Core PMW3389 Logic** (`pmw3389.h/cc`) - Hardware-independent sensor communication protocol
2. **Peripheral Interface** (`peripheral_interface.h`) - Hardware abstraction that you implement for your platform

This design allows the library to work with any hardware platform by implementing a simple set of functions.

## Requirements
- C++11 or higher
- CMake 3.22 or higher
- SPI and GPIO peripheral access on your target platform

## Installation

### As a CMake Subdirectory

This library is designed to be included as a subdirectory in your CMake project:

```cmake
add_subdirectory(pmw3389-rpl4-lib)
target_link_libraries(your_target pmw3389_rpl4_lib)
```

### Implementing Peripheral Functions

You need to implement the peripheral interface functions for your hardware platform. See `peripheral_interface.h` for the required functions:

- `PeripheralInit()` - Initialize SPI and GPIO
- `PeripheralEnableCs()` - Enable chip select
- `PeripheralDisableCs()` - Disable chip select
- `PeripheralSpiTransmitReceive()` - SPI transfer
- `PeripheralDelayMicroseconds()` - Delay function

The library provides weak default implementations that do nothing. Override them in your application to provide actual functionality.

## Building Examples

The repository includes example programs demonstrating how to use the library with RPL4 (Raspberry Pi 4). To build the examples:

```bash
# Clone the repository
git clone https://github.com/yutatech/pmw3389-rpl4-lib.git
cd pmw3389-rpl4-lib

# Run the build script (automatically clones RPL4 and builds)
./build.sh

# Run the example
./.build/example/pmw3389_example
```

The build script will:
- Clone the RPL4 dependency if not present
- Create the `.build` directory
- Configure CMake with BUILD_EXAMPLES enabled
- Build the library and examples

The example program demonstrates:
- Initializing the PMW3389 sensor
- Reading product and revision IDs
- Configuring CPI (resolution)
- Reading motion data
- Using motion burst read mode

**Note:** The examples require a Raspberry Pi 4 with a connected PMW3389 sensor to run successfully.

## Usage

### Basic Example (Raspberry Pi 4 with RPL4)

First, implement the peripheral interface. Here's an example for RPL4 (see `example/peripheral_rpl4.cc` for complete implementation):

```cpp
#include "pmw3389_rpl4_lib/peripheral_interface.h"
#include "rpl4/peripheral/spi.hpp"
#include "rpl4/peripheral/gpio.hpp"
#include "rpl4/rpl4.hpp"

namespace pmw3389_rpl4_lib {

bool PeripheralInit() {
  rpl::Init();
  // Configure SPI and GPIO for your setup
  // ... (see example/peripheral_rpl4.cc)
  return true;
}

void PeripheralEnableCs() {
  // Set CS pin low
}

void PeripheralDisableCs() {
  // Set CS pin high
}

bool PeripheralSpiTransmitReceive(const uint8_t* tx_data, 
                                  uint8_t* rx_data, 
                                  size_t length) {
  // Perform SPI transfer
  return true;
}

void PeripheralDelayMicroseconds(uint32_t microseconds) {
  // Delay implementation
}

}  // namespace pmw3389_rpl4_lib
```

Then use the library:

```cpp
#include "pmw3389_rpl4_lib/pmw3389.h"

int main() {
  // Create PMW3389 instance
  pmw3389_rpl4_lib::PMW3389 sensor;

  // Initialize sensor (calls PeripheralInit internally)
  if (!sensor.Init()) {
    std::cerr << "Failed to initialize sensor" << std::endl;
    return 1;
  }

  // Read product ID
  uint8_t product_id = sensor.ReadProductId();
  std::cout << "Product ID: 0x" << std::hex << static_cast<int>(product_id) << std::endl;

  // Set CPI to 1600
  sensor.SetCPI(1600);

  // Read motion data
  int16_t delta_x, delta_y;
  if (sensor.ReadMotion(&delta_x, &delta_y)) {
    std::cout << "Motion - X: " << delta_x << ", Y: " << delta_y << std::endl;
  }

  return 0;
}
```

### Motion Burst Read

For efficient reading of all motion data at once:

```cpp
pmw3389_rpl4_lib::MotionBurstData burst_data;
if (sensor.ReadMotionBurst(&burst_data)) {
  std::cout << "Delta X: " << burst_data.delta_x << std::endl;
  std::cout << "Delta Y: " << burst_data.delta_y << std::endl;
  std::cout << "SQUAL: " << static_cast<int>(burst_data.squal) << std::endl;
}
```

### Firmware Upload

```cpp
// Example: Upload firmware (firmware_data should be your SROM data)
const uint8_t firmware_data[] = { /* ... your SROM data ... */ };
size_t firmware_length = sizeof(firmware_data);

if (sensor.UploadFirmware(firmware_data, firmware_length)) {
  std::cout << "Firmware uploaded successfully" << std::endl;
  uint8_t srom_id = sensor.ReadSromId();
  std::cout << "SROM ID: 0x" << std::hex << static_cast<int>(srom_id) << std::endl;
}
```

### Implementing for Other Platforms

#### STM32 HAL Example

```cpp
#include "pmw3389_rpl4_lib/peripheral_interface.h"
#include "stm32f4xx_hal.h"

// Static instances
static SPI_HandleTypeDef hspi1;
static GPIO_TypeDef* cs_port = GPIOA;
static uint16_t cs_pin = GPIO_PIN_4;

namespace pmw3389_rpl4_lib {

bool PeripheralInit() {
  // Configure SPI peripheral
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    return false;
  }
  
  // Configure CS pin as output
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = cs_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(cs_port, &GPIO_InitStruct);
  
  // CS starts high (inactive)
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  
  return true;
}

void PeripheralEnableCs() {
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
}

void PeripheralDisableCs() {
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

bool PeripheralSpiTransmitReceive(const uint8_t* tx_data, 
                                  uint8_t* rx_data, 
                                  size_t length) {
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
    &hspi1, 
    (uint8_t*)tx_data, 
    rx_data, 
    length, 
    HAL_MAX_DELAY
  );
  return status == HAL_OK;
}

void PeripheralDelayMicroseconds(uint32_t microseconds) {
  // Use DWT cycle counter for precise delays
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = microseconds * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - start) < cycles);
}

}  // namespace pmw3389_rpl4_lib
```

#### Arduino Example

```cpp
#include "pmw3389_rpl4_lib/peripheral_interface.h"
#include <SPI.h>

// Pin definitions
const int CS_PIN = 10;

namespace pmw3389_rpl4_lib {

bool PeripheralInit() {
  // Configure CS pin
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  // Initialize SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  
  return true;
}

void PeripheralEnableCs() {
  digitalWrite(CS_PIN, LOW);
}

void PeripheralDisableCs() {
  digitalWrite(CS_PIN, HIGH);
}

bool PeripheralSpiTransmitReceive(const uint8_t* tx_data, 
                                  uint8_t* rx_data, 
                                  size_t length) {
  for (size_t i = 0; i < length; i++) {
    rx_data[i] = SPI.transfer(tx_data[i]);
  }
  return true;
}

void PeripheralDelayMicroseconds(uint32_t microseconds) {
  delayMicroseconds(microseconds);
}

}  // namespace pmw3389_rpl4_lib
```

## API Reference

### Initialization
- `PMW3389()` - Constructor
- `Init()` - Initialize the sensor (calls PeripheralInit internally)
- `PowerUpReset()` - Power up and reset the sensor

### Peripheral Interface (to be implemented by user)
- `PeripheralInit()` - Initialize SPI and GPIO hardware
- `PeripheralEnableCs()` - Set CS pin low (active)
- `PeripheralDisableCs()` - Set CS pin high (inactive)
- `PeripheralSpiTransmitReceive(tx_data, rx_data, length)` - Perform SPI transfer
- `PeripheralDelayMicroseconds(microseconds)` - Delay function

### Reading Data
- `ReadProductId()` - Read Product ID register (expected: 0x47)
- `ReadInverseProductId()` - Read Inverse Product ID register
- `ReadRevisionId()` - Read Revision ID register
- `ReadSromId()` - Read SROM ID register
- `ReadMotion(delta_x, delta_y)` - Read motion delta values
- `ReadMotionBurst(data)` - Read all motion data in burst mode

### Configuration
- `SetCPI(cpi)` - Set CPI resolution (50-16000 in steps of 50)
- `GetCPI()` - Get current CPI resolution
- `UploadFirmware(firmware_data, length)` - Upload SROM firmware

### Power Management
- `SetRunMode()` - Set to run mode (normal operation)
- `SetRest1Mode()` - Set to rest1 mode (low power)
- `SetRest2Mode()` - Set to rest2 mode (lower power)
- `SetRest3Mode()` - Set to rest3 mode (lowest power)

### Low-Level Access
- `WriteRegister(address, data)` - Write to a register
- `ReadRegister(address, data)` - Read from a register

## Communication Protocol

The PMW3389 uses SPI communication with specific timing requirements:
- Read operation: MSB = 0, followed by 35μs delay (tSRAD) before reading data
- Write operation: MSB = 1, data follows immediately
- Inter-operation delays: 120μs between operations (tSWW/tSWR)

## Register Map

Key registers include:
- `0x00` - Product_ID (expected: 0x47)
- `0x02` - Motion (motion detection status)
- `0x03-0x06` - Delta X/Y (motion data)
- `0x0E-0x0F` - Resolution (CPI configuration)
- `0x13` - SROM_Enable (firmware upload control)
- `0x50` - Motion_Burst (burst read starting point)
- `0x62` - SROM_Load_Burst (firmware upload)

## Reference
- [PMW3360 Datasheet](https://d3s5r33r268y59.cloudfront.net/datasheets/9604/2017-05-07-18-19-11/PMS0058-PMW3360DM-T2QU-DS-R1.50-26092016._20161202173741.pdf) (similar sensor, for reference)
- [RPL4 Library](https://github.com/yutatech/RPL4)
- [RPL4 Examples](https://github.com/yutatech/RPL4/tree/main/example)

## Example Implementations

### Raspberry Pi 4 (RPL4)

The `example` directory contains a complete implementation using RPL4:
- `example/peripheral_rpl4.cc` - RPL4 peripheral interface implementation
- `example/main.cc` - Example usage demonstrating all sensor features

To build and run:
```bash
./build.sh
./.build/example/pmw3389_example
```

### Future Examples

Additional examples for other platforms will be added:
- STM32 HAL implementation (planned)
- Arduino implementation (planned)

## Project Structure

```
pmw3389-rpl4-lib/
├── include/pmw3389_rpl4_lib/
│   ├── pmw3389.h              # Main sensor class (hardware-independent)
│   ├── peripheral_interface.h # Hardware abstraction interface
│   └── registers.h            # PMW3389 register definitions
├── src/
│   ├── pmw3389.cc             # Sensor communication logic
│   └── peripheral_interface.cc # Weak default implementations
├── example/
│   ├── peripheral_rpl4.cc     # RPL4 peripheral implementation
│   ├── main.cc                # Example usage
│   └── CMakeLists.txt         # Example build configuration
├── CMakeLists.txt             # Library build configuration
└── README.md
```

## License
[Add your license information here]
