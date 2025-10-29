# pmw3389-rpl4-lib
Library for communicating with the PMW3389 (optical mouse sensor) using a Raspberry Pi 4

## Overview
This library provides a C++ interface for controlling the PMW3389 optical mouse sensor via SPI communication on a Raspberry Pi 4. It uses the [RPL4](https://github.com/yutatech/RPL4) library for peripheral access.

## Features
- Read sensor motion data (delta X/Y)
- Motion burst read for efficient data acquisition
- Configure CPI (Counts Per Inch) resolution (50-16000 in steps of 50)
- Upload firmware (SROM) to the sensor
- Read product ID, revision ID, and SROM ID
- Power management modes (Run, Rest1, Rest2, Rest3)
- SPI communication with proper timing delays

## Requirements
- Raspberry Pi 4
- PMW3389 optical mouse sensor
- [RPL4](https://github.com/yutatech/RPL4) library
- CMake 3.22 or higher

## Installation
This library is designed to be included as a subdirectory in your CMake project:

```cmake
add_subdirectory(pmw3389-rpl4-lib)
target_link_libraries(your_target pmw3389_rpl4_lib)
```

## Building Examples

The repository includes example programs demonstrating how to use the library. To build the examples:

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

### Basic Example

```cpp
#include <iostream>
#include <memory>
#include "pmw3389_rpl4_lib/pmw3389.h"
#include "rpl4/peripheral/spi.hpp"
#include "rpl4/peripheral/gpio.hpp"
#include "rpl4/rpl4.hpp"

int main() {
  // Initialize RPL4
  rpl::Init();

  // Get SPI instance
  std::shared_ptr<rpl::Spi> spi = rpl::Spi::GetInstance(rpl::Spi::Port::kSpi0);

  // Configure SPI GPIOs
  rpl::Gpio::SetAltFunction(8, rpl::Gpio::AltFunction::kAlt0);   // SPI0_CE0
  rpl::Gpio::SetAltFunction(9, rpl::Gpio::AltFunction::kAlt0);   // SPI0_MISO
  rpl::Gpio::SetAltFunction(10, rpl::Gpio::AltFunction::kAlt0);  // SPI0_MOSI
  rpl::Gpio::SetAltFunction(11, rpl::Gpio::AltFunction::kAlt0);  // SPI0_SCLK

  // Configure SPI settings
  spi->SetClockPhase(rpl::Spi::ClockPhase::kBeginning);
  spi->SetClockPolarity(rpl::Spi::ClockPolarity::kLow);
  spi->SetClockDivider(128);  // Adjust based on your needs
  spi->SetCs0Polarity(rpl::Spi::CsPolarity::kLow);

  // Create PMW3389 instance
  pmw3389_rpl4_lib::PMW3389 sensor(spi, 0);

  // Initialize sensor
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

## API Reference

### Initialization
- `PMW3389(spi, cs_num, cs_gpio)` - Constructor
- `Init()` - Initialize the sensor
- `PowerUpReset()` - Power up and reset the sensor

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

## License
[Add your license information here]
