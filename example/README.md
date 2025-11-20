# PMW3389 Example - Raspberry Pi 4 with RPL4

This example demonstrates how to use the pmw3389-rpl4-lib library with Raspberry Pi 4 using the RPL4 peripheral library.

## Files

- `peripheral_rpl4.cc` - Implementation of the peripheral interface for RPL4
- `main.cc` - Example application showing sensor usage

## Peripheral Implementation

The `peripheral_rpl4.cc` file implements the hardware abstraction layer defined in `peripheral_interface.h`:

```cpp
bool PeripheralInit()              // Initialize SPI and GPIO using RPL4
void PeripheralEnableCs()          // Set CS pin low
void PeripheralDisableCs()         // Set CS pin high
bool PeripheralSpiTransmitReceive() // SPI transfer using RPL4
void PeripheralDelayMicroseconds()  // Delay using std::this_thread
```

## Building

From the repository root:

```bash
./build.sh
```

This will:
1. Clone RPL4 if not present
2. Build the pmw3389-rpl4-lib library
3. Build this example with the RPL4 peripheral implementation

## Running

```bash
./.build/example/pmw3389_example
```

Note: Requires a Raspberry Pi 4 with a PMW3389 sensor connected to SPI0.

## Hardware Connections

- GPIO 8: CS (Chip Select)
- GPIO 9: MISO
- GPIO 10: MOSI
- GPIO 11: SCLK

## Adapting for Other Platforms

To use this library on other platforms:

1. Copy `peripheral_rpl4.cc` as a template
2. Replace RPL4 calls with your platform's HAL/library calls
3. Link your implementation with the pmw3389-rpl4-lib library
4. Use the same `main.cc` structure (only peripheral code changes)

See the main README.md for STM32 and Arduino examples.
