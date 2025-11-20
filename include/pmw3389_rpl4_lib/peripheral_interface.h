#ifndef PMW3389_RPL4_LIB_PERIPHERAL_INTERFACE_H
#define PMW3389_RPL4_LIB_PERIPHERAL_INTERFACE_H

#include <cstddef>
#include <cstdint>

namespace pmw3389_rpl4_lib {

/**
 * @brief Hardware abstraction interface for PMW3389 sensor communication
 *
 * This interface defines the hardware-dependent operations required to
 * communicate with the PMW3389 sensor. Users can implement these functions
 * to support different hardware platforms (e.g., Raspberry Pi, STM32, Arduino).
 *
 * Default weak implementations are provided that do nothing, allowing the
 * library to compile without any peripheral implementation.
 */

/**
 * @brief Initialize the peripheral hardware (SPI, GPIO, etc.)
 *
 * This function should configure SPI and GPIO peripherals for communication
 * with the PMW3389 sensor.
 *
 * @return true if initialization is successful, false otherwise
 */
bool PeripheralInit();

/**
 * @brief Enable chip select (CS) for SPI communication
 *
 * This function should set the CS pin to active state (typically low).
 */
void PeripheralEnableCs();

/**
 * @brief Disable chip select (CS) for SPI communication
 *
 * This function should set the CS pin to inactive state (typically high).
 */
void PeripheralDisableCs();

/**
 * @brief Perform SPI transmit and receive operation
 *
 * @param tx_data Pointer to transmit data buffer
 * @param rx_data Pointer to receive data buffer
 * @param length Number of bytes to transmit and receive
 * @return true if operation is successful, false otherwise
 */
bool PeripheralSpiTransmitReceive(const uint8_t* tx_data, uint8_t* rx_data,
                                  size_t length);

/**
 * @brief Delay for a specified number of microseconds
 *
 * @param microseconds Number of microseconds to delay
 */
void PeripheralDelayMicroseconds(uint32_t microseconds);

}  // namespace pmw3389_rpl4_lib

#endif  // PMW3389_RPL4_LIB_PERIPHERAL_INTERFACE_H
