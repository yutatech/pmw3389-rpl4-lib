#include "pmw3389_rpl4_lib/peripheral_interface.h"

namespace pmw3389_rpl4_lib {

// Weak default implementations that do nothing
// Users should override these with their own implementations

__attribute__((weak)) bool PeripheralInit() { return false; }

__attribute__((weak)) void PeripheralEnableCs() {}

__attribute__((weak)) void PeripheralDisableCs() {}

__attribute__((weak)) bool PeripheralSpiTransmitReceive(const uint8_t* tx_data,
                                                        uint8_t* rx_data,
                                                        size_t length) {
  (void)tx_data;
  (void)rx_data;
  (void)length;
  return false;
}

__attribute__((weak)) void PeripheralDelayMicroseconds(uint32_t microseconds) {
  (void)microseconds;
}

}  // namespace pmw3389_rpl4_lib
