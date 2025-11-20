#include "pmw3389_rpl4_lib/peripheral_interface.h"

#include <chrono>
#include <memory>
#include <thread>

#include "rpl4/peripheral/gpio.hpp"
#include "rpl4/peripheral/spi.hpp"
#include "rpl4/rpl4.hpp"

namespace {
// Static instances for RPL4 peripherals
std::shared_ptr<rpl::Spi> spi_instance;
std::shared_ptr<rpl::Gpio> cs_gpio;
uint8_t cs_num = 0;
}  // namespace

namespace pmw3389_rpl4_lib {

bool PeripheralInit() {
  // Initialize RPL4
  rpl::Init();

  // Get SPI instance
  spi_instance = rpl::Spi::GetInstance(rpl::Spi::Port::kSpi0);

  // Configure SPI GPIOs
  rpl::Gpio::SetAltFunction(9, rpl::Gpio::AltFunction::kAlt0);   // SPI0_MISO
  rpl::Gpio::SetAltFunction(10, rpl::Gpio::AltFunction::kAlt0);  // SPI0_MOSI
  rpl::Gpio::SetAltFunction(11, rpl::Gpio::AltFunction::kAlt0);  // SPI0_SCLK

  // Get CS GPIO instance
  cs_gpio = rpl::Gpio::GetInstance(8);  // GPIO pin 8 for CS

  // Configure SPI settings
  spi_instance->SetClockPhase(rpl::Spi::ClockPhase::kMiddle);
  spi_instance->SetClockPolarity(rpl::Spi::ClockPolarity::kHigh);
  spi_instance->SetClockDivider(64);  // Adjust based on your needs
  spi_instance->SetCs0Polarity(rpl::Spi::CsPolarity::kLow);
  spi_instance->SetReadEnable(rpl::Spi::ReadEnable::kDisable);

  // Configure CS GPIO
  cs_gpio->SetAltFunction(rpl::Gpio::AltFunction::kOutput);
  cs_gpio->SetPullRegister(rpl::Gpio::PullRegister::kNoRegister);
  cs_gpio->Write(true);  // CS is active low, so start high

  return true;
}

void PeripheralEnableCs() {
  if (spi_instance) {
    spi_instance->SetChipSelectForCommunication(cs_num);
  }
  if (cs_gpio) { cs_gpio->Write(false); }
}

void PeripheralDisableCs() {
  if (cs_gpio) { cs_gpio->Write(true); }
}

bool PeripheralSpiTransmitReceive(const uint8_t* tx_data, uint8_t* rx_data,
                                  size_t length) {
  if (!spi_instance) { return false; }

  // Copy to non-const buffer as RPL4 API requires non-const
  uint8_t* tx_buf = const_cast<uint8_t*>(tx_data);

  spi_instance->TransmitAndReceiveBlocking(tx_buf, rx_data, length);
  return true;
}

void PeripheralDelayMicroseconds(uint32_t microseconds) {
  std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

}  // namespace pmw3389_rpl4_lib
