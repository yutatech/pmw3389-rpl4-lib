#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "pmw3389_rpl4_lib/pmw3389.h"
#include "rpl4/peripheral/gpio.hpp"
#include "rpl4/peripheral/spi.hpp"
#include "rpl4/rpl4.hpp"

int main() {
  std::cout << "PMW3389 Sensor Example" << std::endl;
  std::cout << "======================" << std::endl;

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
  std::cout << "Initializing sensor..." << std::endl;
  if (!sensor.Init()) {
    std::cerr << "Failed to initialize sensor" << std::endl;
    return 1;
  }
  std::cout << "Sensor initialized successfully!" << std::endl;

  // Read and display product information
  uint8_t product_id = sensor.ReadProductId();
  uint8_t revision_id = sensor.ReadRevisionId();
  std::cout << "Product ID: 0x" << std::hex << static_cast<int>(product_id)
            << std::dec << std::endl;
  std::cout << "Revision ID: 0x" << std::hex << static_cast<int>(revision_id)
            << std::dec << std::endl;

  // Set CPI to 1600
  std::cout << "Setting CPI to 1600..." << std::endl;
  if (sensor.SetCPI(1600)) {
    uint16_t cpi = sensor.GetCPI();
    std::cout << "CPI set to: " << cpi << std::endl;
  }

  // Read motion data for 10 seconds
  std::cout << "\nReading motion data for 10 seconds..." << std::endl;
  std::cout << "(Move the sensor to see motion data)" << std::endl;

  auto start_time = std::chrono::steady_clock::now();
  int motion_count = 0;

  while (std::chrono::steady_clock::now() - start_time <
         std::chrono::seconds(10)) {
    int16_t delta_x, delta_y;
    if (sensor.ReadMotion(&delta_x, &delta_y)) {
      if (delta_x != 0 || delta_y != 0) {
        std::cout << "Motion detected - X: " << delta_x << ", Y: " << delta_y
                  << std::endl;
        motion_count++;
      }
    }

    // Sleep for a short time to avoid overwhelming the output
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "\nTotal motion events detected: " << motion_count << std::endl;

  // Demonstrate motion burst read
  std::cout << "\nReading motion burst data..." << std::endl;
  pmw3389_rpl4_lib::MotionBurstData burst_data;
  if (sensor.ReadMotionBurst(&burst_data)) {
    std::cout << "Burst read successful:" << std::endl;
    std::cout << "  Delta X: " << burst_data.delta_x << std::endl;
    std::cout << "  Delta Y: " << burst_data.delta_y << std::endl;
    std::cout << "  SQUAL: " << static_cast<int>(burst_data.squal) << std::endl;
    std::cout << "  Shutter: " << burst_data.shutter << std::endl;
  }

  std::cout << "\nExample completed successfully!" << std::endl;
  return 0;
}
