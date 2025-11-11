#include "pmw3389_rpl4_lib/pmw3389.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "pmw3389_rpl4_lib/registers.h"
#include "rpl4/peripheral/gpio.hpp"
#include "rpl4/peripheral/spi.hpp"

namespace pmw3389_rpl4_lib {

PMW3389::PMW3389(std::shared_ptr<rpl::SpiBase> spi, uint8_t cs_num,
                 std::shared_ptr<rpl::Gpio> cs_gpio)
    : spi_(spi), cs_num_(cs_num), cs_gpio_(cs_gpio) {}

bool PMW3389::Init() {
  if (!cs_gpio_) {
    std::cerr << "CS GPIO is not provided!" << std::endl;
    return false;
  }

  cs_gpio_->SetAltFunction(rpl::Gpio::AltFunction::kOutput);
  cs_gpio_->SetPullRegister(rpl::Gpio::PullRegister::kNoRegister);
  cs_gpio_->Write(true);

  // Power up reset
  PowerUpReset();

  return true;
}

void PMW3389::PowerUpReset() {
  // Send power up reset
  WriteRegister(Register::Power_Up_Reset, 0x5A);
  DelayMicroseconds(50000);  // Wait 50ms for sensor to boot

  // Read from registers 0x02, 0x03, 0x04, 0x05 and 0x06 one time
  uint8_t dummy;
  ReadRegister(Register::Motion, &dummy);
  ReadRegister(Register::Delta_X_L, &dummy);
  ReadRegister(Register::Delta_X_H, &dummy);
  ReadRegister(Register::Delta_Y_L, &dummy);
  ReadRegister(Register::Delta_Y_H, &dummy);
}

bool PMW3389::WriteRegister(Register address, uint8_t data) {
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

  // Send address with MSB = 1 to indicate write
  tx_buf[0] = static_cast<uint8_t>(address) | 0x80;
  tx_buf[1] = data;

  EnableCs();
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, 2);
  DelayMicroseconds(20);  // tSCLK-NCS for write operation
  DisableCs();
  DelayMicroseconds(100);  // tSWW/tSWR (=120us) minus tSCLK-NCS

  return true;
}

bool PMW3389::ReadRegister(Register address, uint8_t* data) {
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

  // Send address with MSB = 0 to indicate read
  tx_buf[0] = static_cast<uint8_t>(address) & 0x7F;
  tx_buf[1] = 0;

  EnableCs();
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, 1);
  DelayMicroseconds(180);  // tSRAD - delay before reading data
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, 1);
  *data = rx_buf[0];

  DelayMicroseconds(1);  // tSCLK-NCS for read operation is 120ns
  DisableCs();
  DelayMicroseconds(19);  // tSRW/tSRR (=20us) minus tSCLK-NCS

  return true;
}

uint8_t PMW3389::ReadProductId() {
  uint8_t product_id;
  ReadRegister(Register::Product_ID, &product_id);
  return product_id;
}

uint8_t PMW3389::ReadInverseProductId() {
  uint8_t inverse_product_id;
  ReadRegister(Register::Inverse_Product_ID, &inverse_product_id);
  return inverse_product_id;
}

uint8_t PMW3389::ReadRevisionId() {
  uint8_t revision_id;
  ReadRegister(Register::Revision_ID, &revision_id);
  return revision_id;
}

uint8_t PMW3389::ReadSromId() {
  uint8_t srom_id;
  ReadRegister(Register::SROM_ID, &srom_id);
  return srom_id;
}

bool PMW3389::ReadMotion(int16_t* delta_x, int16_t* delta_y) {
  WriteRegister(Register::Motion, 0x20);
  uint8_t motion;
  ReadRegister(Register::Motion, &motion);

  // Check if motion is detected
  if (!(motion & 0x80)) {
    *delta_x = 0;
    *delta_y = 0;
    return true;
  }

  uint8_t delta_x_l, delta_x_h, delta_y_l, delta_y_h;
  ReadRegister(Register::Delta_X_L, &delta_x_l);
  ReadRegister(Register::Delta_X_H, &delta_x_h);
  ReadRegister(Register::Delta_Y_L, &delta_y_l);
  ReadRegister(Register::Delta_Y_H, &delta_y_h);

  *delta_x = static_cast<int16_t>((delta_x_h << 8) | delta_x_l);
  *delta_y = static_cast<int16_t>((delta_y_h << 8) | delta_y_l);

  return true;
}

bool PMW3389::ReadMotionBurst(MotionBurstData* data) {
  uint8_t tx_buf[12];
  uint8_t rx_buf[12];

  // Send Motion_Burst address
  WriteRegister(Register::Motion_Burst, 0x00);

  EnableCs();
  tx_buf[0] = static_cast<uint8_t>(Register::Motion_Burst);
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, 1);
  DelayMicroseconds(35);  // tSRAD
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, 12);

  // Read 12 bytes of burst data
  DisableCs();

  // Parse the burst data
  data->motion = rx_buf[0];
  data->observation = rx_buf[1];
  data->delta_x = static_cast<int16_t>((rx_buf[3] << 8) | rx_buf[2]);
  data->delta_y = static_cast<int16_t>((rx_buf[5] << 8) | rx_buf[4]);
  data->squal = rx_buf[6];
  data->raw_data_sum = rx_buf[7];
  data->maximum_raw_data = rx_buf[8];
  data->minimum_raw_data = rx_buf[9];
  data->shutter = static_cast<uint16_t>((rx_buf[11] << 8) | rx_buf[10]);

  DelayMicroseconds(1);  // tSCLK-NCS

  return true;
}

bool PMW3389::SetCPI(uint16_t cpi) {
  // CPI = (Resolution + 1) * 50
  // Resolution = (CPI / 50) - 1
  if (cpi < 50 || cpi > 16000) { return false; }

  uint16_t resolution = (cpi / 50) - 1;
  uint8_t resolution_l = resolution & 0xFF;
  uint8_t resolution_h = (resolution >> 8) & 0xFF;

  WriteRegister(Register::Resolution_L, resolution_l);
  WriteRegister(Register::Resolution_H, resolution_h);

  return true;
}

uint16_t PMW3389::GetCPI() {
  uint8_t resolution_l, resolution_h;
  ReadRegister(Register::Resolution_L, &resolution_l);
  ReadRegister(Register::Resolution_H, &resolution_h);

  uint16_t resolution = (resolution_h << 8) | resolution_l;
  return (resolution + 1) * 50;
}

bool PMW3389::UploadFirmware(const uint8_t* firmware_data, size_t length) {
  // Write 0x1D to Config2 register for wireless mouse design
  WriteRegister(Register::Config2, 0x00);

  // Write 0x18 to SROM_Enable register to initialize
  WriteRegister(Register::SROM_Enable, 0x1D);

  // Wait for 10ms
  DelayMicroseconds(10000);

  // Write 0x18 to SROM_Enable register again to start SROM download
  WriteRegister(Register::SROM_Enable, 0x18);
  DelayMicroseconds(120);

  // Write SROM_Load_Burst register
  uint8_t tx_buf[1];
  uint8_t rx_buf[1];

  EnableCs();

  tx_buf[0] = static_cast<uint8_t>(Register::SROM_Load_Burst) | 0x80;
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, 1);
  DelayMicroseconds(15);
  for (size_t i = 0; i < length; i++) {
    tx_buf[0] = firmware_data[i];
    spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, 1);
    DelayMicroseconds(15);
  }
  DelayMicroseconds(20);
  DisableCs();
  DelayMicroseconds(200);  // Wait for SROM download to complete

  // Read SROM_ID to verify
  uint8_t srom_id = ReadSromId();
  std::cout << "SROM ID after upload: 0x" << std::hex
            << static_cast<int>(srom_id) << std::dec << std::endl;

  WriteRegister(Register::Config2, 0x00);

  return true;
}

bool PMW3389::RunSROMSelfTest() {
  WriteRegister(Register::SROM_Enable, 0x15);
  DelayMicroseconds(10000);
  uint8_t read_buf[2];
  ReadRegister(Register::Data_Out_Upper, &read_buf[0]);
  ReadRegister(Register::Data_Out_Lower, &read_buf[1]);
  uint16_t result = (read_buf[0] << 8) | read_buf[1];
  std::cout << "result of SROM self-test: 0x" << std::hex << result << std::dec << std::endl;

  bool status = result == 0xBEEF;
  if (status) {
    std::cout << "SROM self-test passed!" << std::endl;
  } else {
    std::cout << "SROM self-test failed!" << std::endl;
  }

  return status;
}

void PMW3389::SetRunMode() {
  // Set Run mode by writing to Run_Downshift register
  WriteRegister(Register::Run_Downshift, 0x00);
}

void PMW3389::SetRest1Mode() {
  // Configure Rest1 mode
  WriteRegister(Register::Run_Downshift, 0xFF);  // Fast transition to rest
  WriteRegister(Register::Rest1_Rate_Lower, 0x00);
  WriteRegister(Register::Rest1_Rate_Upper, 0x00);
}

void PMW3389::SetRest2Mode() {
  // Configure Rest2 mode
  WriteRegister(Register::Rest1_Downshift, 0x00);  // Skip Rest1
  WriteRegister(Register::Rest2_Rate_Lower, 0x63);
  WriteRegister(Register::Rest2_Rate_Upper, 0x00);
}

void PMW3389::SetRest3Mode() {
  // Configure Rest3 mode
  WriteRegister(Register::Rest2_Downshift, 0x00);  // Skip Rest2
  WriteRegister(Register::Rest3_Rate_Lower, 0xF3);
  WriteRegister(Register::Rest3_Rate_Upper, 0x01);
}

void PMW3389::EnableCs() {
  spi_->SetChipSelectForCommunication(cs_num_);
  if (cs_gpio_) { cs_gpio_->Write(false); }
}

void PMW3389::DisableCs() {
  if (cs_gpio_) { cs_gpio_->Write(true); }
}

void PMW3389::DelayMicroseconds(uint32_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

}  // namespace pmw3389_rpl4_lib
