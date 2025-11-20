#ifndef PMW3389_RPL4_LIB_PMW3389_H
#define PMW3389_RPL4_LIB_PMW3389_H

#include <cstddef>
#include <cstdint>

#include "pmw3389_rpl4_lib/peripheral_interface.h"
#include "pmw3389_rpl4_lib/registers.h"

namespace pmw3389_rpl4_lib {

/**
 * @brief Burst read data structure for motion data
 */
struct MotionBurstData {
  uint8_t motion;
  uint8_t observation;
  int16_t delta_x;
  int16_t delta_y;
  uint8_t squal;
  uint8_t raw_data_sum;
  uint8_t maximum_raw_data;
  uint8_t minimum_raw_data;
  uint16_t shutter;
};

class PMW3389 {
 public:
  PMW3389();
  ~PMW3389() = default;

  /**
   * @brief Initializes the PMW3389 sensor.
   * @return true if initialization is successful, false otherwise.
   */
  bool Init();

  /**
   * @brief Power up and reset the sensor.
   */
  void PowerUpReset();

  /**
   * @brief Write data to one specific register.
   *
   * @param address The register address to write to.
   * @param data The data byte to write.
   * @return true if writing is successful, false if writing fails.
   */
  bool WriteRegister(Register address, uint8_t data);

  /**
   * @brief Read data from one specific register.
   *
   * @param address The register address to read from.
   * @param data Pointer to store the read data.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadRegister(Register address, uint8_t* data);

  /**
   * @brief Read the Product ID register.
   *
   * @return The value of the Product ID register.
   */
  uint8_t ReadProductId();

  /**
   * @brief Read the Inverse Product ID register.
   *
   * @return The value of the Inverse Product ID register.
   */
  uint8_t ReadInverseProductId();

  /**
   * @brief Read the Revision ID register.
   *
   * @return The value of the Revision ID register.
   */
  uint8_t ReadRevisionId();

  /**
   * @brief Read the SROM ID register.
   *
   * @return The value of the SROM ID register.
   */
  uint8_t ReadSromId();

  /**
   * @brief Read motion data (delta X and Y).
   *
   * @param delta_x Pointer to store the X-axis motion delta.
   * @param delta_y Pointer to store the Y-axis motion delta.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadMotion(int16_t* delta_x, int16_t* delta_y);

  /**
   * @brief Read motion burst data (optimized for reading all motion data at
   * once).
   *
   * @param data Pointer to structure to store the burst data.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadMotionBurst(MotionBurstData* data);

  /**
   * @brief Set the CPI (Counts Per Inch) resolution.
   *
   * @param cpi The desired CPI value (50-16000 in steps of 50).
   * @return true if setting is successful, false if setting fails.
   */
  bool SetCPI(uint16_t cpi);

  /**
   * @brief Get the current CPI (Counts Per Inch) resolution.
   *
   * @return The current CPI value.
   */
  uint16_t GetCPI();

  /**
   * @brief Upload firmware (SROM) to the sensor.
   *
   * @param firmware_data Pointer to the firmware data array.
   * @param length The length of the firmware data.
   * @return true if upload is successful, false if upload fails.
   */
  bool UploadFirmware(const uint8_t* firmware_data, size_t length);

  bool RunSROMSelfTest();

  /**
   * @brief Set the sensor to run mode (normal operation).
   */
  void SetRunMode();

  /**
   * @brief Set the sensor to rest1 mode (low power).
   */
  void SetRest1Mode();

  /**
   * @brief Set the sensor to rest2 mode (lower power).
   */
  void SetRest2Mode();

  /**
   * @brief Set the sensor to rest3 mode (lowest power).
   */
  void SetRest3Mode();

 private:
  void EnableCs();
  void DisableCs();
  void DelayMicroseconds(uint32_t us);
};

}  // namespace pmw3389_rpl4_lib

#endif  // PMW3389_RPL4_LIB_PMW3389_H
