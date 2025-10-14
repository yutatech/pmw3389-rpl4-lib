#ifndef PMW3389_RPL4_LIB_REGISTERS_H
#define PMW3389_RPL4_LIB_REGISTERS_H

#include <cstdint>

namespace pmw3389_rpl4_lib {

enum class Register : uint8_t {
  Product_ID = 0x00,
  Revision_ID = 0x01,
  Motion = 0x02,
  Delta_X_L = 0x03,
  Delta_X_H = 0x04,
  Delta_Y_L = 0x05,
  Delta_Y_H = 0x06,
  SQUAL = 0x07,
  RawData_Sum = 0x08,
  Maximum_RawData = 0x09,
  Minimum_RawData = 0x0A,
  Shutter_Lower = 0x0B,
  Shutter_Upper = 0x0C,
  Ripple_Control = 0x0D,
  Resolution_L = 0x0E,
  Resolution_H = 0x0F,
  Config2 = 0x10,
  Angle_Tune = 0x11,
  Frame_Capture = 0x12,
  SROM_Enable = 0x13,
  Run_Downshift = 0x14,
  Rest1_Rate_Lower = 0x15,
  Rest1_Rate_Upper = 0x16,
  Rest1_Downshift = 0x17,
  Rest2_Rate_Lower = 0x18,
  Rest2_Rate_Upper = 0x19,
  Rest2_Downshift = 0x1A,
  Rest3_Rate_Lower = 0x1B,
  Rest3_Rate_Upper = 0x1C,
  Observation = 0x24,
  Data_Out_Lower = 0x25,
  Data_Out_Upper = 0x26,
  SROM_ID = 0x2A,
  Min_SQ_Run = 0x2B,
  RawData_Threshold = 0x2C,
  Control2 = 0x2D,
  Config5_L = 0x2E,
  Config5_H = 0x2F,
  Power_Up_Reset = 0x3A,
  Shutdown = 0x3B,
  Inverse_Product_ID = 0x3F,
  LiftCutoff_Cal3 = 0x41,
  Angle_Snap = 0x42,
  LiftCutoff_Cal1 = 0x4A,
  Motion_Burst = 0x50,
  SROM_Load_Burst = 0x62,
  Lift_Config = 0x63,
  RawData_Burst = 0x64,
  LiftCutoff_Cal2 = 0x65,
  LiftCutoff_Cal_Timeout = 0x71,
  LiftCutoff_Cal_Min_Length = 0x72,
  PWM_Period_Cnt = 0x73,
  PWM_Width_Cnt = 0x74
};

}  // namespace pmw3389_rpl4_lib

#endif  // PMW3389_RPL4_LIB_REGISTERS_H
