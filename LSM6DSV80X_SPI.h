#pragma once
#include <Arduino.h>
#include <SPI.h>

class LSM6DSV80X_SPI {
public:
  static constexpr uint8_t WHOAMI_EXPECTED = 0x73;

  // Registers (subset)
  enum : uint8_t {
    REG_WHO_AM_I         = 0x0F,
    REG_CTRL1            = 0x10,
    REG_CTRL2            = 0x11,
    REG_CTRL3            = 0x12,
    REG_STATUS           = 0x1E,
    REG_OUTX_L_G         = 0x22,
    REG_OUTX_L_A         = 0x28,
    REG_OUTY_L_A         = 0x2A,
    REG_OUTZ_L_A         = 0x2C,
    REG_FUNCTIONS_ENABLE = 0x50,
    REG_HAODR_CFG        = 0x62
  };

  // ODR codes (used as (odr<<4))
  enum ODR { ODR_OFF=0x00, ODR_104=0x04, ODR_208=0x05, ODR_416=0x06 };

  // Full-scale codes (bits [3:2])
  enum AccelFS { AFS_2G=0, AFS_16G=1, AFS_4G=2, AFS_8G=3 };
  enum GyroFS  { GFS_250=0, GFS_500=1, GFS_1000=2, GFS_2000=3 };

  struct Vec3f { float x, y, z; };

  LSM6DSV80X_SPI(uint8_t csPin, SPIClass& spi = SPI) : _cs(csPin), _spi(&spi) {}

  // Never fails due to WHO_AM_I; logs it and proceeds.
  bool begin(int sck, int miso, int mosi, uint32_t clkHz = 1000000UL);

  bool whoAmI(uint8_t& v);

  // Uses OpMode = 0b001 (HAODR) internally
  bool setAccel(ODR odr, AccelFS fs);
  bool setGyro (ODR odr, GyroFS fs);

  bool dataReady(bool& accReady, bool& gyrReady);
  bool readAccel(Vec3f& g);
  bool readGyro(Vec3f& dps);

  // Debug helpers
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t &val);

private:
  uint8_t _cs;
  SPIClass* _spi;
  SPISettings _set;

  inline void csHigh() { digitalWrite(_cs, HIGH); }
  inline void csLow()  { digitalWrite(_cs, LOW);  }

  uint8_t r8(uint8_t reg);     // read single byte
  void    w8(uint8_t reg, uint8_t val);
  int16_t r16(uint8_t reg_low);

  float accelLSB_g(AccelFS fs);
  float gyroLSB_dps(GyroFS fs);

  AccelFS _afs = AFS_2G;
  GyroFS  _gfs = GFS_2000;
};
