#include "LSM6DSV80X_SPI.h"

uint8_t LSM6DSV80X_SPI::r8(uint8_t reg) {
  csLow();
  _spi->beginTransaction(_set);
  _spi->transfer(0x80 | reg);     // bit7=1 read, full 8-bit address
  uint8_t v = _spi->transfer(0x00);
  _spi->endTransaction();
  csHigh();
  return v;
}

void LSM6DSV80X_SPI::w8(uint8_t reg, uint8_t val) {
  csLow();
  _spi->beginTransaction(_set);
  _spi->transfer(reg & 0x7F);     // bit7=0 write, full 8-bit address
  _spi->transfer(val);
  _spi->endTransaction();
  csHigh();
}

int16_t LSM6DSV80X_SPI::r16(uint8_t reg_low) {
  uint8_t lo = r8(reg_low);
  uint8_t hi = r8(reg_low+1);
  return (int16_t)((hi<<8) | lo);
}

// public API
bool LSM6DSV80X_SPI::begin(int sck, int miso, int mosi, uint32_t clkHz) {
  pinMode(_cs, OUTPUT);
  csHigh();
  _spi->begin(sck, miso, mosi, _cs);
  _set = SPISettings(clkHz, MSBFIRST, SPI_MODE3);

  uint8_t who=0; whoAmI(who);
  Serial.printf("WHO_AM_I=0x%02X (expected 0x73)\n", who);

  // Reset -> BDU=1, IF_INC=1
  w8(REG_CTRL3, 0x01); delay(20);
  w8(REG_CTRL3, 0x44);

  // Enable High-Accuracy ODR mode
  w8(REG_FUNCTIONS_ENABLE, 0x01); // HAODR_EN
  w8(REG_HAODR_CFG, 0x03);

  return true;
}

bool LSM6DSV80X_SPI::whoAmI(uint8_t& v) { v = r8(REG_WHO_AM_I); return true; }

bool LSM6DSV80X_SPI::setAccel(ODR odr, AccelFS fs) {
  _afs = fs;
  // (ODR<<4) | (FS<<2) | OpMode=001 (HAODR)
  uint8_t val = (uint8_t)((odr << 4) | (fs << 2) | 0x01);
  w8(REG_CTRL1, val);
  return true;
}

bool LSM6DSV80X_SPI::setGyro(ODR odr, GyroFS fs) {
  _gfs = fs;
  uint8_t val = (uint8_t)((odr << 4) | (fs << 2) | 0x01);
  w8(REG_CTRL2, val);
  return true;
}

bool LSM6DSV80X_SPI::dataReady(bool& accReady, bool& gyrReady) {
  uint8_t s = r8(REG_STATUS);
  accReady = s & 0x01;
  gyrReady = s & 0x02;
  return true;
}

bool LSM6DSV80X_SPI::readAccel(Vec3f& g) {
  int16_t x = r16(REG_OUTX_L_A);
  int16_t y = r16(REG_OUTY_L_A);
  int16_t z = r16(REG_OUTZ_L_A);
  float s = accelLSB_g(_afs);
  g.x = x*s; g.y = y*s; g.z = z*s;
  return true;
}

bool LSM6DSV80X_SPI::readGyro(Vec3f& dps) {
  int16_t x = r16(REG_OUTX_L_G);
  int16_t y = r16(REG_OUTX_L_G+2);
  int16_t z = r16(REG_OUTX_L_G+4);
  float s = gyroLSB_dps(_gfs);
  dps.x = x*s; dps.y = y*s; dps.z = z*s;
  return true;
}

bool LSM6DSV80X_SPI::writeReg(uint8_t reg, uint8_t val) { w8(reg,val); return true; }
bool LSM6DSV80X_SPI::readReg(uint8_t reg, uint8_t &val) { val = r8(reg); return true; }

// scales
float LSM6DSV80X_SPI::accelLSB_g(AccelFS fs) {
  switch (fs) {
    case AFS_2G:  return 0.000061f;  // 61 µg/LSB
    case AFS_4G:  return 0.000122f;
    case AFS_8G:  return 0.000244f;
    case AFS_16G: return 0.000488f;
  }
  return 0.000122f;
}

float LSM6DSV80X_SPI::gyroLSB_dps(GyroFS fs) {
  switch (fs) {
    case GFS_250:  return 0.00875f;
    case GFS_500:  return 0.0175f;
    case GFS_1000: return 0.035f;
    case GFS_2000: return 0.07f;
  }
  return 0.07f;
}
