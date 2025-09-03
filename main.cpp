#include <Arduino.h>
#include "LSM6DSV80X_SPI.h"

// ESP32-S3 pins (your wiring)
static constexpr int PIN_CS   = 10;
static constexpr int PIN_MOSI = 11;
static constexpr int PIN_SCK  = 12;
static constexpr int PIN_MISO = 13;

LSM6DSV80X_SPI imu(PIN_CS);

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\nLSM6DSV80X SPI Test");

  // Bring up SPI + IMU
  imu.begin(PIN_SCK, PIN_MISO, PIN_MOSI, 1000000UL);

  // Accel: ±2 g @104 Hz
  imu.setAccel(LSM6DSV80X_SPI::ODR_104, LSM6DSV80X_SPI::AFS_2G);

  // Gyro: ±2000 dps @104 Hz
  imu.setGyro (LSM6DSV80X_SPI::ODR_104, LSM6DSV80X_SPI::GFS_2000);

  Serial.println("Accel + Gyro configured. Streaming...");
}

void loop() {
  bool ar=false, gr=false;
  imu.dataReady(ar, gr);

  if (ar) {
    LSM6DSV80X_SPI::Vec3f a;
    imu.readAccel(a);
    Serial.printf("ACC [g]: %+0.3f  %+0.3f  %+0.3f\n", a.x,a.y,a.z);
  }

  if (gr) {
    LSM6DSV80X_SPI::Vec3f g;
    imu.readGyro(g);
    Serial.printf("GYR [dps]: %+0.2f  %+0.2f  %+0.2f\n", g.x,g.y,g.z);
  }

  delay(50);
}
