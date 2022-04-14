#include "sensor.h"

void initialize_mpu() {
  while (!Serial)
    delay(10); 
  Serial.println("Adafruit MPU6050 test!");

  // try to init
  if (!mpu.begin()) {
    Serial.println("Searching for MPU6050");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 found!");
}
