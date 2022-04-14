#include <Arduino.h>
#include <Wire.h>
#include "sensor.h"

#define LED 2

Adafruit_MPU6050 mpu;

void setup() {
  // begin serial and set pins
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // test mpu6050
  initialize_mpu();
}


void loop() {
  // toggle onboard LED
  digitalWrite(LED, !digitalRead(LED));

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  
  // wait
  delay(500);
}
