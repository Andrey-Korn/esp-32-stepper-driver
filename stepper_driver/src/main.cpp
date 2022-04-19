#include <Arduino.h>
#include <sensor.h>
#include <stepper.h>

// pin defines

const int LED = 2;
const int x_step = 0;
const int x_dir = 4;
const int y_step = 26;
const int y_dir = 25;


const long SERIAL_BAUD_RATE = 115200;

HardwareSerial & x_motor_serial = Serial1;
// HardwareSerial & y_motor_serial = Serial2;

// create motors and sensors
stepper_driver x_stepper;
// mpu_driver mpu;

void setup() {
  // begin serial and set pins
  Serial.begin(SERIAL_BAUD_RATE);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  x_stepper.enable();
  // connect to pins
  x_stepper.stepper.connectToPins(x_step, x_dir);

  // set speed limits
  x_stepper.stepper.setSpeedInStepsPerSecond(300);
  x_stepper.stepper.setAccelerationInStepsPerSecondPerSecond(800);
  x_stepper.stepper.setDecelerationInStepsPerSecondPerSecond(800);

  x_stepper.enable();

  x_stepper.test_connection();
  x_stepper.print_parameters();

  // start flexy stepper services on core 0
  // arduino stack runs on core 1
  x_stepper.stepper.startAsService(0);
}


void loop() {
  // toggle onboard LED
  // digitalWrite(LED, !digitalRead(LED));

  /* Get new sensor events with the readings */
  
  // wait
  // delay(500);

  // step motors example
  if (x_stepper.stepper.getDistanceToTargetSigned() == 0) {
    x_stepper.disable();
    delay(4000);
    x_stepper.enable();
    x_stepper.stepper.setTargetPositionRelativeInSteps(500);
    Serial.printf("Moving stepper by %d steps\n", 500);
  }

}
