#include <Arduino.h>
#include <sensor.h>
#include <stepper.h>

// pin defines
const int LED = 2;
const int x_en = 5;
const int x_step = 0;
const int x_dir = 4;
const int y_step = 26;
const int y_dir = 25;

// motor parameters
const uint8_t RUN_CURRENT_PERCENT = 100;


const long SERIAL_BAUD_RATE = 115200;

HardwareSerial & x_motor_serial = Serial1;
// HardwareSerial & y_motor_serial = Serial2;

// create motors and sensors
stepper_driver x_stepper;
// mpu_driver mpu;

void setup() {
  // begin serial comms 
  Serial.begin(SERIAL_BAUD_RATE);

  // enable pins
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(x_en, OUTPUT);
  pinMode(x_step, OUTPUT);
  pinMode(x_dir, OUTPUT);

  // connect to pins
  x_stepper.stepper.connectToPins(x_step, x_dir);

  // set motor parameters
  x_stepper.set_ustep(16);
  x_stepper.set_run_current(RUN_CURRENT_PERCENT);
  x_stepper.set_hold_current(RUN_CURRENT_PERCENT);
  x_stepper.stepper.setSpeedInStepsPerSecond(600);
  x_stepper.stepper.setAccelerationInStepsPerSecondPerSecond(1000);
  x_stepper.stepper.setDecelerationInStepsPerSecondPerSecond(1000);

  x_stepper.enable();

  // bool temp = x_stepper.test_setup();
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
    delay(4000);
    x_stepper.stepper.setTargetPositionRelativeInSteps(5000);
    Serial.printf("Moving stepper by %d steps\n", 5000);
  }

}
