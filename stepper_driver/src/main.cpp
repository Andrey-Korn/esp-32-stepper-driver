#include <Arduino.h>
#include <sensor.h>
#include <stepper.h>

// pin defines
const int LED = 2;  // onboard ESP32 LED on GPIO2

const int x_en = 5; // x_stepper pins
const int x_step = 0;
const int x_dir = 4;

const int y_en = 13;  // y_stepper pins
const int y_step = 26;
const int y_dir = 25;

// motor parameters
const uint8_t RUN_CURRENT_PERCENT = 100;
const int u_step = 8;
const long steps_per_revolution = 200;
const float max_speed = 800;
const float accel = 1000;


const long SERIAL_BAUD_RATE = 115200;

// Hardware serial for use with tmc2209 boards
HardwareSerial & x_motor_serial = Serial1;
HardwareSerial & y_motor_serial = Serial2;

// create motors and sensors
stepper_driver x_stepper;
stepper_driver y_stepper;
mpu_driver mpu;

// accelerometer vars
int16_t ax, ay, az;
int16_t gx, gy, gz;


void setup() {
  // begin serial comms 
  Serial.begin(SERIAL_BAUD_RATE);

  x_stepper.tmc.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_0);
  while(!x_stepper.test_setup()){
      delay(500);
      Serial.println("waiting for x tmc2209 connection!");
  }

  y_stepper.tmc.setup(Serial2, 115200, TMC2209::SERIAL_ADDRESS_0);
  while(!y_stepper.test_setup()){
      delay(500);
      Serial.println("waiting for y tmc2209 connection!");
  }


  // enable pins
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(x_en, OUTPUT);
  pinMode(x_step, OUTPUT);
  pinMode(x_dir, OUTPUT);

  pinMode(y_en, OUTPUT);
  pinMode(y_step, OUTPUT);
  pinMode(y_dir, OUTPUT);

  // connect to pins
  x_stepper.connect_motor_pins(x_step, x_dir, x_en);
  y_stepper.connect_motor_pins(y_step, y_dir, y_en);


  // set motor parameters
  x_stepper.set_ustep(u_step);
  x_stepper.stepper.setStepsPerRevolution(steps_per_revolution * u_step);
  x_stepper.set_run_current(RUN_CURRENT_PERCENT);
  x_stepper.set_hold_current(RUN_CURRENT_PERCENT);
  x_stepper.stepper.setSpeedInStepsPerSecond(max_speed);
  x_stepper.stepper.setAccelerationInStepsPerSecondPerSecond(accel);
  x_stepper.stepper.setDecelerationInStepsPerSecondPerSecond(accel);

  y_stepper.set_ustep(u_step);
  y_stepper.stepper.setStepsPerRevolution(steps_per_revolution * u_step);
  y_stepper.set_run_current(RUN_CURRENT_PERCENT);
  y_stepper.set_hold_current(RUN_CURRENT_PERCENT);
  y_stepper.stepper.setSpeedInStepsPerSecond(max_speed);
  y_stepper.stepper.setAccelerationInStepsPerSecondPerSecond(accel);
  y_stepper.stepper.setDecelerationInStepsPerSecondPerSecond(accel);

  

  // check motor setups
  // bool temp = x_stepper.test_setup();
  // x_stepper.test_connection();
  // x_stepper.print_parameters();

  // bool temp = y_stepper.test_setup();
  // y_stepper.test_connection();
  // y_stepper.print_parameters();

  mpu.Initialize();

  // for (int i = mpu.iAx; i <= mpu.iGx; i++){
  //     mpu.Target[i] = 0; // must fix for ZAccel
  //     mpu.HighOffset[i] = 0;
  //     mpu.LowOffset[i] = 0;
  // } // set targets and initial guesses
  // mpu.Target[mpu.iAz] = 16384;

  // mpu.PullBracketsOut();
  // mpu.PullBracketsIn();

  Serial.println("-------------- done --------------");

  // start flexy stepper services on core 0
  // arduino stack runs on core 1
  x_stepper.stepper.startAsService(0);
  y_stepper.stepper.startAsService(0);
  x_stepper.enable();
  y_stepper.enable();
}


void loop() {
  // toggle onboard LED
  // digitalWrite(LED, !digitalRead(LED));

  /* Get new sensor events with the readings */
  
  // wait
  // delay(500);

  // step motors example
  if (x_stepper.stepper.getDistanceToTargetSigned() == 0) {
    x_stepper.stepper.setTargetPositionRelativeInRevolutions(1);
    Serial.printf("Moving x stepper by %ld steps\n", u_step * steps_per_revolution);
  }
  if (y_stepper.stepper.getDistanceToTargetSigned() == 0) {
    y_stepper.stepper.setTargetPositionRelativeInRevolutions(1);
    Serial.printf("Moving y stepper by %ld steps\n", u_step * steps_per_revolution);
  }

  // mpu.accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Serial.print(ax);
  // Serial.print("\t");
  // Serial.print(ay);
  // Serial.print("\t");
  // Serial.print(az);
  // Serial.print("\t");
  // Serial.print(gx);
  // Serial.print("\t");
  // Serial.print(gy);
  // Serial.print("\t");
  // Serial.println(gz);
}
