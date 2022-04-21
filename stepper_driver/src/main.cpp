#include <Arduino.h>
#include <sensor.h>
#include <stepper.h>

// #include <MPU6050_6Axis_MotionApps_V6_12.h>

// pin defines
const int LED = 2;  // onboard ESP32 LED on GPIO2

const int x_en = 5; // x_stepper pins
const int x_step = 0;
const int x_dir = 4;

const int y_en = 13;  // y_stepper pins
const int y_step = 26;
const int y_dir = 25;

const int mpu_int_pin = 23; // mpu6050 DMP interrupt

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
int16_t rx, ry, rz;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/* create a hardware timer */
hw_timer_t * timer = NULL;

// DMP interrupt routine
volatile bool mpu_interrupt = false;
void dmp_data_ready() {
  mpu_interrupt = true;
}

void mpu_callback() {
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

  pinMode(mpu_int_pin, INPUT);

  pinMode(x_en, OUTPUT);
  pinMode(x_step, OUTPUT);
  pinMode(x_dir, OUTPUT);

  pinMode(y_en, OUTPUT);
  pinMode(y_step, OUTPUT);
  pinMode(y_dir, OUTPUT);

  // connect to motor pins
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

  // setup mpu6050 
  mpu.Initialize();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, mpu_callback, true);
  timerAlarmWrite(timer, 100000, true);
  timerAlarmEnable(timer);
  // devStatus = mpu.accelgyro.dmpInitialize();
  mpu.accelgyro.setXAccelOffset(-1422);
  mpu.accelgyro.setYAccelOffset(243);
  mpu.accelgyro.setZAccelOffset(5626);
  mpu.accelgyro.setXGyroOffset(16);
  mpu.accelgyro.setYGyroOffset(126);
  mpu.accelgyro.setZGyroOffset(98);
  // mpu.accelgyro.setDMPEnabled(true);

  // attach mpu6050 DMP pin to intterupt
  attachInterrupt(digitalPinToInterrupt(mpu_int_pin), dmp_data_ready, RISING);
  mpuIntStatus = mpu.accelgyro.getIntStatus();
  // set DMP ready flag so main loop() can use it
  dmpReady = true;

  packetSize = mpu.accelgyro.dmpGetFIFOPacketSize();

  // start flexy stepper services on core 0
  // arduino stack runs on core 1
  x_stepper.stepper.startAsService(0);
  y_stepper.stepper.startAsService(0);

  // enable motors
  x_stepper.enable();
  y_stepper.enable();

  Serial.println("-------------- ready --------------");
}


void loop() {
  // toggle onboard LED
  digitalWrite(LED, !digitalRead(LED));

  /* Get new sensor events with the readings */
  
  // step motors example
  if (x_stepper.stepper.getDistanceToTargetSigned() == 0) {
    x_stepper.stepper.setTargetPositionRelativeInRevolutions(1);
    Serial.printf("Moving x stepper by %ld steps\n", u_step * steps_per_revolution);
  }
  if (y_stepper.stepper.getDistanceToTargetSigned() == 0) {
    y_stepper.stepper.setTargetPositionRelativeInRevolutions(1);
    Serial.printf("Moving y stepper by %ld steps\n", u_step * steps_per_revolution);
  }

  // if mpu DMP packet ready
  // if (mpu.accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) {
  //   mpu.accelgyro.dmpGetQuaternion(&q, fifoBuffer);
  // //   mpu.accelgyro.dmpGetGravity(&gravity, &q);
  // //   mpu.accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
  // //   // Serial.print("ypr\t");
  // //   // Serial.print(ypr[0] * 180 / PI);
  // //   // Serial.print("\t");
  // //   // Serial.print(ypr[1] * 180 / PI);
  // //   // Serial.print("\t");
  // //   // Serial.println(ypr[2] * 180 / PI);

  //   mpu.accelgyro.dmpGetEuler(euler, &q);
  //   Serial.print("euler\t");
  //   Serial.print(euler[0] * 180 / PI);
  //   Serial.print("\t");
  //   Serial.print(euler[1] * 180 / PI);
  //   Serial.print("\t");
  //   Serial.println(euler[2] * 180 / PI);

  // }

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
