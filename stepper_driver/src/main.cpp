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

const int mpu_int_pin = 23; // mpu6050 DMP interrupt

// motor parameters
const uint8_t RUN_CURRENT_PERCENT = 100;
const int u_step = 8;
const long steps_per_revolution = 200;
const float max_speed = 800;
const float accel = 2000;

// max table tilt of 2 degrees in any axis
const float x_max_angle = float(12) / 180;
const float y_max_angle = float(12) / 180;

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

// serial stream parameters and message buffers
const long SERIAL_BAUD_RATE = 115200;
const byte num_chars = 32;
char received_chars[num_chars];
char temp_chars[num_chars];

char message_from_pc[num_chars] = {0};
boolean new_data = false;
int data_number = 0;
float x_angle_curr = 0.0;
float x_angle_prev = 0.0;
float y_angle_curr = 0.0;
float y_angle_prev = 0.0;

// serial angle stream format: <x,y><x,y><x,y>...
// <-0.01,-0.02>
// packet char length = 9-13 (depending on negatives and decimal resolution)

// serial read functions
void recv_with_end_marker() {
  static boolean recv_in_progress = false;
  static byte ndx = 0;
  char start_marker = '<';
  char end_marker = '>';
  char rc;

  while (Serial.available() > 0 && new_data == false) {
    // grab character
    rc = Serial.read();

    if (recv_in_progress == true) {
      if (rc != end_marker) {
        if (rc != start_marker){
          received_chars[ndx] = rc;
          ndx++;
          if (ndx >= num_chars) {
            ndx = num_chars - 1;
          }
        }
      }

      else {
        received_chars[ndx] = '\0'; // terminate
        ndx = 0;
        // keep running if more available after '>'
        if (Serial.available() == 0) {
          new_data = true;
        }
        new_data = true;
      }
    }
    else if (rc == start_marker) {
      recv_in_progress = true;
    }
  }
}

void parse_angles() {
  char * strtok_idx;  // used by strtok() as index

  strtok_idx = strtok(temp_chars, ","); // grab first part of message

  x_angle_curr = atof(strtok_idx);

  strtok_idx = strtok(NULL, ",");
  y_angle_curr = atof(strtok_idx);
}

void print_angles() {
  Serial.print("X: ");
  Serial.print(x_angle_curr);
  // Serial.print(x_angle * x_max_angle);
  Serial.print(" Y: ");
  Serial.println(y_angle_curr);
  // Serial.println(y_angle * y_max_angle);
}

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
  // x_stepper.tmc.setup(Serial1, 115200);
  while(!x_stepper.test_setup()){
      delay(500);
      Serial.println("waiting for x tmc2209 connection!");
  }

  y_stepper.tmc.setup(Serial2, 115200, TMC2209::SERIAL_ADDRESS_0);
  // y_stepper.tmc.setup(Serial2, 115200);
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
  x_stepper.set_motor_parameters(u_step, steps_per_revolution, accel, max_speed, RUN_CURRENT_PERCENT);
  delay(0.5);
  y_stepper.set_motor_parameters(u_step, steps_per_revolution, accel, max_speed, RUN_CURRENT_PERCENT);

  // delay(0.5);

  // y_stepper.tmc.setMicrostepsPerStep(u_step);
  // y_stepper.stepper.setStepsPerRevolution(steps_per_revolution * u_step);
  // y_stepper.set_hold_current(RUN_CURRENT_PERCENT);
  // y_stepper.set_run_current(RUN_CURRENT_PERCENT);
  // y_stepper.set_speed(max_speed);
  // y_stepper.set_accel(accel);


  // check motor setups
  Serial.println("X motor setup\n---------------------");
  bool temp = x_stepper.test_setup();
  x_stepper.test_connection();
  x_stepper.print_parameters();
  
  Serial.println("Y motor setup\n---------------------");
  temp = y_stepper.test_setup();
  y_stepper.test_connection();
  y_stepper.print_parameters();


  // setup mpu6050 
  mpu.Initialize();
  // timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(timer, mpu_callback, true);
  // timerAlarmWrite(timer, 100000, true);
  // timerAlarmEnable(timer);
  // devStatus = mpu.accelgyro.dmpInitialize();
  mpu.accelgyro.setXAccelOffset(-1422);
  mpu.accelgyro.setYAccelOffset(243);
  mpu.accelgyro.setZAccelOffset(5626);
  mpu.accelgyro.setXGyroOffset(16);
  mpu.accelgyro.setYGyroOffset(126);
  mpu.accelgyro.setZGyroOffset(98);
  // mpu.accelgyro.setDMPEnabled(true);

  // attach mpu6050 DMP pin to intterupt
  // attachInterrupt(digitalPinToInterrupt(mpu_int_pin), dmp_data_ready, RISING);
  // mpuIntStatus = mpu.accelgyro.getIntStatus();
  // set DMP ready flag so main loop() can use it
  // dmpReady = true;

  // packetSize = mpu.accelgyro.dmpGetFIFOPacketSize();


  // start flexy stepper services on core 0
  // arduino stack runs on core 1
  x_stepper.stepper.startAsService(0);
  y_stepper.stepper.startAsService(0);

  // enable motors
  x_stepper.enable();
  y_stepper.enable();

  // set initial motor 0 position
  x_stepper.stepper.setCurrentPositionInRevolutions(0);
  y_stepper.stepper.setCurrentPositionInRevolutions(0);

  Serial.println("-------------- ready --------------");
}

// stream angles from Serial0
void angle_serial_read() {
  recv_with_end_marker();
  if (new_data == true) {
    // make copy because strtok in parse_angles() replaces commas
    strcpy(temp_chars, received_chars);

    parse_angles();
    // print_angles();

    new_data = false;
  }
}

float dir = 1.00;

// set new Flexy Stepper targets for motors
void update_motors() {
  // set target position based if new angles are available
  // if (y_angle_curr != y_angle_prev) {
  //   y_stepper.set_target(-y_angle_curr, y_max_angle);
  //   // Serial.print("New y angle: ");
  //   // Serial.println(y_angle_curr);
  //   y_angle_prev = y_angle_curr;
  //   return;
  // }
  if (x_angle_curr != x_angle_prev) {
    x_stepper.set_target(-x_angle_curr, x_max_angle);
    x_angle_prev = x_angle_curr;
    return;
  }

  // spin motors example
  // if (x_stepper.stepper.getDistanceToTargetSigned() == 0) {
  //   x_stepper.stepper.setTargetPositionRelativeInRevolutions(1);
  //   Serial.printf("Moving x stepper by %ld steps\n", u_step * steps_per_revolution);
  // }
  if (y_stepper.stepper.getDistanceToTargetSigned() == 0) {
    y_stepper.stepper.setTargetPositionRelativeInRevolutions(1);
    // y_stepper.stepper.setTargetPositionInRevolutions(y_max_angle * dir);
    dir *= -1;
    // Serial.printf("Moving y stepper by %ld steps\n", u_step * steps_per_revolution);
    return;
  }
}


void loop() {
  // toggle onboard LED to indicate activity
  digitalWrite(LED, !digitalRead(LED));

  angle_serial_read();
  update_motors();

  // Get new sensor events with the readings 
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
