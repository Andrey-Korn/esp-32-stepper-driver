#pragma once
#include <Arduino.h>
#include <ESP_FlexyStepper.h>
#include <TMC2209.h>

class stepper_driver {
    private:

        const int DELAY = 1000;
        const int MOTOR_STEP_PIN = 0;
        const int MOTOR_DIRECTION_PIN = 4;

        const int SPEED_SPS = 300;
        const int ACCEL_SPS = 800;

        HardwareSerial & serial_stream = Serial1;

        // create TMC2209 
        TMC2209 tmc;

    public:

        // constructor
        stepper_driver();

        // create stepper controller
        ESP_FlexyStepper stepper;

        // tmc2209 functions
        bool test_setup(); 
        void test_connection();
        void print_parameters();
        void set_ustep(int);
        void set_current(uint8_t);

        // tmc2209 on/off
        void enable();
        void disable();

        // flexy stepper

}; // stepper_driver
