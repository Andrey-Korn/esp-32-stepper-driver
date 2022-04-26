#pragma once
#include <Arduino.h>
#include <ESP_FlexyStepper.h>
#include <TMC2209.h>

class stepper_driver {
    private:

        const int DELAY = 100;
        int MOTOR_EN_PIN;
        HardwareSerial & serial_stream = Serial1;

    public:

        // constructor
        stepper_driver();

        // create stepper controller
        ESP_FlexyStepper stepper;

        // create TMC2209 
        TMC2209 tmc;

        // tmc2209 functions
        bool test_setup(); 
        void test_connection();
        void print_parameters();
        void set_ustep(int);
        void connect_motor_pins(int, int, int);
        void set_run_current(uint8_t);
        void set_hold_current(uint8_t);
        void set_motor_parameters(int, int, int, int, uint8_t);

        // tmc2209 on/off
        void enable();
        void disable();

        // flexy stepper
        void set_target(float, float);
        void set_speed(int);
        void set_accel(int);

}; // stepper_driver
