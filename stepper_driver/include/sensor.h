#pragma once
#include <Arduino.h>
#include <I2Cdev.h>
// #include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <MPU6050.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// define I2C pins
#ifndef I2C_SDA
#define I2C_SDA 21
#endif

#ifndef I2C_SCL
#define I2C_SCL 22
#endif

class mpu_driver {
    private:

    public:

        // create accelerometer
        MPU6050 accelgyro;

        mpu_driver();
        void ForceHeader();
        void GetSmoothed();
        void Initialize();
        void SetOffsets(int *);
        void ShowProgress();
        void PullBracketsIn();
        void PullBracketsOut();
        void SetAveraging(int);

        const char LBRACKET = '[';
        const char RBRACKET = ']';
        const char COMMA    = ',';
        const char BLANK    = ' ';
        const char PERIOD   = '.';

        const int iAx = 0;
        const int iAy = 1;
        const int iAz = 2;
        const int iGx = 3;
        const int iGy = 4;
        const int iGz = 5;

        const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
        const int NFast =  1000;    // the bigger, the better (but slower)
        const int NSlow = 10000;    // ..
        const int LinesBetweenHeaders = 5;
            int LowValue[6];
            int HighValue[6];
            int Smoothed[6];
            int LowOffset[6];
            int HighOffset[6];
            int Target[6];
            int LinesOut;
            int N;

        int baudRate;


}; //mpu_driver
