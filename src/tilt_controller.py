import sys, time
from TMC_2209_StepperDriver import *
from mpu6050 import ANGLE_KAL, FILTER_ANGLES, MPU6050
import pins
from machine import Timer

# get new accel ofs calibration after power cycle
ofs = (-1428, 247, 5644, 16, 138, 125)

#------------------------------------------------------
# tilt_controller
# 
# create 2 TMC2209 and 1 mpu6050 drivers
#------------------------------------------------------
class tilt_controller:
    x_step = None
    x_range = None
    y_step = None
    y_range = None
    sensor = None

    def __init__(self, ustep, log=False):
        # create TMC_2209 drivers with the correct pins
        print('init')
        self.x_step = TMC_2209(pins._x_step, pins._x_dir, pins._x_en, serialport=2)
        self.set_parameters(self.x_step, ustep)

        self.y_step = TMC_2209(pins._y_step, pins._y_dir, pins._y_en, serialport=1)
        self.set_parameters(self.y_step, ustep)

        # turn on motor driver logs
        if log:
            self.y_step.setLoglevel(Loglevel.info)
            self.x_step.setLoglevel(Loglevel.info)

        # create mpu6050
        self.sensor = MPU6050(1, pins._sda, pins._scl, ofs, 
                              filtered=FILTER_ANGLES,
                              anglefilter=ANGLE_KAL)


    # set parameters for a particular motor
    def set_parameters(self, motor, ustep):
        # relative movement
        motor.setMovementAbsRel(MovementAbsRel.relative)

        motor.setDirection_reg(False)
        motor.setVSense(True)
        motor.setCurrent(200)
        motor.setIScaleAnalog(True)
        motor.setInterpolation(True)
        motor.setSpreadCycle(False)
        motor.setMicrosteppingResolution(ustep)
        motor.setInternalRSense(False)

    # destroy tmc2209s
    def __del__(self):
        del self.x_step
        del self.y_step

    # print uart registers
    def uart_test(self, motor):
        motor.readIOIN()
        motor.readCHOPCONF()
        motor.readDRVSTATUS()
        motor.readGCONF()

    # set max table tilt angles
    def set_max_angles(self, x_range, y_range):
        self.x_range = x_range
        self.y_range = y_range

    def set_max_speed(self, motor, s):
        motor.setMaxSpeed(s)

    def set_max_accel(self, motor, a):
        motor.setAcceleration(a)

    # simple test routine for back and forth
    def motor_test(self, motor):
        print('start')
        motor.setDirection_pin(1)
        motor.setMotorEnabled(True)
        motor.setCurrentPosition(0)
        for i in range(5):
            motor.runToPositionSteps(50, MovementAbsRel.relative)
            time.sleep(0.1)
            motor.runToPositionSteps(-50, MovementAbsRel.relative)
            time.sleep(0.1)
        
        motor.setMotorEnabled(False)
        print('finish')
    
    def set_tilt(self, angles):
        self.target = angles

    # step motors towards target angles
    def motor_callback(self, t):
        self.x_step.makeAStep()
        # delay steps
        time.sleep(1/1000/1000)
        self.y_step.makeAStep()




# table parameters
ustep = 3
x_range = (-2, 2)
y_range = (-2, 2)
initial_target = (0, 0)

# create controller
# tilt_controller(microstep, logLevel)
# c = tilt_controller(ustep, True)
c = tilt_controller(ustep)

c.set_max_angles(x_range, y_range)


c.set_max_accel(c.y_step, 1500)
c.set_max_accel(c.x_step, 1500)
c.set_max_speed(c.y_step, 750)
c.set_max_speed(c.x_step, 750)


# uncomment for various tests
print('x_motor_test')
# print(c.x_step.readStepsPerRevolution())
# c.uart_test(c.x_step)
# c.x_step.testDirStepEn()
c.motor_test(c.x_step)

print('y_motor_test')
# print(c.y_step.readStepsPerRevolution())
# c.uart_test(c.y_step)
# c.y_step.testDirStepEn()
c.motor_test(c.y_step)


# create timers
t0 = Timer(0)
t1 = Timer(1)
t2 = Timer(2)
t3 = Timer(3)

# motor callback
# t0.init(period=4, callback=c.motor_callback)

# accelerometer callback
# t1.init(period=5, callback=c.)

# deinit
# del c

# spin
while(1):
    pass
