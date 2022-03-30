import sys, time
from TMC_2209_StepperDriver import *
import pins

#------------------------------------------------------
# tilt_controller
# 
# create 2 TMC2209 and 1 mpu6050 drivers
# functions:
# 1. set target table angle
# 2. set max_speed
# 3. set max acceleration 
# 4. set allowable offset between current and target
#------------------------------------------------------
class tilt_controller:
    x_step = None
    y_step = None

    def __init__(self, ustep, log=False):
        # create TMC_2209 drivers with the correct pins
        # self.x_step = TMC_2209(pins._x_step, pins._x_dir, pins._x_en, serialport=1)
        self.y_step = TMC_2209(pins._y_step, pins._y_dir, pins._y_en, serialport=2)

        # turn on motor driver logs
        if log:
            self.y_step.setLoglevel(Loglevel.info)

        # set motor parameters
        # relative movement
        self.y_step.setMovementAbsRel(MovementAbsRel.relative)

        self.y_step.setDirection_reg(False)
        self.y_step.setVSense(True)
        self.y_step.setCurrent(300)
        self.y_step.setIScaleAnalog(True)
        self.y_step.setInterpolation(True)
        self.y_step.setSpreadCycle(False)
        self.y_step.setMicrosteppingResolution(ustep)
        self.y_step.setInternalRSense(False)

    # destroy tmc2209s
    def __del__(self):
        del self.y_step

    def uart_test(self):
        self.y_step.readIOIN()
        self.y_step.readCHOPCONF()
        self.y_step.readDRVSTATUS()
        self.y_step.readGCONF()

    def set_max_angles(self, x_range, y_range):
        pass

    def set_max_speed(self, s):
        self.y_step.setMaxSpeed(s)

    def set_max_accel(self, a):
        self.y_step.setAcceleration(a)

    # simple test routine for back and forth
    def motor_test(self, motor):
        # print('start')
        motor.setMotorEnabled(True)
        for i in range(10):
            motor.runToPositionSteps(1000, MovementAbsRel.relative)
            motor.runToPositionSteps(-500, MovementAbsRel.relative)
        
        motor.setMotorEnabled(False)
        # print('finish')



# table parameters
ustep = 1
x_range = (-2, 2)
y_range = (-2, 2)

# create controller
# (microstep, logLevel)
tilt = tilt_controller(ustep, True)
# tilt = tilt_controller(ustep)
tilt.set_max_angles(x_range, y_range)

# uncomment for various tests
# tilt.uart_test()
tilt.set_max_accel(2000)
tilt.set_max_speed(500)
tilt.motor_test(tilt.y_step)

# deinit
del tilt

# spin
while(1):
    pass
