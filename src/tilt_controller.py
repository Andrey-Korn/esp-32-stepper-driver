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
        print('init')
        self.x_step = TMC_2209(pins._x_step, pins._x_dir, pins._x_en, serialport=1)
        self.set_parameters(self.x_step, ustep)

        self.y_step = TMC_2209(pins._y_step, pins._y_dir, pins._y_en, serialport=2)
        self.set_parameters(self.y_step, ustep)

        # turn on motor driver logs
        # if log:
            # self.y_step.setLoglevel(Loglevel.info)

        print('finish')

    # set parameters for a particular motor
    def set_parameters(self, motor, ustep):
        # relative movement
        motor.setMovementAbsRel(MovementAbsRel.relative)

        motor.setDirection_reg(False)
        # motor.setVSense(True)
        motor.setVSense(True)
        motor.setCurrent(400)
        motor.setIScaleAnalog(True)
        motor.setInterpolation(True)
        motor.setSpreadCycle(False)
        motor.setMicrosteppingResolution(ustep)
        motor.setInternalRSense(False)

    # destroy tmc2209s
    def __del__(self):
        del self.x_step
        del self.y_step

    def uart_test(self, motor):
        motor.readIOIN()
        motor.readCHOPCONF()
        motor.readDRVSTATUS()
        motor.readGCONF()

    def set_max_angles(self, x_range, y_range):
        pass

    def set_max_speed(self, motor, s):
        motor.setMaxSpeed(s)

    def set_max_accel(self, motor, a):
        motor.setAcceleration(a)

    # simple test routine for back and forth
    def motor_test(self, motor):
        # print('start')
        motor.setDirection_pin(1)
        motor.setMotorEnabled(True)
        for i in range(10):
            motor.runToPositionSteps(1000, MovementAbsRel.relative)
            motor.runToPositionSteps(-500, MovementAbsRel.relative)
        
        motor.setMotorEnabled(False)
        # print('finish')



# table parameters
ustep = 2
x_range = (-2, 2)
y_range = (-2, 2)

# create controller
# (microstep, logLevel)
c = tilt_controller(ustep, True)
# c = tilt_controller(ustep)

c.set_max_angles(x_range, y_range)

# uncomment for various tests
# c.uart_test(c.y_step)
c.set_max_accel(c.y_step, 2000)
c.set_max_accel(c.x_step, 2000)
c.set_max_speed(c.y_step, 1000)
c.set_max_speed(c.x_step, 1000)


# print('y_motor_test')
c.motor_test(c.y_step)

# print('x_motor_test')
c.motor_test(c.x_step)

# deinit
del c

# spin
while(1):
    pass
