import sys, time
from TMC_2209_StepperDriver import *
from mpu6050 import ANGLE_KAL, FILTER_ANGLES, MPU6050
import pins
from machine import Timer

# get new accel ofs calibration after power cycle
# ofs = (-1428, 247, 5644, 16, 138, 125)
ofs=(-1412, 241, 5642, 15, 137, 122)
#------------------------------------------------------
# tilt_controller
# 
# create 2 TMC2209 and 1 mpu6050 drivers
#------------------------------------------------------
class tilt_controller:
    x_step = None   # x tmc2209 stepper
    x_range = None  # tuple of max x angles
    y_step = None   # y tmc2209 stepper
    y_range = None  # tuple of max y angles
    sensor = None   # mpu6050
    target = None   # target table angle (x, y)
    current = None  # current table angle (x, y)
    converge = 0.2  # acceptable angle difference for convergence

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
        if not self.sensor.passed_self_test:
            print('MPU6050 init error!')
            exit()
        
        # take initial board measurement
        self.current = self.sensor.angles


    # set parameters for a particular motor
    def set_parameters(self, motor, ustep):
        # relative movement
        motor.setMovementAbsRel(MovementAbsRel.relative)

        motor.setDirection_reg(False)
        motor.setVSense(True)
        motor.setCurrent(300)
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
        # check what direction to step 
        curtime = time.ticks_us()

        # move x motor
        if (curtime - self.x_step._lastStepTime >= self.x_step._stepInterval):
            if not self.at_target(0, self.x_step):
                self.x_step.makeAStep()
                self.x_step._lastStepTime = curtime # does not account for makeAStep() costs

        # move y motor
        if (curtime - self.y_step._lastStepTime >= self.y_step._stepInterval):
            if not self.at_target(1, self.y_step):
                self.y_step.makeAStep()
                self.y_step._lastStepTime = curtime # does not account for makeAStep() costs


    # set direction pin 
    # 0 = CCW/right; 1 = CW/left
    def check_dir(self, axis, motor):

        # set x or y to rotate right
        if self.current[axis] < self.target[axis]:
            motor.setDirection_pin(0)

        # set x or y to rotate left
        else:
            motor.setDirection_pin(1)


    def at_target(self, axis, motor):
        if abs(self.target[axis] - self.current[axis]) < self.converge:
            return True
        else:
            self.check_dir(axis, motor)
            return False
        


    def sensor_callback(self, t):
        sample = self.sensor.angles
        # print(f"roll: {round(sample[0], 2)} | pitch: {round(sample[0], 2)}")

        # try keeping running avg
        # swap pitch and roll values in sample tuple
        self.current = (round(sample[1], 2), round(sample[0], 2))
        


# table parameters
ustep = 2
x_range = (-2, 2)
y_range = (-2, 2)
initial_target = (0, 0)

# create controller
# tilt_controller(microstep, logLevel)
# c = tilt_controller(ustep, True)
c = tilt_controller(ustep)

c.set_max_angles(x_range, y_range)
c.set_tilt(initial_target)


c.set_max_accel(c.y_step, 1500)
c.set_max_accel(c.x_step, 1500)
c.set_max_speed(c.y_step, 100)
c.set_max_speed(c.x_step, 100)


# uncomment for various tests
# print('x_motor_test')
# print(c.x_step.readStepsPerRevolution())
# c.uart_test(c.x_step)
# c.x_step.testDirStepEn()
# c.motor_test(c.x_step)

# print('y_motor_test')
# print(c.y_step.readStepsPerRevolution())
# c.uart_test(c.y_step)
# c.y_step.testDirStepEn()
# c.motor_test(c.y_step)


# create timers
t0 = Timer(0)
t1 = Timer(1)
t2 = Timer(2)
t3 = Timer(3)

# motor callback
t0.init(period=1, callback=c.motor_callback)

# accelerometer callback
t1.init(period=1, callback=c.sensor_callback)

# deinit
# del c

# spin
while(1):
    pass
