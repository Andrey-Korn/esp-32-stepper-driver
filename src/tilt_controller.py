import sys, time, select
from TMC_2209_StepperDriver import *
from mpu6050 import ANGLE_KAL, FILTER_ANGLES, MPU6050
import pins
from machine import Timer
# from micropython import schedule

# get new accel ofs calibration after power cycle
ofs=(-1428, 241, 5646, 14, 134, 119)

# poll stdin for x/y angles
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

#------------------------------------------------------
# tilt_controller
# 
# create 2 TMC2209 and 1 mpu6050 drivers
#------------------------------------------------------
class tilt_controller:
    serial_count = 0
    sensor_count = 0
    x_step = None   # x tmc2209 stepper
    y_step = None   # y tmc2209 stepper
    range = None  # tuple of max allowable x/y angles
    sensor = None   # mpu6050
    target = [0, 0]   # target table angle (x, y)
    curr_sample = None  # current table angle (x, y)
    prev_sample = None # prev table angle (x, y)
    position = None # current belief of position (x, y)
    converge = 0.5  # acceptable angle difference for convergence
    far_away = (1.5, 2)

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
        self.curr_sample = self.sensor.angles
        self.prev_sample = self.curr_sample
        self.position = self.curr_sample


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
    def set_max_angles(self, range_tuple):
        self.range = range_tuple

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
        self.target[0] = angles[0] * self.range[0]
        self.target[1] = angles[1] * self.range[1]


    # step motors towards target angles
    def motor_callback(self, t):
        # if self.sensor_count < 5:
            # self.sensor_count += 1
        # else:
            # self.sensor_callback()
            # self.sensor_count = 0
        self.sensor_callback()

        # if self.serial_count < 50:
        #     self.serial_count += 1
        # else:
        #     self.serial_read()
        #     self.serial_count = 0

        curtime = time.ticks_us()

        # print(f'target: {self.target}')
        # move x motor
        if (curtime - self.x_step._lastStepTime >= self.x_step._stepInterval):
            if not self.at_target(0, self.x_step):
                self.x_step.makeAStep()
                # time.sleep(0.00005)
                # self.x_step.makeAStep()
                # if abs(self.target[0] - self.position[0]) > self.far_away[0]:
                    # time.sleep(0.0001)
                    # self.x_step.makeAStep()
                    # time.sleep(0.00005)
                    # self.x_step.makeAStep()
                self.x_step._lastStepTime = curtime # does not account for makeAStep() costs

        # move y motor
        if (curtime - self.y_step._lastStepTime >= self.y_step._stepInterval):
            if not self.at_target(1, self.y_step):
                self.y_step.makeAStep()
                # time.sleep(0.00005)
                # self.y_step.makeAStep()
                # if abs(self.target[1] - self.position[1]) > self.far_away[1]:
                    # time.sleep(0.0001)
                    # self.y_step.makeAStep()
                    # time.sleep(0.00005)
                    # self.y_step.makeAStep()
                self.y_step._lastStepTime = curtime # does not account for makeAStep() costs


    # set direction pin 
    # 0 = CCW/right; 1 = CW/left
    def check_dir(self, axis, motor):

        # set x or y to rotate right
        if self.position[axis] < self.target[axis]:
            motor.setDirection_pin(0)

        # set x or y to rotate left
        else:
            motor.setDirection_pin(1)


    def at_target(self, axis, motor):
        if abs(self.target[axis] - self.position[axis]) < self.converge:
            return True
        else:
            self.check_dir(axis, motor)
            return False
        


    def sensor_callback(self):
    # def sensor_callback(self, t):
        self.prev_sample = self.curr_sample
        # sample = self.sensor.angles
        # self.prev_sample = (round(sample[1], 2), round(sample[0], 2))
        sample = self.sensor.angles
        # swap pitch and roll values in sample tuple
        self.curr_sample = (round(sample[1], 2), round(sample[0], 2))

        # try keeping running avg
        # self.position = self.sensor_avg()
        self.position = self.curr_sample

    def sensor_avg(self):
        return (self.curr_sample[0] + self.prev_sample[0] / 2, 
                self.curr_sample[1] + self.prev_sample[1] / 2)

    # poll stdin for 1 ms
    def serial_read(self):
    # def serial_read(self, t):
        stream = poll.poll(1)
        # do nothing if no input seen
        if stream == []:
           pass
        else:        
            data = sys.stdin.readline()
            data = data.strip('\n')
            data = data.split(',')
            data[0] = round(float(data[0]), 2)
            data[1] = round(float(data[1]), 2)
            print(f'{data[0]} {data[1]}')
            self.set_tilt((data[0], data[1]))

        # data = sys.stdin.readline()
        # data = data.strip('\n')
        # data = data.split(',')
        # data[0] = round(float(data[0]), 2)
        # data[1] = round(float(data[1]), 2)
        # # print(f'{data[0]} {data[1]}')
        # self.set_tilt((data[0], data[1]))


        


# table parameters
ustep = 2
# ustep = 16
# max range (x, y)
range = (2, 2)
initial_target = (0, 0)

# create controller
# tilt_controller(microstep, logLevel)
# c = tilt_controller(ustep, True)
c = tilt_controller(ustep)

c.set_max_angles(range)
c.set_tilt(initial_target)


c.set_max_accel(c.y_step, 1000)
c.set_max_accel(c.x_step, 1000)
c.set_max_speed(c.y_step, 2000)
c.set_max_speed(c.x_step, 2000)


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
# t1.init(period=2, callback=c.sensor_callback)

# t2.init(period=50, callback=c.serial_read)

# deinit
# del c

# spin
while(1):
    pass
