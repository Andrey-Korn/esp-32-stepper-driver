from machine import SoftI2C, Pin, Timer
from mpu6050 import MPU6050

# get new ofs calibration after power cycle
ofs = (-3094, 205, 3754, 9, 111, 61)

# ESP32 default scl/sda pins
scl_pin_num = 22
sda_pin_num = 21

tim0 = Timer(0)

# iterrupt/callback
def sensor_callback(t):
    if 'mpu' in globals():
        # mpu.print_angles()
        mpu.print_all()
        # mpu.print_data()

# create accelerometer
mpu = MPU6050(1, sda_pin_num, scl_pin_num, ofs)
if not mpu.passed_self_test:
    print('MPU6050 init error!')
    exit()

# start timers
tim0.init(period=100, callback=sensor_callback)

# spin
while(1):
    pass
