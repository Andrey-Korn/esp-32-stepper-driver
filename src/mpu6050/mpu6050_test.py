from machine import SoftI2C, Pin, Timer
from mpu6050 import ANGLE_KAL, FILTER_ANGLES, MPU6050
import pins

# get new ofs calibration after power cycle
ofs=(-1428, 247, 5644, 16, 138, 125)

tim0 = Timer(0)

# iterrupt/callback
def sensor_callback(t):
    if 'mpu' in globals():
        mpu.print_angles()
        # mpu.print_all()
        # mpu.print_data()

# create accelerometer
mpu = MPU6050(1, pins._sda, pins._scl, ofs, 
              filtered=FILTER_ANGLES,
              anglefilter=ANGLE_KAL)

if not mpu.passed_self_test:
    print('MPU6050 init error!')
    exit()

# start timers
tim0.init(period=100, callback=sensor_callback)

# spin
while(1):
    pass
