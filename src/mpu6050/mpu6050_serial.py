from machine import SoftI2C, Pin, Timer, UART
from mpu6050 import MPU6050

# get new ofs calibration after power cycle
ofs = (-3094, 205, 3754, 9, 111, 61)

# ESP32 default scl/sda pins
scl_pin_num = 22
sda_pin_num = 21

# sensor sampling
sample_rate = 10
sample_t = 5000

# UART connection to PC
baudrate = 9600
uart = UART(1, baudrate)
uart.init(baudrate, bits=8, parity=None, stop=1)

tim0 = Timer(0)
tim1 = Timer(1)

# iterrupt/callback
def sensor_callback(t):
    if 'mpu' in globals():
        # mpu.print_angles()
        # mpu.print_all()

        # data to sample
        sample = mpu.angles + mpu.data

        uart.write(sample)


# create accelerometer
mpu = MPU6050(1, sda_pin_num, scl_pin_num, ofs)
if not mpu.passed_self_test:
    print('MPU6050 init error!')
    exit()

def stop_recording(t):

    tim0.deinit()
    tim1.deinit()
    jsonfile.close()

    print(f'Sampling over. Log saved to {filename}')


print(f'Sample rate: {sample_rate}. Sample time: {sample_t}')
print('Sampling started!...')

# start timers
tim0.init(period=sample_rate, callback=sensor_callback)
tim1.init(period=sample_t, mode=Timer.ONE_SHOT, callback=stop_recording)

# spin
while(1):
    pass
