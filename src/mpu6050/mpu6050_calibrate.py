from mpu6050 import MPU6050

# ESP32 default scl/sda pins
scl_pin_num = 22
sda_pin_num = 21

# record ofs output of the following line, and add it to MPU6050 config
MPU6050(1, sda_pin_num, scl_pin_num)
