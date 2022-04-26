from mpu6050 import ANGLE_KAL, FILTER_ANGLES, MPU6050
import pins

# record ofs output of the following line, and add it to MPU6050 config
# MPU6050(1, pins._sda, pins._scl)
MPU6050(1, pins._sda, pins._scl, 
        filtered=FILTER_ANGLES, 
        anglefilter=ANGLE_KAL)
