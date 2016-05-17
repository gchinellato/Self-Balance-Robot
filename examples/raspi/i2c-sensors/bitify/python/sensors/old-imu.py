import time

from bitify.python.sensors.mpu6050 import MPU6050
from bitify.python.sensors.hmc5883l import HMC5883L

class OLDIMU(object):
    
    K = 0.98
    K1 = 1 - K
    
    def __init__(self, bus, gyro_address, compass_address, name, gyro_scale=MPU6050.FS_2000, accel_scale=MPU6050.AFS_16g):
        self.bus = bus
        self.gyro_address = gyro_address 
        self.name = name
        self.gyro_scale = gyro_scale 
        self.accel_scale = accel_scale
        self.gyroscope = MPU6050(bus, gyro_address, name + "-gyroscope", gyro_scale, accel_scale)
        self.compass = HMC5883L(bus, compass_address, name + "-compass")

        self.last_time = time.time()
        self.time_diff = 0

        # Take a reading as a starting point
        (self.pitch, self.roll, self.gyro_scaled_x, self.gyro_scaled_y, \
            self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, \
            self.accel_scaled_z) = self.gyroscope.read_all()

    def comp_filter(self, current_x, current_y):
        '''
        Apply a complementary filter to the Gyroscope and Accelerometer data
        '''
        new_pitch = IMU.K * (self.pitch + self.gyro_scaled_x * self.time_diff) + (IMU.K1 * current_x)
        new_roll = IMU.K * (self.roll + self.gyro_scaled_y * self.time_diff) + (IMU.K1 * current_y)
        return (new_pitch, new_roll)

    def read_pitch_roll_yaw(self):
        '''
        Return pitch, roll and yaw in radians
        '''
        
        (raw_pitch, raw_roll, self.gyro_scaled_x, self.gyro_scaled_y, \
            self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, \
            self.accel_scaled_z) = self.gyroscope.read_all()
        
        now = time.time()
        self.time_diff = now - self.last_time
        self.last_time = now 
        
        (self.pitch, self.roll) = self.comp_filter(raw_pitch, raw_roll)
        self.yaw = self.compass.read_compensated_bearing(self.pitch, self.roll)
        
        return (self.pitch, self.roll, self.yaw)

    def set_compass_offsets(self,x_offset, y_offset, z_offset):
        self.compass.set_offsets(x_offset, y_offset, z_offset)