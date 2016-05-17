import time

from bitify.python.sensors.adxl345 import ADXL345
from bitify.python.sensors.l3g4200d import L3G4200D
from bitify.python.sensors.hmc5883l import HMC5883L

class IMU(object):
    
    K = 0.98
    K1 = 1 - K
    
    def __init__(self, bus, gyro_address, accel_address, compass_address, name, gyro_scale=L3G4200D.FS_2000, accel_scale=ADXL345.AFS_16g):
        self.bus = bus
        self.gyro_address = gyro_address 
        self.accel_address = accel_address
        self.name = name
        self.gyro_scale = gyro_scale 
        self.accel_scale = accel_scale
        self.accelerometer = ADXL345(bus, accel_address, name + "-accelerometer", accel_scale)
        self.gyroscope = L3G4200D(bus, gyro_address, name + "-gyroscope", gyro_scale)
        self.compass = HMC5883L(bus, compass_address, name + "-compass")

        self.last_time = time.time()
        self.time_diff = 0

        self.pitch = 0
        self.roll = 0
        # take a reading from the device to allow it to settle after config changes
        self.read_all()
        # now take another to act a starting value
        self.read_all()
        self.pitch = self.rotation_x
        self.roll = self.rotation_y

    def read_all(self):
        '''Return pitch and roll in radians and the scaled x, y & z values from the gyroscope and accelerometer'''
        self.gyroscope.read_raw_data()
        self.accelerometer.read_raw_data()
        
        self.gyro_scaled_x = self.gyroscope.read_scaled_gyro_x()
        self.gyro_scaled_y = self.gyroscope.read_scaled_gyro_y()
        self.gyro_scaled_z = self.gyroscope.read_scaled_gyro_z()
        
        self.accel_scaled_x = self.accelerometer.read_scaled_accel_x()
        self.accel_scaled_y = self.accelerometer.read_scaled_accel_y()
        self.accel_scaled_z = self.accelerometer.read_scaled_accel_z()
        
        self.rotation_x = self.accelerometer.read_x_rotation(self.accel_scaled_x, self.accel_scaled_y, self.accel_scaled_z)
        self.rotation_y = self.accelerometer.read_y_rotation(self.accel_scaled_x, self.accel_scaled_y, self.accel_scaled_z)
        
        now = time.time()
        self.time_diff = now - self.last_time
        self.last_time = now 
        (self.pitch, self.roll) = self.comp_filter(self.rotation_x, self.rotation_y)
        
        # return (self.pitch, self.roll, self.gyro_scaled_x, self.gyro_scaled_y, self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, self.accel_scaled_z)
        return (self.pitch, self.roll, self.gyro_scaled_x, self.gyro_scaled_y, self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, self.accel_scaled_z)
        
    def read_x_rotation(self, x, y, z):
        return self.rotation_x

    def read_y_rotation(self, x, y, z):
        return self.rotation_y

    def comp_filter(self, current_x, current_y):
        new_pitch = IMU.K * (self.pitch + self.gyro_scaled_x * self.time_diff) + (IMU.K1 * current_x)
        new_roll = IMU.K * (self.roll + self.gyro_scaled_y * self.time_diff) + (IMU.K1 * current_y)
        return (new_pitch, new_roll)


    def read_pitch_roll_yaw(self):
        '''
        Return pitch, roll and yaw in radians
        '''
        (raw_pitch, raw_roll, self.gyro_scaled_x, self.gyro_scaled_y, \
            self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, \
            self.accel_scaled_z) = self.read_all()
        
        now = time.time()
        self.time_diff = now - self.last_time
        self.last_time = now 
        
        (self.pitch, self.roll) = self.comp_filter(raw_pitch, raw_roll)
        self.yaw = self.compass.read_compensated_bearing(self.pitch, self.roll)
        
        return (self.pitch, self.roll, self.yaw)

    def set_compass_offsets(self,x_offset, y_offset, z_offset):
        self.compass.set_offsets(x_offset, y_offset, z_offset)
