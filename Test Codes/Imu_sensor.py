from smbus2 import SMBus
import time
import numpy as np

# I am using an IMU MPU6886 sensor from the AIoT garage, connected to the Jetson Orin Nano on physical pins 2, 14, 27, and 28. I did not need to enable any GPIO pins as we are not using any.
# The accelerometer readings seem to be accurate and working with in expected reange
# The gyroscope readings are ok except the Z-axis readings, which could mean many things
class MPU6886: # sensor class from data sheet
    MPU6886_ADDRESS = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    def __init__(self, bus_number=1):
        self.bus = SMBus(bus_number)
        
    def write_byte(self, reg, val):
        self.bus.write_byte_data(self.MPU6886_ADDRESS, reg, val)

    def read_bytes(self, reg, length):
        return self.bus.read_i2c_block_data(self.MPU6886_ADDRESS, reg, length)

    def initialize(self):
        self.write_byte(self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.write_byte(self.ACCEL_CONFIG, 0x00)
        self.write_byte(self.GYRO_CONFIG, 0x00)

    def read_accelerometer(self):
        data = self.read_bytes(self.ACCEL_XOUT_H, 6)
        x = np.int16((data[0] << 8) | data[1])
        y = np.int16((data[2] << 8) | data[3])
        z = np.int16((data[4] << 8) | data[5])
        
        x = x / 16384.0
        y = y / 16384.0
        z = z / 16384.0
        
        return (x, y, z)

    def read_gyroscope(self):
        data = self.read_bytes(self.GYRO_XOUT_H, 6)
        x = np.int16((data[0] << 8) | data[1])
        y = np.int16((data[2] << 8) | data[3])
        z = np.int16((data[4] << 8) | data[5])
        
        x = x / 131.0
        y = y / 131.0
        z = z / 131.0
        
        return (x, y, z)

def main():
    sensor = MPU6886()
    sensor.initialize()
    
    while True:
        accel = sensor.read_accelerometer()
        gyro = sensor.read_gyroscope()
        
        print(f"Accelerometer (g): X={accel[0]:.2f}, Y={accel[1]:.2f}, Z={accel[2]:.2f}") #These will print the data as a float number rounded to 2 decimal places
        print(f"Gyroscope (deg/s): X={gyro[0]:.2f}, Y={gyro[1]:.2f}, Z={gyro[2]:.2f}")
        print("-----------------")
        
        time.sleep(0.5)

