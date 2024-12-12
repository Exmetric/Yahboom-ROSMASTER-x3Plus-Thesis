#!/usr/bin/env python3
from smbus2 import SMBus
import socket
import pickle
import signal
import sys
import asyncio
import time
import numpy as np

class MPU6886:
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

class IMUNetworkBridge:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.client_registered = False
        self.running = True
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))
        self.sock.setblocking(False)
        
        self.imu_sensor = MPU6886()
        self.imu_sensor.initialize()
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        print(f"IMU Bridge initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")
        
    def signal_handler(self, signum, frame):
        print("\nShutting down IMU Bridge...")
        self.running = False
        if hasattr(self, 'sock'):
            self.sock.close()
        sys.exit(0)

    async def wait_for_registration(self, loop):
        print("Waiting for client registration...")
        while self.running:
            try:
                data = await loop.sock_recv(self.sock, 1024)
                message = data.decode('utf-8', errors='ignore')
                if message == "REGISTER":
                    self.client_registered = True
                    print("Client registered successfully")
                    self.sock.sendto("REGISTERED".encode('utf-8'),
                                   (self.remote_ip, self.remote_port))
                    break
            except Exception as e:
                await asyncio.sleep(1)

    async def send_imu_data(self):
        while self.running:
            if self.client_registered:
                try:
                    accel = self.imu_sensor.read_accelerometer()
                    gyro = self.imu_sensor.read_gyroscope()
                    
                    data_dict = {
                        'accel': accel,
                        'gyro': gyro,
                        'timestamp': time.time()
                    }
                    
                    data = pickle.dumps(data_dict)
                    self.sock.sendto(data, (self.remote_ip, self.remote_port))
                except Exception as e:
                    print(f"Error sending IMU data: {e}")
                    self.client_registered = False
                
            await asyncio.sleep(0.1)  # Adjust this value to change the update rate, this will help with readability

    async def run(self, loop):
        await self.wait_for_registration(loop)
        await self.send_imu_data()

async def main():
    local_ip = "0.0.0.0"
    local_port = 7540  # Robot/Jetson's listening port
    remote_ip = "100.65.62.108"  # Windows client IP, if using vpn netbird, use provided IP adress for windows client
    remote_port = 7541  # Windows client's receiving port

    loop = asyncio.get_event_loop()
    bridge = IMUNetworkBridge(local_ip, local_port, remote_ip, remote_port)
    await bridge.run(loop)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")