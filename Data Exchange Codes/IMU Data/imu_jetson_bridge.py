#!/usr/bin/env python3
from smbus2 import SMBus    
import socket               # Network communication, TCP or UDP
import pickle               # To transport data over the network, and convert data (bytes to Python object)
import signal               # Handler, like when pressing Ctrl + c
import sys                  # Needed for system specific parameters and functions
import asyncio              # For asynchronous operations, allowing multiple processes at the same time
import time                 # For time related functions
import numpy as np          # For numerical operations

class MPU6886:                  # Decide class of IMU sensor used    
    MPU6886_ADDRESS = 0x68      # I2C address of IMU
    PWR_MGMT_1 = 0x6B           # Power management register
    ACCEL_CONFIG = 0x1C         # Accelerometer configuration register
    GYRO_CONFIG = 0x1B          # Gyroscope configuration register
    ACCEL_XOUT_H = 0x3B         # Starting register for accelerometer data
    GYRO_XOUT_H = 0x43          # Starting register for gyroscope data

    def __init__(self, bus_number=1):
        self.bus = SMBus(bus_number)    # Initializes the I2C bus
        
    def write_byte(self, reg, val):
        self.bus.write_byte_data(self.MPU6886_ADDRESS, reg, val)    # Writes a byte to a register on the IMU

    def read_bytes(self, reg, length):
        return self.bus.read_i2c_block_data(self.MPU6886_ADDRESS, reg, length)  # Reads multiple bytes from the IMU

    def initialize(self):                           # Initialize the IMU sensor
        self.write_byte(self.PWR_MGMT_1, 0x00)      # "Wakes" up the sensor
        time.sleep(0.1)                             # Waits 0.1 seconds for the wake up
        self.write_byte(self.ACCEL_CONFIG, 0x00)    # Sets accelerometer configuration
        self.write_byte(self.GYRO_CONFIG, 0x00)     # Sets gyrcoscope configuration

    def read_accelerometer(self):
        data = self.read_bytes(self.ACCEL_XOUT_H, 6)    # Reads and converts accelerometer data
        x = np.int16((data[0] << 8) | data[1])          # Converts the 16 bit integers to "g" scale, g=16384.0 for this IMU sensor.          
        y = np.int16((data[2] << 8) | data[3])
        z = np.int16((data[4] << 8) | data[5])
        
        x = x / 16384.0     # Converts to g-forces of IMU sensor into our (earth) g-force 
        y = y / 16384.0     # So we get the measurement of acceleration in terms of g-force (1g=9.81m/s^2)
        z = z / 16384.0
        
        return (x, y, z)

    def read_gyroscope(self):
        data = self.read_bytes(self.GYRO_XOUT_H, 6)     # Reads and converts the gyroscope data
        x = np.int16((data[0] << 8) | data[1])          
        y = np.int16((data[2] << 8) | data[3])          
        z = np.int16((data[4] << 8) | data[5])
        
        x = x / 131.0       # Converts to degrees per second, we use 131 because of the IMU sensor we are using
        y = y / 131.0
        z = z / 131.0
        
        return (x, y, z)    # Returns the tuple of rotation values in degrees/second in order of left/right, forward/backward, up/down

class IMUNetworkBridge:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):   # Defining to make things easier/readable
        self.local_ip = local_ip                                        # Stores local IP address (Jetson)
        self.local_port = local_port                                    # Stores local port number (Jetson)
        self.remote_ip = remote_ip                                      # Stores remote IP address (Windows PC)
        self.remote_port = remote_port                                  # Stores remote port number (Windows PC)
        self.client_registered = False
        self.running = True
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    # We create the UDP socket
        self.sock.bind((self.local_ip, self.local_port))                # We need to bind the socket to the local address
        self.sock.setblocking(False)                                    # Needed to set socket to "non-blocking" mode
        
        self.imu_sensor = MPU6886()                                     # Initializes the IMU sensor
        self.imu_sensor.initialize()
        # Set up signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)               # SIGINT for interrup signal, with ctrl + c  
        signal.signal(signal.SIGTERM, self.signal_handler)              # SIGTERM for interrup signal, system shutdown or when using "kill" comand
        
        print(f"IMU Bridge initialized on {local_ip}:{local_port}")     # Print these to make it more user friendly, to know whats happening
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")
        
    def signal_handler(self, signum, frame):
        print("\nShutting down IMU Bridge...")
        self.running = False                        # Stop the main loop when pressed ctrl + c
        if hasattr(self, 'sock'):
            self.sock.close()                       # Close network socket
        sys.exit(0)                                 # Exit program with success code

    async def wait_for_registration(self, loop):                            # So that we can wait for client to register before sending data                
        print("Waiting for client registration...")
        while self.running:                                                 # Keeps checking as long as program is running
            try:
                data = await loop.sock_recv(self.sock, 1024)                # This waits for incoming data, I used 1024 bytes which is more than enough for messages
                message = data.decode('utf-8', errors='ignore')             # Conversts recieved bytes to string, and ignores any invalid characters, utf8 is used  to decode the bytes into strings
                if message == "REGISTER":                                   # Checks if the recieved message is "REGISTER"
                    self.client_registered = True                           # Mark client as registered
                    print("Client registered successfully")
                    self.sock.sendto("REGISTERED".encode('utf-8'),          # Send back "REGISTERED" confirmation message to client with encoder
                                   (self.remote_ip, self.remote_port))
                    break                                                   # Exits the loops once registered
            except Exception as e:                                          # If there is any error, waits 1 second before trying again
                await asyncio.sleep(1)

    async def send_imu_data(self):
        # Function to continuously send IMU sensor data
        while self.running:                                                         # Keeps running until program stops                                    
            if self.client_registered:                                              # Only sends data if client is registered
                try:
                    accel = self.imu_sensor.read_accelerometer()                    # Gets current accelerometer readings (x,y,z)
                    gyro = self.imu_sensor.read_gyroscope()                         # Gets current gyroscope readings (x,y,z)
                    
                    data_dict = {                                                   # Creates dicitionary with all sensor data
                        'accel': accel,                                             # Acceleration values
                        'gyro': gyro,                                               # Rotation values 
                        'timestamp': time.time()                                    # Current time
                    }
                    
                    data = pickle.dumps(data_dict)                                      # Converts dictionary to bytes before sending
                    self.sock.sendto(data, (self.remote_ip, self.remote_port))          # Sends the data packet to client (Windows PC)
                except Exception as e:                                                  # If any error occurs, prints it and marks the client as unregistered
                    print(f"Error sending IMU data: {e}")
                    self.client_registered = False
                
            await asyncio.sleep(0.1)                                                # Waits 0.1 seconds before next reading (10 times per second)

    async def run(self, loop):                      # Waits for client to register
        await self.wait_for_registration(loop)      # When registered, starts sending the IMU data
        await self.send_imu_data()

# Here we configure our network settings
async def main():
    local_ip = "0.0.0.0"            # Listen on all interfaces
    local_port = 7540               # Robot/Jetson's listening port
    remote_ip = "100.65.62.108"     # Windows client IP, if using vpn netbird, use provided IP adress for windows client
    remote_port = 7541              # Windows client's receiving port

    loop = asyncio.get_event_loop()                                             # Gets event loop for async operations
    bridge = IMUNetworkBridge(local_ip, local_port, remote_ip, remote_port)     # Creates instance of "IMUNetworkBridge" with network settings
    await bridge.run(loop)                                                      # Runs the bridge, waits for registration and then sends data

if __name__ == '__main__':                         
    try:
        asyncio.run(main())                         # Starts the async program by running main()
    except KeyboardInterrupt:                       # If ctrl + c is pressed, shows shutdown message below
        print("\nShutdown requested... exiting")