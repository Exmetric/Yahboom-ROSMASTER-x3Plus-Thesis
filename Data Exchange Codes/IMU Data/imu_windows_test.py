import asyncio                  # For asynchronous operations, allowing multiple processes at the same time 
import socket                   # Network communication, TCP or UDP
import pickle                   # To transport data over the network, and convert data (bytes to Python object)
import signal                   # Handler, like when pressing Ctrl + c
import sys                      # Needed for system specific parameters and functions
from datetime import datetime   # Handling date and time operations

class IMUClient:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):   # Defining to make things easier/readable
        self.local_ip = local_ip                                        # Stores local IP address (Windows PC)
        self.local_port = local_port                                    # Stores local port number (Windows PC)
        self.remote_ip = remote_ip                                      # Stores remote IP address (Jetson)
        self.remote_port = remote_port                                  # Stores remote port number (Jetson)
        self.running = True                                              
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    # We create the UDP socket
        self.sock.bind((self.local_ip, self.local_port))                # We need to bind the socket to the local address
        self.sock.setblocking(False)                                    # Needed to set socket to "non-blocking mode"
        
        signal.signal(signal.SIGINT, self.signal_handler)               # The signal hanlder to stop the code with ctrl + c
        
        print(f"IMU Client initialized on {local_ip}:{local_port}")     # Print these to make it more user friendly, to know whats happening
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")
        
    def signal_handler(self, signum, frame):
        print("\nStopping IMU Client...")   
        self.running = False                # Stops the client
        if hasattr(self, 'sock'):
            self.sock.close()               # Closes the socket if it exists
        sys.exit(0)                         # Exits the program

    async def register(self, loop):
        print("Registering with robot...")
        max_attempts = 5    # Number of attemps before stopping code
        attempt = 0
        
        while attempt < max_attempts and self.running:
            try:
                print(f"Registration attempt {attempt + 1}/{max_attempts}")
                self.sock.sendto("REGISTER".encode('utf-8'), (self.remote_ip, self.remote_port))    # Sends registration attempt, utf8 is used encode the string into bytes
                
                try:
                    data = await asyncio.wait_for(                 
                        loop.sock_recv(self.sock, 1024),    # We used 1024 bytes which is small, because we just want the printed "REGISTERED" above
                        timeout=5.0                         # Wait for response, otherwise timesout
                    )
                    message = data.decode('utf-8', errors='ignore')
                    if message == "REGISTERED":
                        print("Successfully registered with robot")
                        return True
                except asyncio.TimeoutError:
                    print("Registration timeout, retrying...")
                
            except Exception as e:
                print(f"Registration attempt failed: {e}")
            
            attempt += 1
            if attempt < max_attempts:
                await asyncio.sleep(2)  # Waits 2 seconds before retrying
        
        print("Registration failed after all attempts")
        return False

    def print_imu_data(self, imu_data):
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # Get's current time in hour, minute, seconds as float format
        print(f"\n=== IMU Reading at {current_time} ===")
        print(f"Accelerometer (g):")
        print(f"  X: {imu_data['accel'][0]:.2f}")   # These will print the acceleration X, Y, Z data as a float number rounded to 2 decimal places
        print(f"  Y: {imu_data['accel'][1]:.2f}")
        print(f"  Z: {imu_data['accel'][2]:.2f}")
        print(f"Gyroscope (deg/s):")
        print(f"  X: {imu_data['gyro'][0]:.2f}")    # These will print the angular velocity X, Y, Z data as a float number rounded to 2 decimal places
        print(f"  Y: {imu_data['gyro'][1]:.2f}")
        print(f"  Z: {imu_data['gyro'][2]:.2f}")
        print("=" * 40)

    async def receive_data(self, loop):
        while self.running:
            try:
                data = await loop.sock_recv(self.sock, 65535)   # Recieve raw bytes (this is why we use pickle lib), we use the maximum of 65535 bytes here because the data is large
                imu_data = pickle.loads(data)                   # Converts the bytes back to python object
                self.print_imu_data(imu_data)                   # Prints the IMU data
            except Exception as e:
                if self.running:
                    print(f"Error receiving data: {e}")
                    await asyncio.sleep(1)                      # Waits 1 second before retrying

async def main():
    local_ip = "0.0.0.0"          # To listen on all available interfaces
    local_port = 7541             # Wiindows client receiving port
    remote_ip = "100.65.149.245"  # Robot IP, if using netbird vpn, use ip adress provided of robot
    remote_port = 7540            # Robot/Jetson's listening port

    loop = asyncio.get_event_loop()                                     # Gets event loop
    client = IMUClient(local_ip, local_port, remote_ip, remote_port)    # Create client
    
    if await client.register(loop):                 # Try to register
        print("Starting to receive IMU data...")    
        print("Press Ctrl+C to stop")
        await client.receive_data(loop)             # Starts recieving data

if __name__ == '__main__':
    try:
        asyncio.run(main())                         # Runs the async main function above
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")