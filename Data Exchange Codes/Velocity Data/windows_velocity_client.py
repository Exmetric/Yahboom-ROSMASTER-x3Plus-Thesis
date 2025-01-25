# Please make sure to first read Velocity_Data_Instructions.txt before.

import asyncio                  # For asynchronous operations, allowing multiple processes at the same time
import socket                   # Network communication, TCP or UDP
import pickle                   # To transport data over the network, and convert data (bytes to Python object)
import signal                   # Handler, like when pressing Ctrl + c
import sys                      # Needed for system specific parameters and functions
from datetime import datetime   # Handling date and time operations  

class VelocityClient:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        # Store network configuration
        self.local_ip = local_ip            # Stores local IP address (Windows PC)
        self.local_port = local_port        # Stores local port number (Windows PC)
        self.remote_ip = remote_ip          # Stores IP address of the remote device (Jetson)
        self.remote_port = remote_port      # Stores port number of the remote device (Jetson)
        self.running = True                 # Checks if communicator is running
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))                # Binds socket to local address
        self.sock.setblocking(False)                                    # Enables non-blocking mode
        
        # Setup signal handler (ctrl + c)
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print(f"Velocity Client initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")

    # Handler for shutdown on ctrl+c    
    def signal_handler(self, signum, frame):
        print("\nStopping Velocity Client...")
        self.running = False                        
        if hasattr(self, 'sock'):
            self.sock.close()                       # Closes socket on shutdown, essential
        sys.exit(0)                                 # Exits program

    async def register(self, loop):
        print("Registering with robot...")
        max_attempts = 5
        attempt = 0
        
        while attempt < max_attempts and self.running:
            try:
                # Send registration request
                print(f"Registration attempt {attempt + 1}/{max_attempts}")
                self.sock.sendto("REGISTER".encode('utf-8'), (self.remote_ip, self.remote_port))
                
                # Wait for confirmation with timeout
                try:
                    data = await asyncio.wait_for(
                        loop.sock_recv(self.sock, 1024),
                        timeout=5.0
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
                await asyncio.sleep(2)                              # Waits 2 seconds before trying again
        
        print("Registration failed after all attempts")
        return False

    # Format and print velocity data with timestamp, hour, minute and second as a float
    def print_velocity_info(self, vel_data):
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"\n=== Velocity Data at {current_time} ===")
        
        # Print linear velocity components
        print("Linear Velocity:")
        print(f"  X: {vel_data['linear']['x']:8.3f} m/s")
        print(f"  Y: {vel_data['linear']['y']:8.3f} m/s")
        print(f"  Z: {vel_data['linear']['z']:8.3f} m/s")
        
        # Print angular velocity components
        print("\nAngular Velocity:")
        print(f"  X: {vel_data['angular']['x']:8.3f} rad/s")
        print(f"  Y: {vel_data['angular']['y']:8.3f} rad/s")
        print(f"  Z: {vel_data['angular']['z']:8.3f} rad/s")
        
        # Calculate total linear speed (magnitude)
        speed = (vel_data['linear']['x']**2 + 
                vel_data['linear']['y']**2 + 
                vel_data['linear']['z']**2)**0.5
        
        # Calculate total angular speed (magnitude)
        angular_speed = (vel_data['angular']['x']**2 + 
                        vel_data['angular']['y']**2 + 
                        vel_data['angular']['z']**2)**0.5
        
        # Print total speeds
        print(f"\nTotal Speed: {speed:8.3f} m/s")
        print(f"Total Angular Speed: {angular_speed:8.3f} rad/s")
        print("=" * 40)

    # Main loop for receiving velocity data
    async def receive_data(self, loop):
        while self.running:
            try:
                # Receive and deserialize velocity data
                data = await loop.sock_recv(self.sock, 65535)       # 65535 bytes, maximum buffer size
                vel_data = pickle.loads(data)
                self.print_velocity_info(vel_data)                  # Displays the data
            except Exception as e:
                if self.running:
                    print(f"Error receiving data: {e}")
                    await asyncio.sleep(1)                          # Waits for 1 second before retrying

async def main():
    # Network config
    local_ip = "0.0.0.0"            # Listens on all interfaces
    local_port = 7532               # Sending port of this device (Windows PC)
    remote_ip = "100.65.149.245"    # The robot's IP, if set up with NetBird, put in the provided ip adress for the Jetson/robot.
    remote_port = 7531              # Listening port of remote device (Jetson)

    # Create and get the event loop (async loop)
    loop = asyncio.get_event_loop()
    
    # Create client instance
    client = VelocityClient(local_ip, local_port, remote_ip, remote_port)
    
    # Register and start receiving data if successful
    if await client.register(loop):
        print("Starting to receive velocity data...")
        print("Press Ctrl+C to stop")
        await client.receive_data(loop)

# Program entry point
if __name__ == '__main__':
    try:
        asyncio.run(main())                         # Run async main function
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")