# Make sure to read the Voltage_Data_Instructions.txt first

import asyncio                      # For asynchronous operations, allowing multiple processes at the same time
import socket                       # Network communication, TCP or UDP
import pickle                       # To transport data over the network, and convert data (bytes to Python object)
import signal                       # Handler, like when pressing Ctrl + c
import sys                          # Needed for system specific parameters and functions
from datetime import datetime       # Handling date and time operations    

class VoltageClient:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        # Store network config
        self.local_ip = local_ip                                            # Stores local IP address (Windows PC)
        self.local_port = local_port                                        # Stores local port number (Windows PC)
        self.remote_ip = remote_ip                                          # Stores IP address of the remote device (Jetson)
        self.remote_port = remote_port                                      # Stores port number of the remote device (Jetson)
        self.running = True                                                 # Checks if communicator is running
        
        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))                    # Binds to local address
        self.sock.setblocking(False)                                        # Enables non-blocking mode
        
        # Handler for shutdown (ctrl+c)
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print(f"Voltage Client initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")

    # Cleanup on shutdown    
    def signal_handler(self, signum, frame):
        print("\nStopping Voltage Client...")
        self.running = False                        # Communicator set to false, meaning not running
        if hasattr(self, 'sock'):
            self.sock.close()                       # Closes the socket, essential
        sys.exit(0)                                 # Exits the program

    # Registers with Robot (Yahboom) server
    async def register(self, loop):
        print("Registering with robot...")
        max_attempts = 5
        attempt = 0
        
        while attempt < max_attempts and self.running:
            try:
                # Sends a registration request
                print(f"Registration attempt {attempt + 1}/{max_attempts}")
                self.sock.sendto("REGISTER".encode('utf-8'), (self.remote_ip, self.remote_port))
                
                try:
                    # Waits for confirmation with timeout
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
                await asyncio.sleep(2)                                  # Waits 2 seconds before retrying
        
        print("Registration failed after all attempts")
        return False

    # Display voltage reading with timestamp
    def print_voltage_info(self, voltage_data):
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]      # Hour, minute, second as a float
        print(f"\n=== Voltage Reading at {current_time} ===")
        print(f"Voltage: {voltage_data['voltage']:.2f}V")
        print("=" * 40)

    # Continuous data reception loop
    async def receive_data(self, loop):
        while self.running:
            try:
                # Get and process voltage data
                data = await loop.sock_recv(self.sock, 65535)           # 65535 bytes, for maximum buffer size
                voltage_data = pickle.loads(data)
                self.print_voltage_info(voltage_data)
            except Exception as e:
                if self.running:
                    print(f"Error receiving data: {e}")
                    await asyncio.sleep(1)

async def main():
    # Network config
    local_ip = "0.0.0.0"                # Listens on all interfaces
    local_port = 7534                   # Sending port of local device (Windows PC)
    remote_ip = "100.65.149.245"        # Robot IP, if used with NetBird, put the provided ip address of the Robot/Jetson.
    remote_port = 7533                  # Jetson's listening port (remote device)

    # Setup and run client
    loop = asyncio.get_event_loop()
    client = VoltageClient(local_ip, local_port, remote_ip, remote_port)
    
    if await client.register(loop):
        print("Starting to receive voltage data...")
        print("Press Ctrl+C to stop")
        await client.receive_data(loop)

# Program main Entry point
if __name__ == '__main__':
    try:
        asyncio.run(main())                         # Starts async program
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")