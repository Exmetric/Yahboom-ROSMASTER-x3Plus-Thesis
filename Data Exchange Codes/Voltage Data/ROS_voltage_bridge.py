# Make sure to read Voltage_Data_Instruction.txt first, and dont uncomment next line of code.
#!/usr/bin/env python3

import rospy                        # ROS library
from std_msgs.msg import Float32    # Voltage is published as Float32, single-precision floating point
import socket                       # Network communication, TCP or UDP
import pickle                       # To transport data over the network, and convert data (bytes to Python object)
import signal                       # Handler, like when pressing Ctrl + c
import sys                          # Needed for system specific parameters and functions
import asyncio                      # For asynchronous operations, allowing multiple processes at the same time
import threading                    # Threading is needed for parallel executions

class VoltageNetworkBridge:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        # Initialize ROS node
        rospy.init_node('voltage_network_bridge', anonymous=True)

        # Network Config parameters
        self.local_ip = local_ip            # Stores local IP address (Jetson)
        self.local_port = local_port        # Stores local port number (Jetson)
        self.remote_ip = remote_ip          # Stores IP address of the remote device (Windows PC)
        self.remote_port = remote_port      # Stores port number of the remote device (Windows PC)
        self.client_registered = False      # Tracks client registration status
        self.running = True                 # Checks if communicator is running
        
        # Creates the UDP socket and binds to the local address
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))                    # Binds to local address
        self.sock.setblocking(False)                                        # Enables non-blocking mode
        
        # Register signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)                   # Handle for Ctrl+C
        signal.signal(signal.SIGTERM, self.signal_handler)                  # Handles termination signal
        
        # Subscribe to ROS voltage topic
        self.voltage_sub = rospy.Subscriber('/voltage', Float32, self.voltage_callback, queue_size=1)
        
        print(f"Voltage Bridge initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")

    # Handler for shutdown (ctrl+c)    
    def signal_handler(self, signum, frame):
        print("\nShutting down Voltage Bridge...")
        self.running = False
        if hasattr(self, 'sock'):
            self.sock.close()                               # Closes network socket
        rospy.signal_shutdown("User requested shutdown")    # Shutsdown the ROS node
        sys.exit(0)                                         # Exits the program

    # Wait for client registration
    async def wait_for_registration(self, loop):
        print("Waiting for client registration...")
        while self.running and not rospy.is_shutdown():
            try:
                # Receive registration message
                data = await loop.sock_recv(self.sock, 1024)
                message = data.decode('utf-8', errors='ignore')
                if message == "REGISTER":
                    self.client_registered = True
                    print("Client registered successfully")
                    # Sends confirmation
                    self.sock.sendto("REGISTERED".encode('utf-8'),
                                   (self.remote_ip, self.remote_port))
                    break
            except Exception as e:
                print(f"Waiting for registration... ({str(e)})")
                await asyncio.sleep(1)

    # Handle incoming voltage data from ROS
    def voltage_callback(self, voltage_data):
        if not self.client_registered or not self.running:
            return

        # Create data dictionary with voltage and timestamp    
        data_dict = {
            'voltage': voltage_data.data,           # Current voltage reading
            'timestamp': rospy.get_time()           # ROS time
        }
        
        try:
            # Serialize and send data to the client
            data = pickle.dumps(data_dict)
            self.sock.sendto(data, (self.remote_ip, self.remote_port))
        except Exception as e:
            print(f"Error sending voltage data: {e}")
            self.client_registered = False                              # Reset registration on error

    async def run(self, loop):
        await self.wait_for_registration(loop)
        while self.running and not rospy.is_shutdown():
            await asyncio.sleep(0.01)                                   # Small delay to prevent CPU overuse

# Function to run ROS event loop
def run_ros_spin():
    rospy.spin()

async def main():
    # Network Config
    local_ip = "0.0.0.0"                # Listen on all interfaces
    local_port = 7533                   # Sending port of local device (Jetson)
    remote_ip = "192.168.50.252"        # Windows client IP, use Netbird given address if using vpn
    remote_port = 7534                  # Listening port of remote device (Windows PC)

    # Setup and run bridge
    loop = asyncio.get_event_loop()
    bridge = VoltageNetworkBridge(local_ip, local_port, remote_ip, remote_port)
    
    # Start ROS spin in separate thread
    ros_thread = threading.Thread(target=run_ros_spin, daemon=True)
    ros_thread.start()
    
    # Run bridge
    await bridge.run(loop)

# Program main entry point
if __name__ == '__main__':
    try:
        asyncio.run(main())                         # Starts async program
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")