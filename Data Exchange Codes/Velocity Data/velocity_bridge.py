# Please make sure you have read the Velocity_Data_Instructions.txt first, and dont uncomment the next line of code.
#!/usr/bin/env python3
import rospy                            # ROS library
from geometry_msgs.msg import Twist     # ROS message type for velocity commands
import socket                           # Network communication, TCP or UDP                         
import pickle                           # To transport data over the network, and convert data (bytes to Python object)
import signal                           # Handler, like when pressing Ctrl + c
import sys                              # Needed for system specific parameters and functions
import asyncio                          # For asynchronous operations, allowing multiple processes at the same time 
import threading                        # Threading is needed for parallel executions

class VelocityNetworkBridge:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        # Initialize ROS node
        rospy.init_node('velocity_network_bridge', anonymous=True)
       
        # Network configuration parameters
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
       
        # Setups the signal handlers, for shutdown (ctrl + c)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
       
        # Subscribe to the ROS velocity topic on the Jetson (rostopic list)
        self.vel_sub = rospy.Subscriber('/vel_raw', Twist, self.velocity_callback, queue_size=1)
       
        print(f"Velocity Bridge initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")

     # Handles shutdown signals   
    def signal_handler(self, signum, frame):
        print("\nShutting down Velocity Bridge...")
        self.running = False
        if hasattr(self, 'sock'):
            self.sock.close()
        rospy.signal_shutdown("User requested shutdown")
        sys.exit(0)

    # Waits for client to register before sending data
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
                    
                    # Sends confirmation to the client (Windows PC)
                    self.sock.sendto("REGISTERED".encode('utf-8'),
                                   (self.remote_ip, self.remote_port))
                    break
            except Exception as e:
                print(f"Waiting for registration... ({str(e)})")
                await asyncio.sleep(1)

    # Handles incoming velocity messages from ROS
    def velocity_callback(self, vel_data):
        if not self.client_registered or not self.running:
            return

        # Converts velocity data to dictionary, like our ROS topic   
        vel_dict = {
            'linear': {
                'x': vel_data.linear.x,
                'y': vel_data.linear.y,
                'z': vel_data.linear.z
            },
            'angular': {
                'x': vel_data.angular.x,
                'y': vel_data.angular.y,
                'z': vel_data.angular.z
            },
            'timestamp': rospy.get_time()
        }
       
        try:
            # Send serialized velocity data to client
            data = pickle.dumps(vel_dict)
            self.sock.sendto(data, (self.remote_ip, self.remote_port))
        except Exception as e:
            print(f"Error sending velocity data: {e}")
            self.client_registered = False

    async def run(self, loop):
        # Main execution loop
        await self.wait_for_registration(loop)
        while self.running and not rospy.is_shutdown():
            await asyncio.sleep(0.01)

# Function to run ROS event loop
def run_ros_spin():
    rospy.spin()

async def main():
    # Network Config
    local_ip = "0.0.0.0"            # Listens on all interfaces
    local_port = 7531               # Sending port of this device (Jetson)
    remote_ip = "100.65.62.108"     # The Windows client's IP, if using netbird, netbird provided windows client ip
    remote_port = 7532              # Listening port of remote device (Windows PC)

    # Create and get the event loop (async)
    loop = asyncio.get_event_loop()
   
   # Create bridge 
    bridge = VelocityNetworkBridge(local_ip, local_port, remote_ip, remote_port)
   
    # Run ROS spin in a separate thread
    ros_thread = threading.Thread(target=run_ros_spin, daemon=True)
    ros_thread.start()
   
   # Run bridge
    await bridge.run(loop)

# Main entry point
if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")