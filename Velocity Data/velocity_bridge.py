#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import socket
import pickle
import signal
import sys
import asyncio
import threading

class VelocityNetworkBridge:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        # Initialize ROS node
        rospy.init_node('velocity_network_bridge', anonymous=True)
       
        # Network configuration
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.client_registered = False
        self.running = True
       
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))
        self.sock.setblocking(False)
       
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
       
        # Subscribe to velocity topic
        self.vel_sub = rospy.Subscriber('/vel_raw', Twist, self.velocity_callback, queue_size=1)
       
        print(f"Velocity Bridge initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")
       
    def signal_handler(self, signum, frame):
        print("\nShutting down Velocity Bridge...")
        self.running = False
        if hasattr(self, 'sock'):
            self.sock.close()
        rospy.signal_shutdown("User requested shutdown")
        sys.exit(0)

    async def wait_for_registration(self, loop):
        print("Waiting for client registration...")
        while self.running and not rospy.is_shutdown():
            try:
                data = await loop.sock_recv(self.sock, 1024)
                message = data.decode('utf-8', errors='ignore')
                if message == "REGISTER":
                    self.client_registered = True
                    print("Client registered successfully")
                    # Use direct socket send instead of loop.sock_sendto
                    self.sock.sendto("REGISTERED".encode('utf-8'),
                                   (self.remote_ip, self.remote_port))
                    break
            except Exception as e:
                print(f"Waiting for registration... ({str(e)})")
                await asyncio.sleep(1)

    def velocity_callback(self, vel_data):
        if not self.client_registered or not self.running:
            return
           
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
            data = pickle.dumps(vel_dict)
            self.sock.sendto(data, (self.remote_ip, self.remote_port))
        except Exception as e:
            print(f"Error sending velocity data: {e}")
            self.client_registered = False

    async def run(self, loop):
        await self.wait_for_registration(loop)
        while self.running and not rospy.is_shutdown():
            await asyncio.sleep(0.01)

def run_ros_spin():
    rospy.spin()

async def main():
    local_ip = "0.0.0.0"
    local_port = 7531
    remote_ip = "192.168.50.252"  # The Windows client's IP
    remote_port = 7532

    # Create and get the event loop
    loop = asyncio.get_event_loop()
   
    bridge = VelocityNetworkBridge(local_ip, local_port, remote_ip, remote_port)
   
    # Run ROS spin in a separate thread
    ros_thread = threading.Thread(target=run_ros_spin, daemon=True)
    ros_thread.start()
   
    await bridge.run(loop)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")