#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32  # Assuming voltage is published as Float32
import socket
import pickle
import signal
import sys
import asyncio
import threading

class VoltageNetworkBridge:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        rospy.init_node('voltage_network_bridge', anonymous=True)
        
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.client_registered = False
        self.running = True
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))
        self.sock.setblocking(False)
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.voltage_sub = rospy.Subscriber('/voltage', Float32, self.voltage_callback, queue_size=1)
        
        print(f"Voltage Bridge initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")
        
    def signal_handler(self, signum, frame):
        print("\nShutting down Voltage Bridge...")
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
                    self.sock.sendto("REGISTERED".encode('utf-8'),
                                   (self.remote_ip, self.remote_port))
                    break
            except Exception as e:
                print(f"Waiting for registration... ({str(e)})")
                await asyncio.sleep(1)

    def voltage_callback(self, voltage_data):
        if not self.client_registered or not self.running:
            return
            
        data_dict = {
            'voltage': voltage_data.data,
            'timestamp': rospy.get_time()
        }
        
        try:
            data = pickle.dumps(data_dict)
            self.sock.sendto(data, (self.remote_ip, self.remote_port))
        except Exception as e:
            print(f"Error sending voltage data: {e}")
            self.client_registered = False

    async def run(self, loop):
        await self.wait_for_registration(loop)
        while self.running and not rospy.is_shutdown():
            await asyncio.sleep(0.01)

def run_ros_spin():
    rospy.spin()

async def main():
    local_ip = "0.0.0.0"
    local_port = 7533  # Different port from velocity
    remote_ip = "192.168.50.252"  # Windows client IP
    remote_port = 7534

    loop = asyncio.get_event_loop()
    bridge = VoltageNetworkBridge(local_ip, local_port, remote_ip, remote_port)
    
    ros_thread = threading.Thread(target=run_ros_spin, daemon=True)
    ros_thread.start()
    
    await bridge.run(loop)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")