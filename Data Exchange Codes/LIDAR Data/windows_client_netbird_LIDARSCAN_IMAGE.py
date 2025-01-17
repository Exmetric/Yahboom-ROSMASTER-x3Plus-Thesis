# Windows Client Side (Subscriber)
#Please read the LIDAR_Data_Instructions.txt first
#This code works using a TCP/IP connection rather than UDP, there might be a small delay in data, but nothing to complain
import socket                       # Network communication, TCP or UDP                   
import pickle                       
import struct
import matplotlib.pyplot as plt      
import numpy as np
import time
from matplotlib.animation import FuncAnimation

class LidarSubscriber:
    def __init__(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.server_address = ('100.65.149.245', 12345) #Netbird given IP address of Windows client
        
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.line, = self.ax.plot([], [], 'r.')
        self.ax.set_rmax(10)
        self.ax.grid(True)

    def connect(self):
        while True:
            try:
                print("Connecting to server...")
                self.client_socket.connect(self.server_address)
                self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("Connected!")
                break
            except socket.error as e:
                print(f"Connection failed: {e}")
                time.sleep(2)  # Wait before retrying
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def receive_data(self):
        try:
            raw_msgsize = self.client_socket.recv(4)
            if not raw_msgsize:
                return None
            msgsize = struct.unpack('!I', raw_msgsize)[0]
            
            data = b''
            while len(data) < msgsize:
                chunk = self.client_socket.recv(min(msgsize - len(data), 4096))
                if not chunk:
                    return None
                data += chunk
            
            return pickle.loads(data)
        
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None

    def update_plot(self, lidar_data):
        if lidar_data:
            try:
                angles = np.arange(
                    lidar_data['angle_min'],
                    lidar_data['angle_max'] + lidar_data['angle_increment'],
                    lidar_data['angle_increment']
                )[:len(lidar_data['ranges'])]  # Ensure angles match ranges length
                
                ranges = np.array(lidar_data['ranges'])
                valid_idx = np.isfinite(ranges)
                
                if len(angles) != len(ranges):
                    print(f"Warning: angles ({len(angles)}) and ranges ({len(ranges)}) length mismatch")
                    return
                
                self.line.set_xdata(angles[valid_idx])
                self.line.set_ydata(ranges[valid_idx])
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
                
            except Exception as e:
                print(f"Error updating plot: {e}")

    def run(self):
        while True:
            try:
                lidar_data = self.receive_data()
                if lidar_data:
                    self.update_plot(lidar_data)
                time.sleep(0.01)  # Reduced delay
            
            except KeyboardInterrupt:
                print("Stopping client...")
                break
            except Exception as e:
                print(f"Error in main loop: {e}")
                self.connect()  # Try to reconnect
        
        self.client_socket.close()
        plt.close()

if __name__ == '__main__':
    subscriber = LidarSubscriber()
    try:
        subscriber.connect()
        subscriber.run()
    except Exception as e:
        print(f"Error: {e}")