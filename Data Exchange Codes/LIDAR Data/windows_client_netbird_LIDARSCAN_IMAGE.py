# Windows Client Side (Subscriber)
#Please read the LIDAR_Data_Instructions.txt first
#This code works using a TCP/IP connection rather than UDP, there might be a small delay in data, but nothing to complain

import socket                                       # Network communication, TCP or UDP                   
import pickle                                       # To transport data over the network, and convert data (bytes to Python object)
import struct                                       # For packing data into bytes for network transmission
import matplotlib.pyplot as plt                     # Library for creating plots and visualizations      
import numpy as np                                  # Numerical computing library for efficient array operations
import time                                         # For time related functions
from matplotlib.animation import FuncAnimation      # Enables creation of animated plots, for our polar coord lidar view


# Define a class to handle LIDAR data subscription and visualization
class LidarSubscriber:      
    def __init__(self):                                                                 # Initializes the class
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)          # Creates a TCP socket
        self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)        # Disables Nagle's algorithm to reduce latency
        self.server_address = ('100.65.149.245', 12345)                                 # Netbird given IP address of Windows client, and server port
        
        # Setup matplotlib for interactive plotting
        plt.ion()                                                                   
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})        # Creates a figure with polar projection for LIDAR data
        self.line, = self.ax.plot([], [], 'r.')                                     # Creates an empty line plot with red dots
        self.ax.set_rmax(10)                                                        # Sets maximum radius for the polar plot
        self.ax.grid(True)                                                          # Enables grid lines

# Establish connection with server
    def connect(self):
        while True:
            try:
                print("Connecting to server...")
                self.client_socket.connect(self.server_address)                                 # Attempts to connect to the server
                self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)        # Disables Nagle's algorithm again after connection
                print("Connected!")
                break
            except socket.error as e:
                print(f"Connection failed: {e}")
                time.sleep(2)                                                               # Wait before retrying, set to 2 seconds
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)      # Creates new socket for retry

    # Receive LIDAR data from server
    def receive_data(self):
        try:

            # Receive message size (4 bytes) first
            raw_msgsize = self.client_socket.recv(4)                                
            if not raw_msgsize:
                return None
            
            # Unpack the size from network bytes to integer value
            msgsize = struct.unpack('!I', raw_msgsize)[0]
            
            # Initializse empty byte string for data
            data = b''

            # Receive data in chunks until complete message is received
            while len(data) < msgsize:
                chunk = self.client_socket.recv(min(msgsize - len(data), 4096))
                if not chunk:
                    return None
                data += chunk
            
            # Convert received bytes back to Python object
            return pickle.loads(data)
        
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None

    # Update visualization with new data
    def update_plot(self, lidar_data):
        if lidar_data:
            try:

                # Create array of angles for LIDAR measurements, like the lidar rostopic on Yahboom
                angles = np.arange(
                    lidar_data['angle_min'],
                    lidar_data['angle_max'] + lidar_data['angle_increment'],
                    lidar_data['angle_increment']
                )[:len(lidar_data['ranges'])]                                   # Ensure angles match ranges length
                
                # Converts ranges to numpy array
                ranges = np.array(lidar_data['ranges'])

                # Creates mask for valid (finite) range values, essential
                valid_idx = np.isfinite(ranges)
                
                # Check for data consistency
                if len(angles) != len(ranges):
                    print(f"Warning: angles ({len(angles)}) and ranges ({len(ranges)}) length mismatch")
                    return
                
                # Update plot with new data points
                self.line.set_xdata(angles[valid_idx])
                self.line.set_ydata(ranges[valid_idx])
                
                # Redraw the plot, for continious updated points on graph
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
                
            except Exception as e:
                print(f"Error updating plot: {e}")

    # Main loop method
    def run(self):
        while True:
            try:
                # Receive new LIDAR data
                lidar_data = self.receive_data()
                if lidar_data:
                    # Update visualization if data received
                    self.update_plot(lidar_data)
                time.sleep(0.01)                        # Small delay to prevent CPU overuse
            
            except KeyboardInterrupt:
                print("Stopping client...")
                break
            except Exception as e:
                print(f"Error in main loop: {e}")
                self.connect()                          # Try to reconnect if connection is lost for whatever reason
        
        # Clean up resources, essential
        self.client_socket.close()
        plt.close()

# Main execution
if __name__ == '__main__':
    # Create instance of LidarSubscriber
    subscriber = LidarSubscriber()
    try:
        # Connect to server and start main loop
        subscriber.connect()
        subscriber.run()
    except Exception as e:
        print(f"Error: {e}")