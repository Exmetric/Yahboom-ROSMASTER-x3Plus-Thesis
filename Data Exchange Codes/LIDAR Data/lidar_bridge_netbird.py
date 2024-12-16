#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import socket
import pickle
import struct
import time

class LidarPublisher:
    def __init__(self):
        print("Initializing LidarPublisher...")
        
        # Initialize ROS node with more verbose logging
        rospy.init_node('lidar_publisher', anonymous=True, log_level=rospy.DEBUG)
        print("ROS node initialized")
        
        # TCP Server setup
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.server_address = ('100.65.149.245', 12345) #Netbird given IP address of Yahboom
            print(f"Attempting to bind to {self.server_address}")
            self.server_socket.bind(self.server_address)
            self.server_socket.listen(1)
            print("Server socket setup complete")
        except Exception as e:
            print(f"Error setting up server socket: {e}")
            raise

        # Check if /scan topic exists
        print("Checking for /scan topic...")
        topics = rospy.get_published_topics()
        scan_topics = [t for t, t_type in topics if t == '/scan']
        if not scan_topics:
            print("WARNING: /scan topic not found! Available topics are:")
            for topic, topic_type in topics:
                print(f"- {topic} ({topic_type})")
        else:
            print("/scan topic found")

        print("Subscribing to /scan topic...")
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.client_socket = None
        self.message_count = 0
        print("Subscriber setup complete")

    def lidar_callback(self, data):
        self.message_count += 1
        if self.message_count % 100 == 0:  # Print every 100 messages
            print(f"Received {self.message_count} messages from /scan")
            
        if not self.client_socket:
            print("No client connected, waiting for connection...")
            try:
                self.client_socket, client_address = self.server_socket.accept()
                print(f"New client connected from {client_address}")
            except Exception as e:
                print(f"Error accepting connection: {e}")
                return

        try:
            # Print some debug info about the scan data
            if self.message_count % 100 == 0:  # Print every 100 messages
                print(f"Scan data - ranges: {len(data.ranges)}, "
                      f"angle_min: {data.angle_min}, "
                      f"angle_max: {data.angle_max}")

            lidar_data = {
                'angle_min': data.angle_min,
                'angle_max': data.angle_max,
                'angle_increment': data.angle_increment,
                'time_increment': data.time_increment,
                'scan_time': data.scan_time,
                'range_min': data.range_min,
                'range_max': data.range_max,
                'ranges': list(data.ranges),
                'intensities': list(data.intensities) if data.intensities else []
            }
            
            serialized_data = pickle.dumps(lidar_data)
            message_size = struct.pack("!I", len(serialized_data))
            
            self.client_socket.sendall(message_size)
            self.client_socket.sendall(serialized_data)
            
        except Exception as e:
            print(f"Error in lidar_callback: {e}")
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None

    def shutdown(self):
        print("Shutting down...")
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()

if __name__ == '__main__':
    try:
        print("Starting LidarPublisher...")
        publisher = LidarPublisher()
        print("LidarPublisher started. Waiting for data...")
        rospy.spin()
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        try: