#Please read LIDAR_Data_Instructions.txt first, dont uncomment next line of code. It is a line for Python 3 interpreter.
#!/usr/bin/env python3
import rospy                                # ROS library 
from sensor_msgs.msg import LaserScan       # Imports LaserScan message type from sensor_msgs package, used for LIDAR data
import socket                               # Network communication, TCP or UDP
import pickle                               # To transport data over the network, and convert data (bytes to Python object)
import struct                               # For packing data into bytes for network transmission
import time                                 # For time related functions


# Define the main LIDAR publisher class
class LidarPublisher:

    # Initializes the LIDAR publisher
    def __init__(self):

        # Prints the initialization message
        print("Initializing LidarPublisher...")
        
        # Initialize ROS node with debug-level logging
        rospy.init_node('lidar_publisher', anonymous=True, log_level=rospy.DEBUG)
        print("ROS node initialized")
        
        # TCP Server setup
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)          # Creates a TCP socket
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)        # Enables address reuse to prevent "Address already in use" errors, essential
            self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)        # Helps reduce latency
            self.server_address = ('100.65.149.245', 12345)                                 # Netbird given IP address of Yahboom, with port
            print(f"Attempting to bind to {self.server_address}")
            self.server_socket.bind(self.server_address)                                    # Binds socket to address
            self.server_socket.listen(1)                                                    # Listens for incoming connections (max 1 connection in queue)
            print("Server socket setup complete")
        except Exception as e:
            print(f"Error setting up server socket: {e}")                                   # Prints error message if socket setup fails
            raise                                                                           # Re-raises the exception (e)

        # Check if /scan topic exists
        print("Checking for /scan topic...")
        topics = rospy.get_published_topics()                                       # Gets list of all published topics
        scan_topics = [t for t, t_type in topics if t == '/scan']                   # Filters for /scan topic
        
        # Prints warning if /scan topic not found, for example if roscore command hasnt been executed, rostopic will not show up
        if not scan_topics:
            print("WARNING: /scan topic not found! Available topics are:")
            for topic, topic_type in topics:
                print(f"- {topic} ({topic_type})")
        else:
            print("/scan topic found")

        print("Subscribing to /scan topic...")
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)      # Subscribse to the /scan topic
        self.client_socket = None                                                       # Initializes client socket as None
        self.message_count = 0                                                          # Initializes message counter
        print("Subscriber setup complete")                                              

    # Callback function for handling LIDAR data
    def lidar_callback(self, data):
        self.message_count += 1                                                         # Sets increment to message counter
        if self.message_count % 100 == 0:                                               # Prints message count every 100 messages
            print(f"Received {self.message_count} messages from /scan")


        # Check if client is connected    
        if not self.client_socket:
            print("No client connected, waiting for connection...")
            try:

                # Accept new client connection
                self.client_socket, client_address = self.server_socket.accept()
                print(f"New client connected from {client_address}")
            except Exception as e:
                print(f"Error accepting connection: {e}")                           # Print errors if any
                return

        try:
            # Print some debug info about the scan data
            if self.message_count % 100 == 0:  # Prints deubg info every 100 messages
                print(f"Scan data - ranges: {len(data.ranges)}, "
                      f"angle_min: {data.angle_min}, "
                      f"angle_max: {data.angle_max}")

            # Create dictionary of LIDAR data
            lidar_data = {
                'angle_min': data.angle_min,                                            # Minimum angle of scan
                'angle_max': data.angle_max,                                            # Maximum angle of scan
                'angle_increment': data.angle_increment,                                # Angular distance between measurements
                'time_increment': data.time_increment,                                  # Time between measurements
                'scan_time': data.scan_time,                                            # Time for complete scan
                'range_min': data.range_min,                                            # Minimum range value
                'range_max': data.range_max,                                            # Maximum range value
                'ranges': list(data.ranges),                                            # List of range measurements
                'intensities': list(data.intensities) if data.intensities else []       # List of intensity values
            }
            
            # Serialize the LIDAR data dictionary
            serialized_data = pickle.dumps(lidar_data)

            # Packs the size of serialized data as a 4-byte integer
            message_size = struct.pack("!I", len(serialized_data))
            
            # Sends the size of the data
            self.client_socket.sendall(message_size)

            # Sends the serialized data
            self.client_socket.sendall(serialized_data)
            
        except Exception as e:
            print(f"Error in lidar_callback: {e}")          # Prints error message if sending fails
            if self.client_socket:

                # Closes client socket on error, essential
                self.client_socket.close()
                self.client_socket = None

    # Method to handle shutdown
    def shutdown(self):
        print("Shutting down...")
        if self.client_socket:
            self.client_socket.close()          # Closes client socket if it exists
        self.server_socket.close()              # Closes server socket

# Main
if __name__ == '__main__':
    try:
        print("Starting LidarPublisher...")
        publisher = LidarPublisher()                                # Creates instance of LidarPublisher
        print("LidarPublisher started. Waiting for data...")
        rospy.spin()                                                # Keeps the node running
    except Exception as e:
        print(f"Error in main: {e}")                                # Prints any errors that occur
    finally:
        try:
#            publisher.shutdown()                                    # Clean shutdown of the publisher
#        except Exception as e:
#            print(f"Error during shutdown: {e}")