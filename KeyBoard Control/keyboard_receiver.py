#Please red Keyboard_Control_Instructions.txt first, dont uncommnet next line of code
#!/usr/bin/env python
import asyncio                              # For asynchronous operations, allowing multiple processes at the same time
import json                                 # Used for decoding and encoding JSON data
import logging                              # For logging messages, helps with debugging and monitoring
import socket                               # Network communication, TCP or UDP
from typing import Optional, Callable       # This imports type hints from the typing module
import threading                            # Threading is needed for parallel executions
import queue                                # For thread-safe data exchange
import rospy                                # ROS library 
from geometry_msgs.msg import Twist         # Twist is used here for the message type in robot movement commands

logging.basicConfig(level=logging.INFO)     # Essential for logging system to show INFO messages
logger = logging.getLogger(__name__)

class KeyboardReceiver:
    def __init__(self, local_ip: str = "0.0.0.0", local_port: int = 7532):      # Defines the class that will handle our keyboard input over the network
        self.local_ip = local_ip                                                # Stores local IP address (Jetson)
        self.local_port = local_port                                            # Stores local port number (Jetson)
        self.transport = None                                                   # "transport" holds the network transport layer
        self.is_running = False                                                 # Tracks if the receiver is active
        self._socket = None                                                     # "_socket" holds the network socket
        self.key_queue = queue.Queue()                                          # "key_queue" is a thread-safe queue for storing received keyboard events
        self.current_keys = set()                                               # "current_keys" tracks currently pressed keys
        self.last_key = None                                                    # "last_key" stores the most recent key event
        self._start_receiver()                                                  # Calls the _start_receiver method as soon as a new KeyboardReceiver object is created

    def _create_protocol(self):
        class ReceiverProtocol(asyncio.DatagramProtocol):               # Creates an inner class that inherits from DatagramProtocol to handle UDP network communications
            def __init__(self, key_queue):                              # Constructor method, takes a queue that will store our keyboard events
                self.key_queue = key_queue                              # Saves the queue as an instance variable

            def datagram_received(self, data, addr):                    # Handles received UDP datagrams
                try:
                    message = json.loads(data.decode())                 # Decodes the received data bytes as JSON
                    event = message.get('event', {})                    # Extracts the "event" field from the message
                    key = event.get('key', '')                          # Gets the "key" value from the event
                    event_type = event.get('type', '')                  # Gets the event type, for example "up", "down"
                    if event_type == 'down':                            # If this is a key pressed "event", then...
                        self.key_queue.put(key)                         # Adds the key to the queue
                except Exception as e:
                    logger.error(f"Error processing message: {e}")      # Logs any errors that occur during message processing
        return ReceiverProtocol(self.key_queue)                         # Returns a new instance of "ReceiverProtocol" with the current key queue

    def _start_receiver(self):                                                      # Starts the asynchronous receiver
        async def run_receiver():                                                   # Defines an async function to run the receiver
            try:
                loop = asyncio.get_event_loop()                                     # Gets the current event loop
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)             # Creates a UDP socket
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)          # Allows reuse of the address which is useful for restart if a crash happens
                sock.bind((self.local_ip, self.local_port))                         # Binds the socket to the to the assigned IP and port
                
                # Here we create an endpoint for receiving UDP packets
                transport, _ = await loop.create_datagram_endpoint(
                    lambda: self._create_protocol(),
                    sock=sock
                )
                
                # Stores the transport layer and marks receiver as running, so we know its connected
                self.transport = transport
                self.is_running = True
                # To log the successful start of the receiver
                logger.info(f"Keyboard receiver started on {self.local_ip}:{self.local_port}")
                
                # Main receiver loop
                while self.is_running:              # Sleep to prevent CPU overuse, usually a small value
                    await asyncio.sleep(0.001)      # 1ms delay
                    
            except Exception as e:
                logger.error(f"Error in receiver: {e}")     # Logs any errors that occur during receiver operation, helpful for user and debugging
            finally:
                if self.transport:
                    self.transport.close()                  # Makes sure transport is closed when loop ends, this is essential to not leave anything runnning

         # Function to set up and run the asyncio event loop
        def run_asyncio_loop():
            # Create new event loop                     
            loop = asyncio.new_event_loop()

            # To set it as the current event loop         
            asyncio.set_event_loop(loop)

             #Runs the receiver until completion
            loop.run_until_complete(run_receiver())

        # Create and start a thread running the asyncio loop
        self.receiver_thread = threading.Thread(target=run_asyncio_loop, daemon=True)
        self.receiver_thread.start()

    # Function to retrieve a key from the queue with a timeout
    def get_key(self, timeout=0.001):                       # 1ms timeout needed
        """Get the next key from the queue"""
        try:
            key = self.key_queue.get(timeout=timeout)       # Try to get a key from the queue
            return key
        except queue.Empty:
            return ''                                       # Returns empty string, if queue is empty

    def stop(self):                                         # Stops the reciever, can be changed or further developed
        """Stop the receiver"""