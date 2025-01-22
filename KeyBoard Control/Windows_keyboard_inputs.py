#Please red Keyboard_Control_Instructions.txt first

import asyncio                              # For asynchronous operations, allowing multiple processes at the same time
import json                                 # Used for decoding and encoding JSON data
import logging                              # For logging messages, helps with debugging and monitoring
import socket                               # Network communication, TCP or UDP
import time                                 # For time related functions
from typing import Optional, Callable       # This imports type hints from the typing module
import keyboard                             # Library for monitoring / controlling keyboard inputs
import threading                            # Threading is needed for parallel executions
import queue                                # For thread-safe data exchange

logging.basicConfig(level=logging.INFO)     # Configures basic logging with INFO level
logger = logging.getLogger(__name__)        # Creates a logger

class UDPCommunicator:
    
    # Initializes the UDP communication class with the network parameters
    def __init__(self, local_ip: str, local_port: int, remote_ip: str, remote_port: int):
        self.local_ip = local_ip                                                                # Stores local IP address (Windows PC)
        self.local_port = local_port                                                            # Stores local port number (Windows PC)
        self.remote_ip = remote_ip                                                              # Stores IP address of the remote device (Jetson)
        self.remote_port = remote_port                                                          # Stores port number of the remote device (Jetson)
        self.transport: Optional[asyncio.DatagramTransport] = None                              # Initializes transport variable for UDP communication
        self.message_callback: Optional[Callable] = None                                        # Initializes callback function for handling received messages (handler)
        self.is_running = False                                                                 # Checks if communicator is running
        self._socket = None                                                                     # Initializes the socket variable
        self.pressed_keys = set()                                                               # Creates an empty set to store currently pressed keys
        self.message_queue = queue.Queue()                                                      # Creates thread-safe queue for storing messages

    # Sets callback function for handling messages
    def set_message_callback(self, callback: Callable):             
        self.message_callback = callback                    # Stores the callback function

    # Class to handle UDP messaging protocol
    class CommunicationProtocol(asyncio.DatagramProtocol):
        def __init__(self, callback: Optional[Callable] = None):
            self.callback = callback                                    # Stores callback function for handling received data
            self.transport = None                                       # Initializes transport variable

        # Called when UDP connection is made
        def connection_made(self, transport):
            self.transport = transport                      # Stores the transport object
            logger.info("UDP Connection established")       # Logs the successful connection

        # Called when UDP data is received
        def datagram_received(self, data, addr):
            try:
                message = json.loads(data.decode())                     # Converts received bytes to JSON object
                logger.info(f"Received from {addr}: {message}")         # Logs received message
                if self.callback:
                    self.callback(message, addr)                        # Call callback function with message if it exists
            except json.JSONDecodeError:
                logger.error(f"Failed to decode message: {data}")       # Logs JSON decode errors if any
            except Exception as e:
                logger.error(f"Error processing message: {e}")          # Logs other errors (not decode)

    # Create and bind UDP socket
    def _create_and_bind_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)         # Creates UDP socket
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)      # Allows reuse of address
        
        max_retries = 5     # Max retries before terminating entire code
        retry_delay = 1     # Delay in seconds between each retry
        
        # Try to bind socket to port multiple times
        for attempt in range(max_retries):
            try:
                sock.bind((self.local_ip, self.local_port))                         # Binds socket to local IP and port (Windows PC)
                logger.info(f"Successfully bound to port {self.local_port}")        # Logger
                return sock                                                         # Returns socket if binding is successful
            except OSError as e:
                if attempt < max_retries - 1:               # If not last attempt
                    logger.warning(f"Port {self.local_port} is busy, retrying in {retry_delay} seconds...")     
                    time.sleep(retry_delay)                 # Waits before retrying
                else:
                    raise                                   # Raises error if all attempts fail
        
        raise OSError(f"Failed to bind to port {self.local_port} after {max_retries} attempts")     # Error message if all attempts fail

    # Start UDP communication
    async def start(self):
        try:
            loop = asyncio.get_event_loop()                                     # Gets the current event loop for async operations
            self._socket = self._create_and_bind_socket()                       # Creates and binds the socket
            transport, _ = await loop.create_datagram_endpoint(                 # Creates UDP endpoint
                lambda: self.CommunicationProtocol(self.message_callback),      # Creates protocol with callback function
                sock=self._socket                                               # Use our built socket
            )
            self.transport = transport                                                          # Stores transport for later use
            self.is_running = True                                                              # Set running flag to true, that socket is in use
            logger.info(f"UDP Communicator started on {self.local_ip}:{self.local_port}")       # Logs our successful start to the user
            
        except Exception as e:
            logger.error(f"Failed to start UDP communication: {e}")     # Logs any startup errors, helpful for user
            if self._socket:
                self._socket.close()                                    # Cleans up socket if error occurs, to avoid problems when trying again
            raise                                                       # Re-raise function for the exception

            # Process messages waiting in the queue
    async def process_message_queue(self):
        """Process messages from the queue"""
        while self.is_running:                                          # Continue processing while communicator is running       
            try:
                
                # Check if there are messages to process
                while not self.message_queue.empty():               # Process all the messages in queue
                    message = self.message_queue.get_nowait()       # Gets message without waiting
                    if self.transport:
                        data = json.dumps(message).encode()                                         # Converts message to JSON and encodes it to bytes
                        self.transport.sendto(data, (self.remote_ip, self.remote_port))             # Send it to remote device (Jetson)
                        logger.info(f"Sent to {self.remote_ip}:{self.remote_port}: {message}")      # Logs the sent message
                    self.message_queue.task_done()                                                  # Marks the message as processed
                
                # Small delay to prevent CPU overload
                await asyncio.sleep(0.01)
            except Exception as e:
                logger.error(f"Error processing message queue: {e}")        # Log any errors
                await asyncio.sleep(1)                                      # Logger delay error, timeout 1 second

    # Adds a message to the queue
    def queue_message(self, message: dict):
        """Add message to queue for sending"""
        self.message_queue.put(message)             # Adds message to the queue for later processing

    # Stops the UDP communicator
    def stop(self):
        if self.transport:
            self.transport.close()                  # Closes transport if it exists, essential
        if self._socket:
            self._socket.close()                    # Closes socket if it exists, essential
        self.is_running = False                     # Sets running flag to false, because its no longer running
        logger.info("UDP Communicator stopped")     # Logger shutdown message

# Function to monitor our keyboard events
def keyboard_monitor(communicator):
    """Monitor all keyboard inputs and queue updates"""
    def on_key_event(e):                                # Function to handle each keyboard event
        try:
            key = e.name.lower()                        # Gets key name in lowercase
            event_type = e.event_type                   # Gets event type of key, up or down
            
            # Updates the set of currently pressed keys
            if event_type == 'down':
                communicator.pressed_keys.add(key)          # Adds key to set when pressed (down)
            elif event_type == 'up':
                communicator.pressed_keys.discard(key)      # Removes key from set when released (up)
            
            # Create message with key event and current state, helpful information for user
            message = {
                "event": {
                    "key": key,                     # The key that was pressed/released
                    "type": event_type,             # Type of event (down or up)
                    "scan_code": e.scan_code,       # Hardware scan code of the key
                    "timestamp": time.time()        # Time info of when it happened
                },
                "currently_pressed": list(communicator.pressed_keys)        # List of all currently pressed keys
            }
            
            # Add message to queue instead of sending directly
            communicator.queue_message(message)
            
            # Prints locally for monitoring, for user
            print(f"Key {event_type}: {key} (scan_code: {e.scan_code})")
            print(f"Currently pressed keys: {communicator.pressed_keys}")
            
        except Exception as err:
            logger.error(f"Error in keyboard monitoring: {err}")        # Logs any errors to user

    keyboard.hook(on_key_event)                                         # Registers the keyboard event handler
    print("\nKeyboard monitoring active. Press Ctrl+C to exit.")

# Function to handle incoming messages
async def message_handler(message: dict, addr):
    """Handle incoming messages"""
    logger.info(f"Handling message from {addr}: {message}")     # Logs the received messages to user

# Our main function 
async def main():                   # Our communication parameters
    local_ip = "0.0.0.0"            # Listens on all interfaces
    local_port = 7531               # Sending port of this device
    remote_ip = "100.65.149.245"    # The Jetson's LAN IP, if set up with NetBird, change to the provided ip of the Jetson/Yahboom peer.
    remote_port = 7532              # Listening port of Jetson device

    # Creates and configures UDP communicator
    communicator = UDPCommunicator(local_ip, local_port, remote_ip, remote_port)
    communicator.set_message_callback(message_handler)                                  # Sets up message handling
    
    try:
        await communicator.start()                          # Starts the UDP communicator

        # Creates and starts our keyboard monitoriing thread
        keyboard_thread = threading.Thread(
            target=keyboard_monitor,            # Function to run in thread
            args=(communicator,),               # Pass communicator as argument
            daemon=True                         # When set to "True", thread will stop when main program stops
        )
        keyboard_thread.start()                 # Starts the keyboard monitoring thread

        print("\nMonitoring all keyboard inputs...")
        print("Press Ctrl+C to exit")

        # Start the message queue processor
        await asyncio.gather(
            communicator.process_message_queue(),       # Process messages asynchronously
            
            # Add other async tasks here if needed
        )

    except KeyboardInterrupt:
        logger.info("Shutting down...")     # Logs the shutdown on Ctrl+C
    finally:
        communicator.stop()                 # Stops the communicator

if __name__ == "__main__":
    # Configure Windows-specific event loop policy, needed when using python on windows
    if hasattr(asyncio, 'WindowsSelectorEventLoopPolicy'):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    
    asyncio.run(main())     # Runs the main async function