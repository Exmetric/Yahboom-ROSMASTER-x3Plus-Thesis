#Please red Keyboard_Control_Instructions.txt first

import asyncio                              # For asynchronous operations, allowing multiple processes at the same time
import json
import logging
import socket                               # Network communication, TCP or UDP
import time
from typing import Optional, Callable       
import keyboard 
import threading
import queue

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class UDPCommunicator:
    def __init__(self, local_ip: str, local_port: int, remote_ip: str, remote_port: int):
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.message_callback: Optional[Callable] = None
        self.is_running = False
        self._socket = None
        self.pressed_keys = set()
        self.message_queue = queue.Queue()

    def set_message_callback(self, callback: Callable):
        self.message_callback = callback

    class CommunicationProtocol(asyncio.DatagramProtocol):
        def __init__(self, callback: Optional[Callable] = None):
            self.callback = callback
            self.transport = None

        def connection_made(self, transport):
            self.transport = transport
            logger.info("UDP Connection established")

        def datagram_received(self, data, addr):
            try:
                message = json.loads(data.decode())
                logger.info(f"Received from {addr}: {message}")
                if self.callback:
                    self.callback(message, addr)
            except json.JSONDecodeError:
                logger.error(f"Failed to decode message: {data}")
            except Exception as e:
                logger.error(f"Error processing message: {e}")

    def _create_and_bind_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        max_retries = 5 #max retries before terminating entire code
        retry_delay = 1 #delay in seconds between each retry
        
        for attempt in range(max_retries):
            try:
                sock.bind((self.local_ip, self.local_port))
                logger.info(f"Successfully bound to port {self.local_port}")
                return sock
            except OSError as e:
                if attempt < max_retries - 1:
                    logger.warning(f"Port {self.local_port} is busy, retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    raise
        
        raise OSError(f"Failed to bind to port {self.local_port} after {max_retries} attempts")

    async def start(self):
        try:
            loop = asyncio.get_event_loop()
            self._socket = self._create_and_bind_socket()
            transport, _ = await loop.create_datagram_endpoint(
                lambda: self.CommunicationProtocol(self.message_callback),
                sock=self._socket
            )
            self.transport = transport
            self.is_running = True
            logger.info(f"UDP Communicator started on {self.local_ip}:{self.local_port}")
            
        except Exception as e:
            logger.error(f"Failed to start UDP communication: {e}")
            if self._socket:
                self._socket.close()
            raise

    async def process_message_queue(self):
        """Process messages from the queue"""
        while self.is_running:
            try:
                # Check if there are messages to process
                while not self.message_queue.empty():
                    message = self.message_queue.get_nowait()
                    if self.transport:
                        data = json.dumps(message).encode()
                        self.transport.sendto(data, (self.remote_ip, self.remote_port))
                        logger.info(f"Sent to {self.remote_ip}:{self.remote_port}: {message}")
                    self.message_queue.task_done()
                
                # Small delay to prevent CPU overload
                await asyncio.sleep(0.01)
            except Exception as e:
                logger.error(f"Error processing message queue: {e}")
                await asyncio.sleep(1)  # Longer delay on error

    def queue_message(self, message: dict):
        """Add message to queue for sending"""
        self.message_queue.put(message)

    def stop(self):
        if self.transport:
            self.transport.close()
        if self._socket:
            self._socket.close()
        self.is_running = False
        logger.info("UDP Communicator stopped")

def keyboard_monitor(communicator):
    """Monitor all keyboard inputs and queue updates"""
    def on_key_event(e):
        try:
            key = e.name.lower()
            event_type = e.event_type
            
            # Update pressed keys set
            if event_type == 'down':
                communicator.pressed_keys.add(key)
            elif event_type == 'up':
                communicator.pressed_keys.discard(key)
            
            # Create message with key event and current state
            message = {
                "event": {
                    "key": key,
                    "type": event_type,
                    "scan_code": e.scan_code,
                    "timestamp": time.time()
                },
                "currently_pressed": list(communicator.pressed_keys)
            }
            
            # Add message to queue instead of sending directly
            communicator.queue_message(message)
            
            # Print locally for monitoring
            print(f"Key {event_type}: {key} (scan_code: {e.scan_code})")
            print(f"Currently pressed keys: {communicator.pressed_keys}")
            
        except Exception as err:
            logger.error(f"Error in keyboard monitoring: {err}")

    keyboard.hook(on_key_event)
    print("\nKeyboard monitoring active. Press Ctrl+C to exit.")

async def message_handler(message: dict, addr):
    """Handle incoming messages"""
    logger.info(f"Handling message from {addr}: {message}")

async def main():                   #Our communication parameters
    local_ip = "0.0.0.0"            #Listen on all interfaces
    local_port = 7531               #Sending port of this device
    remote_ip = "100.65.149.245"    #The Jetson's LAN IP, if set up with NetBird, change to the provided ip of the Jetson/Yahboom peer.
    remote_port = 7532              #Listening port of Jetson device

    communicator = UDPCommunicator(local_ip, local_port, remote_ip, remote_port)
    communicator.set_message_callback(message_handler)
    
    try:
        await communicator.start()

        # Start keyboard monitoring in a separate thread
        keyboard_thread = threading.Thread(
            target=keyboard_monitor,
            args=(communicator,),
            daemon=True
        )
        keyboard_thread.start()

        print("\nMonitoring all keyboard inputs...")
        print("Press Ctrl+C to exit")

        # Start the message queue processor
        await asyncio.gather(
            communicator.process_message_queue(),
            # Add other async tasks here if needed
        )

    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        communicator.stop()

if __name__ == "__main__":
    if hasattr(asyncio, 'WindowsSelectorEventLoopPolicy'):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    
    asyncio.run(main()) 