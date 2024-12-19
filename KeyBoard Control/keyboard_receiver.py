#Please red Keyboard_Control_Instructions.txt first, dont uncommnet next line of code
#!/usr/bin/env python
import asyncio
import json
import logging
import socket
from typing import Optional, Callable
import threading
import queue
import rospy
from geometry_msgs.msg import Twist

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class KeyboardReceiver:
    def __init__(self, local_ip: str = "0.0.0.0", local_port: int = 7532):
        self.local_ip = local_ip
        self.local_port = local_port
        self.transport = None
        self.is_running = False
        self._socket = None
        self.key_queue = queue.Queue()
        self.current_keys = set()
        self.last_key = None
        self._start_receiver()

    def _create_protocol(self):
        class ReceiverProtocol(asyncio.DatagramProtocol):
            def __init__(self, key_queue):
                self.key_queue = key_queue

            def datagram_received(self, data, addr):
                try:
                    message = json.loads(data.decode())
                    event = message.get('event', {})
                    key = event.get('key', '')
                    event_type = event.get('type', '')
                    if event_type == 'down':
                        self.key_queue.put(key)
                except Exception as e:
                    logger.error(f"Error processing message: {e}")
        return ReceiverProtocol(self.key_queue)

    def _start_receiver(self):
        async def run_receiver():
            try:
                loop = asyncio.get_event_loop()
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind((self.local_ip, self.local_port))
                
                transport, _ = await loop.create_datagram_endpoint(
                    lambda: self._create_protocol(),
                    sock=sock
                )
                
                self.transport = transport
                self.is_running = True
                logger.info(f"Keyboard receiver started on {self.local_ip}:{self.local_port}")
                
                while self.is_running:
                    await asyncio.sleep(0.001)  # Reduced to 1ms
                    
            except Exception as e:
                logger.error(f"Error in receiver: {e}")
            finally:
                if self.transport:
                    self.transport.close()

        def run_asyncio_loop():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(run_receiver())

        self.receiver_thread = threading.Thread(target=run_asyncio_loop, daemon=True)
        self.receiver_thread.start()

    def get_key(self, timeout=0.001):  # Reduced to 1ms
        """Get the next key from the queue"""
        try:
            key = self.key_queue.get(timeout=timeout)
            return key
        except queue.Empty:
            return ''

    def stop(self):
        """Stop the receiver"""