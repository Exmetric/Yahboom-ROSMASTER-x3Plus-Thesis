import asyncio
import socket
import pickle
import signal
import sys
from datetime import datetime

class LidarClient:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.running = True
        self.previous_seq = None
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))
        self.sock.setblocking(False)
        
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print(f"LiDAR Client initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")
        
    def signal_handler(self, signum, frame):
        print("\nStopping LiDAR Client...")
        self.running = False
        if hasattr(self, 'sock'):
            self.sock.close()
        sys.exit(0)

    async def register(self, loop):
        print("Registering with robot...")
        max_attempts = 5
        attempt = 0
        
        while attempt < max_attempts and self.running:
            try:
                print(f"Registration attempt {attempt + 1}/{max_attempts}")
                self.sock.sendto("REGISTER".encode('utf-8'), (self.remote_ip, self.remote_port))
                
                try:
                    data = await asyncio.wait_for(
                        loop.sock_recv(self.sock, 1024),
                        timeout=5.0
                    )
                    message = data.decode('utf-8', errors='ignore')
                    if message == "REGISTERED":
                        print("Successfully registered with robot")
                        return True
                except asyncio.TimeoutError:
                    print("Registration timeout, retrying...")
                
            except Exception as e:
                print(f"Registration attempt failed: {e}")
            
            attempt += 1
            if attempt < max_attempts:
                await asyncio.sleep(2)
        
        print("Registration failed after all attempts")
        return False

    def print_scan_info(self, scan_data):
        # Only print if sequence number has changed
        if self.previous_seq != scan_data['header']['seq']:
            self.previous_seq = scan_data['header']['seq']
            
            print("\nheader: ")
            print(f"  seq: {scan_data['header']['seq']}")
            print("  stamp: ")
            print(f"    secs: {scan_data['header']['stamp']['secs']}")
            print(f"    nsecs: {scan_data['header']['stamp']['nsecs']}")
            print(f"  frame_id: '{scan_data['header']['frame_id']}'")
            
            print(f"angle_min: {scan_data['angle_min']}")
            print(f"angle_max: {scan_data['angle_max']}")
            print(f"angle_increment: {scan_data['angle_increment']}")
            print(f"time_increment: {scan_data['time_increment']}")
            print(f"scan_time: {scan_data['scan_time']}")
            print(f"range_min: {scan_data['range_min']}")
            print(f"range_max: {scan_data['range_max']}")
            
            print("ranges: [", end='')
            ranges = [f"{x:.8f}" for x in scan_data['ranges'][:5]]  # Show first 5 ranges
            print(f"{', '.join(ranges)}...")
            print(f"  ... {len(scan_data['ranges'])} values in total]")
            
            if scan_data['intensities']:
                print("intensities: [", end='')
                intensities = [f"{x:.8f}" for x in scan_data['intensities'][:5]]  # Show first 5 intensities
                print(f"{', '.join(intensities)}...")
                print(f"  ... {len(scan_data['intensities'])} values in total]")
            else:
                print("intensities: []")
            
            print("\n" + "=" * 80 + "\n")

    async def receive_data(self, loop):
        while self.running:
            try:
                data = await loop.sock_recv(self.sock, 65535)
                scan_data = pickle.loads(data)
                self.print_scan_info(scan_data)
            except Exception as e:
                if self.running:
                    print(f"Error receiving data: {e}")
                    await asyncio.sleep(1)

async def main():
    local_ip = "0.0.0.0"
    local_port = 7536 # listening port
    remote_ip = "100.65.149.245"  # Robot IP, with netbird ip adress
    remote_port = 7535 #sending port

    loop = asyncio.get_event_loop()
    client = LidarClient(local_ip, local_port, remote_ip, remote_port)
    
    if await client.register(loop):
        print("Starting to receive LiDAR data...")
        print("Press Ctrl+C to stop")
        await client.receive_data(loop)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")