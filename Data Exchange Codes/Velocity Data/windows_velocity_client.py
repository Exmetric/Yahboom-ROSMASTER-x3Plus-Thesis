import asyncio
import socket
import pickle
import signal
import sys
from datetime import datetime

class VelocityClient:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.running = True
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))
        self.sock.setblocking(False)
        
        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print(f"Velocity Client initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")
        
    def signal_handler(self, signum, frame):
        print("\nStopping Velocity Client...")
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
                
                # Wait for confirmation with timeout
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

    def print_velocity_info(self, vel_data):
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"\n=== Velocity Data at {current_time} ===")
        print("Linear Velocity:")
        print(f"  X: {vel_data['linear']['x']:8.3f} m/s")
        print(f"  Y: {vel_data['linear']['y']:8.3f} m/s")
        print(f"  Z: {vel_data['linear']['z']:8.3f} m/s")
        
        print("\nAngular Velocity:")
        print(f"  X: {vel_data['angular']['x']:8.3f} rad/s")
        print(f"  Y: {vel_data['angular']['y']:8.3f} rad/s")
        print(f"  Z: {vel_data['angular']['z']:8.3f} rad/s")
        
        speed = (vel_data['linear']['x']**2 + 
                vel_data['linear']['y']**2 + 
                vel_data['linear']['z']**2)**0.5
        
        angular_speed = (vel_data['angular']['x']**2 + 
                        vel_data['angular']['y']**2 + 
                        vel_data['angular']['z']**2)**0.5
        
        print(f"\nTotal Speed: {speed:8.3f} m/s")
        print(f"Total Angular Speed: {angular_speed:8.3f} rad/s")
        print("=" * 40)

    async def receive_data(self, loop):
        while self.running:
            try:
                data = await loop.sock_recv(self.sock, 65535)
                vel_data = pickle.loads(data)
                self.print_velocity_info(vel_data)
            except Exception as e:
                if self.running:
                    print(f"Error receiving data: {e}")
                    await asyncio.sleep(1)

async def main():
    local_ip = "0.0.0.0"
    local_port = 7532
    remote_ip = "100.65.149.245"  # The robot's IP, if set up with NetBird, put in the provided ip adress for the Jetson/robot.
    remote_port = 7531

    # Create and get the event loop
    loop = asyncio.get_event_loop()
    
    client = VelocityClient(local_ip, local_port, remote_ip, remote_port)
    
    if await client.register(loop):
        print("Starting to receive velocity data...")
        print("Press Ctrl+C to stop")
        await client.receive_data(loop)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")