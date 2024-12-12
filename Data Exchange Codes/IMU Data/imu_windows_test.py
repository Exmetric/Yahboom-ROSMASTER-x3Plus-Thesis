import asyncio
import socket
import pickle
import signal
import sys
from datetime import datetime

class IMUClient:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.running = True
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))
        self.sock.setblocking(False)
        
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print(f"IMU Client initialized on {local_ip}:{local_port}")
        print(f"Remote endpoint set to {remote_ip}:{remote_port}")
        
    def signal_handler(self, signum, frame):
        print("\nStopping IMU Client...")
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

    def print_imu_data(self, imu_data):
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"\n=== IMU Reading at {current_time} ===")
        print(f"Accelerometer (g):")
        print(f"  X: {imu_data['accel'][0]:.2f}")
        print(f"  Y: {imu_data['accel'][1]:.2f}")
        print(f"  Z: {imu_data['accel'][2]:.2f}")
        print(f"Gyroscope (deg/s):")
        print(f"  X: {imu_data['gyro'][0]:.2f}")
        print(f"  Y: {imu_data['gyro'][1]:.2f}")
        print(f"  Z: {imu_data['gyro'][2]:.2f}")
        print("=" * 40)

    async def receive_data(self, loop):
        while self.running:
            try:
                data = await loop.sock_recv(self.sock, 65535)
                imu_data = pickle.loads(data)
                self.print_imu_data(imu_data)
            except Exception as e:
                if self.running:
                    print(f"Error receiving data: {e}")
                    await asyncio.sleep(1)

async def main():
    local_ip = "0.0.0.0"
    local_port = 7541  # Wiindows client receiving port
    remote_ip = "100.65.149.245"  # Robot IP, if using netbird vpn, use ip adress provided of robot
    remote_port = 7540  # Robot/Jetson's listening port

    loop = asyncio.get_event_loop()
    client = IMUClient(local_ip, local_port, remote_ip, remote_port)
    
    if await client.register(loop):
        print("Starting to receive IMU data...")
        print("Press Ctrl+C to stop")
        await client.receive_data(loop)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")