import asyncio
import socket
import pickle
import signal
import sys
import zlib
import numpy as np
from collections import defaultdict

class LidarClient:
    def __init__(self, local_ip, local_port, remote_ip, remote_port):
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.running = True
        self.previous_seq = None
        self.chunk_buffers = {}
        self.current_scan_info = None
        
        print(f"Initializing LiDAR client...")
        print(f"Local: {local_ip}:{local_port}")
        print(f"Remote: {remote_ip}:{remote_port}")
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))
        self.sock.setblocking(False)
        
        # Increase receive buffer size
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        
        signal.signal(signal.SIGINT, self.signal_handler)
        print("LiDAR Client initialized and ready")

    def signal_handler(self, signum, frame):
        print("\nStopping LiDAR Client...")
        self.running = False
        if hasattr(self, 'sock'):
            self.sock.close()
        sys.exit(0)

    def decompress_data(self, compressed_data):
        try:
            print(f"Decompressing data of size: {len(compressed_data)} bytes")
            decompressed = zlib.decompress(compressed_data)
            print(f"Decompressed to size: {len(decompressed)} bytes")
            return pickle.loads(decompressed)
        except Exception as e:
            print(f"Error in decompress_data: {type(e).__name__}: {str(e)}")
            raise

    def process_chunks(self, chunk_info):
        try:
            # Handle header packet
            if chunk_info.get('type') == 'header':
                seq = chunk_info['seq']
                print(f"\nReceived header for sequence {seq}")
                print(f"Expecting {chunk_info['total_chunks']} chunks, total size: {chunk_info['total_size']} bytes")
                
                self.chunk_buffers[seq] = {
                    'total_chunks': chunk_info['total_chunks'],
                    'total_size': chunk_info['total_size'],
                    'received_chunks': {},
                    'timestamp': asyncio.get_event_loop().time()
                }
                return None
                
            # Handle data chunk
            seq = chunk_info['seq']
            if seq not in self.chunk_buffers:
                print(f"Received chunk for unknown sequence {seq}")
                return None
                
            chunk_index = chunk_info['chunk_index']
            self.chunk_buffers[seq]['received_chunks'][chunk_index] = chunk_info['data']
            
            buffer_info = self.chunk_buffers[seq]
            received_count = len(buffer_info['received_chunks'])
            total_chunks = buffer_info['total_chunks']
            
            print(f"Received chunk {chunk_index + 1}/{total_chunks} for sequence {seq}")
            
            if received_count == total_chunks:
                print(f"\nAll chunks received for sequence {seq}")
                # Reconstruct the complete data
                sorted_chunks = [buffer_info['received_chunks'][i] 
                               for i in range(total_chunks)]
                complete_data = b''.join(sorted_chunks)
                
                print(f"Reassembled data size: {len(complete_data)} bytes")
                
                # Clean up the buffer
                del self.chunk_buffers[seq]
                
                # Clean old incomplete buffers
                current_time = asyncio.get_event_loop().time()
                old_seqs = [seq for seq, info in self.chunk_buffers.items() 
                           if current_time - info['timestamp'] > 5.0]
                for old_seq in old_seqs:
                    print(f"Cleaning up old sequence {old_seq}")
                    del self.chunk_buffers[old_seq]
                
                # Decompress and return
                return self.decompress_data(complete_data)
                
        except Exception as e:
            print(f"Error processing chunks: {type(e).__name__}: {str(e)}")
            if seq in self.chunk_buffers:
                del self.chunk_buffers[seq]
        return None

    def print_scan_info(self, scan_data):
        try:
            if self.previous_seq != scan_data['header']['seq']:
                self.previous_seq = scan_data['header']['seq']
                
                print("\n" + "="*40 + " LIDAR SCAN " + "="*40)
                print(f"Sequence: {scan_data['header']['seq']}")
                print(f"Timestamp: {scan_data['header']['stamp']['secs']}.{scan_data['header']['stamp']['nsecs']}")
                print(f"Frame ID: {scan_data['header']['frame_id']}")
                print(f"Angle range: {scan_data['angle_min']:.2f} to {scan_data['angle_max']:.2f} radians")
                print(f"Number of points: {len(scan_data['ranges'])}")
                
                # Print some statistics about the ranges
                ranges = scan_data['ranges']
                valid_ranges = [r for r in ranges if not np.isinf(r) and not np.isnan(r)]
                if valid_ranges:
                    print("\nRange Statistics:")
                    print(f"  Total points: {len(ranges)}")
                    print(f"  Valid points: {len(valid_ranges)}")
                    print(f"  Min distance: {min(valid_ranges):.3f}m")
                    print(f"  Max distance: {max(valid_ranges):.3f}m")
                    print(f"  Average distance: {sum(valid_ranges)/len(valid_ranges):.3f}m")
                
                if scan_data.get('intensities'):
                    print(f"\nIntensity data available: {len(scan_data['intensities'])} points")
                
                print("=" * 89 + "\n")
        except Exception as e:
            print(f"Error printing scan info: {type(e).__name__}: {str(e)}")

    async def receive_data(self, loop):
        print("\nStarting to receive data...")
        while self.running:
            try:
                print("\nWaiting for next packet...")
                data = await loop.sock_recv(self.sock, 65535)
                print(f"Received packet of size: {len(data)} bytes")
                
                chunk_info = pickle.loads(data)
                print(f"Successfully unpickled packet data")
                
                # Process received chunk
                scan_data = self.process_chunks(chunk_info)
                if scan_data:
                    print("Successfully processed complete scan")
                    self.print_scan_info(scan_data)
                    
            except Exception as e:
                if self.running:
                    print(f"Error in receive_data: {type(e).__name__}: {str(e)}")
                    await asyncio.sleep(1)

    async def register(self, loop):
        print("\nStarting registration process...")
        max_attempts = 5
        attempt = 0
        
        while attempt < max_attempts and self.running:
            try:
                print(f"\nRegistration attempt {attempt + 1}/{max_attempts}")
                print(f"Sending REGISTER to {self.remote_ip}:{self.remote_port}")
                self.sock.sendto("REGISTER".encode('utf-8'), (self.remote_ip, self.remote_port))
                
                try:
                    print("Waiting for registration response...")
                    data = await asyncio.wait_for(
                        loop.sock_recv(self.sock, 1024),
                        timeout=5.0
                    )
                    message = data.decode('utf-8', errors='ignore')
                    if message == "REGISTERED":
                        print("Successfully registered with robot")
                        return True
                except asyncio.TimeoutError:
                    print("Registration timeout")
                
            except Exception as e:
                print(f"Registration attempt failed: {type(e).__name__}: {str(e)}")
            
            attempt += 1
            if attempt < max_attempts:
                print("Waiting before next attempt...")
                await asyncio.sleep(2)
        
        print("Registration failed after all attempts")
        return False

async def main():
    # Configuration
    local_ip = "0.0.0.0"
    local_port = 7536
    remote_ip = "100.65.149.245"  # Jetson's IP address (VPN IP)
    remote_port = 7535

    print("\nLiDAR Client Starting")
    print(f"Local IP: {local_ip}:{local_port}")
    print(f"Remote IP: {remote_ip}:{remote_port}")

    loop = asyncio.get_event_loop()
    client = LidarClient(local_ip, local_port, remote_ip, remote_port)
    
    if await client.register(loop):
        print("\nRegistration successful, starting data reception...")
        print("Press Ctrl+C to stop")
        await client.receive_data(loop)
    else:
        print("\nFailed to register with the robot")
        sys.exit(1)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")