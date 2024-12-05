#!/usr/bin/env python3
import asyncio
import json
import websockets
from datetime import datetime
import sys

class RosBridgeLidarClient:
    def __init__(self, ros_bridge_url='ws://100.65.149.245:9090'):
        self.ros_bridge_url = ros_bridge_url
        self.running = True
        self.previous_seq = None
        
    async def subscribe_to_scan(self):
        while self.running:
            try:
                print(f"Attempting to connect to ROS Bridge at {self.ros_bridge_url}")
                
                async with websockets.connect(
                    self.ros_bridge_url,
                    ping_interval=30,
                    ping_timeout=60,
                    max_size=None
                ) as websocket:
                    print("Connected to ROS Bridge successfully!")
                    
                    # Subscribe to the /scan topic
                    subscribe_msg = {
                        "op": "subscribe",
                        "topic": "/scan",
                        "type": "sensor_msgs/LaserScan"
                    }
                    await websocket.send(json.dumps(subscribe_msg))
                    print(f"Subscribed to /scan topic via ROS Bridge")
                    
                    while self.running:
                        try:
                            message = await asyncio.wait_for(websocket.recv(), timeout=60.0)
                            scan_data = json.loads(message)
                            if 'msg' in scan_data:
                                self.print_scan_info(scan_data['msg'])
                        except asyncio.TimeoutError:
                            print("No message received in 60 seconds, checking connection...")
                            await websocket.ping()
                            continue
                        except websockets.exceptions.ConnectionClosed:
                            print("Connection to ROS Bridge lost. Attempting to reconnect...")
                            break
                        except Exception as e:
                            print(f"Error processing message: {e}")
                            await asyncio.sleep(1)
                            
            except ConnectionRefusedError:
                print(f"Connection refused. Please verify:")
                print("1. ROS Bridge is running on the robot")
                print("2. The IP address is correct")
                print("3. Port 9090 is not blocked by firewall")
                print("4. VPN connection is stable")
                await asyncio.sleep(5)
                
            except Exception as e:
                print(f"Connection error: {e}")
                print("Attempting to reconnect in 5 seconds...")
                await asyncio.sleep(5)

    def print_scan_info(self, scan_data):
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
            
            print("ranges: [")
            for i, range_val in enumerate(scan_data['ranges']):
                if isinstance(range_val, float) and range_val == float('inf'):
                    print("inf", end='')
                else:
                    print(f"{range_val:.8f}", end='')
                if i < len(scan_data['ranges']) - 1:
                    print(", ", end='')
                    if (i + 1) % 10 == 0:
                        print()
                        print("  ", end='')
            print("]")
            
            if scan_data['intensities']:
                print("intensities: [")
                for i, intensity in enumerate(scan_data['intensities']):
                    print(f"{intensity:.8f}", end='')
                    if i < len(scan_data['intensities']) - 1:
                        print(", ", end='')
                        if (i + 1) % 10 == 0:
                            print()
                            print("  ", end='')
                print("]")
            else:
                print("intensities: []")
            
            print("\n" + "=" * 80 + "\n")

async def main():
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else '100.65.149.245'
    ros_bridge_url = f'ws://{robot_ip}:9090'
    
    print(f"Starting ROS Bridge LiDAR client...")
    print(f"Connecting to robot at: {ros_bridge_url}")
    print("Press Ctrl+C to stop")
    
    client = RosBridgeLidarClient(ros_bridge_url)
    try:
        await client.subscribe_to_scan()
    except KeyboardInterrupt:
        client.running = False
        print("\nShutdown requested... exiting")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested... exiting")