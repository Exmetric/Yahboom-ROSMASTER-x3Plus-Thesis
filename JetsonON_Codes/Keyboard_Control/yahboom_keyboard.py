#This code needs to be replaced with the original yahboom_keyboard.py file as the launch file initiates it.
#This code was made to control both the wheels and the 6 sevo arm with the keyboard
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from yahboomcar_msgs.msg import ArmJoint
from keyboard_receiver import KeyboardReceiver

msg = """
Combined Robot Control
---------------------------
Movement Controls:
   u    i    o   : Forward movement
   j    k    l   : k=force stop, j/l=rotation
   m    ,    .   : Backward movement

Arm Controls:
   1/q: joint1 | 2/w: joint2 | 3/e: joint3
   4/r: joint4 | 5/t: joint5 | 6/y: grip

Speed Controls:
a/z : increase/decrease max speeds by 10%
s/x : increase/decrease only linear speed by 10%
d/c : increase/decrease only angular speed by 10%
g/G : x and y speed switch
h/H : stop movement control
space key : force stop

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),    # Forward
    'o': (1, -1),   # Forward + Right
    'u': (1, 1),    # Forward + Left
    'l': (0, -1),   # Right
    'j': (0, 1),    # Left
    ',': (-1, 0),   # Backward
    '.': (-1, -1),  # Backward + Right
    'm': (-1, 1),   # Backward + Left
}

speedBindings = {
    'a': (1.1, 1.1),
    'z': (.9, .9),
    's': (1.1, 1),
    'x': (.9, 1),
    'd': (1, 1.1),
    'c': (1, .9),
}

class CombinedRobotController:
    def __init__(self):
        rospy.init_node('combined_controller')
        
        # Movement control setup
        self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.linear_limit = rospy.get_param('~linear_speed_limit', 1.0)
        self.angular_limit = rospy.get_param('~angular_speed_limit', 5.0)
        self.speed = 0.2
        self.turn = 1.0
        self.x = 0
        self.th = 0
        self.xspeed_switch = True
        self.stop_movement = False
        
        # Arm control setup
        self.joint_pub = rospy.Publisher('/TargetAngle', ArmJoint, queue_size=10) #Subscribes to the /TargetAngle rostopic
        self.step_size = 2.0 # Can change degree steps to your liking
        self.current_angles = {1: 90.0, 2: 90.0, 3: 90.0, 4: 90.0, 5: 90.0, 6: 30.0}
        
        self.joint_limits = {
            1: (0, 180),    # joint1
            2: (0, 180),    # joint2
            3: (0, 180),    # joint3
            4: (0, 180),    # joint4
            5: (0, 180),    # joint5
            6: (6, 180)     # gripper, 6 degrees minimum is essential to not damage the gripper
        }
        
        self.arm_keys = {
            '1': (1, 1), 'q': (1, -1),  # First Servo motor, increase/decrease
            '2': (2, 1), 'w': (2, -1),  # Second Servo motor, increase/decrease
            '3': (3, 1), 'e': (3, -1),  # Third Servo motor, increase/decrease
            '4': (4, 1), 'r': (4, -1),  # Fourth Servo motor, increase/decrease
            '5': (5, 1), 't': (5, -1),  # Fifth Servo motor, increase/decrease
            '6': (6, 1), 'y': (6, -1)   # Gripper Servo motor, increase/decrease
        }
        
        # Initialize keyboard receiver, the script we made
        self.receiver = KeyboardReceiver()
        
    def vels(self):
        return f"currently:\tspeed {self.speed}\tturn {self.turn}"
        
    def enforce_limits(self, joint_id, angle):
        min_angle, max_angle = self.joint_limits[joint_id]
        return max(min_angle, min(angle, max_angle))
        
    def process_arm_control(self, key):
        if key in self.arm_keys:
            joint_id, direction = self.arm_keys[key]
            self.current_angles[joint_id] += direction * self.step_size
            self.current_angles[joint_id] = self.enforce_limits(joint_id, self.current_angles[joint_id])
            
            msg = ArmJoint()
            msg.id = joint_id
            msg.run_time = 10
            msg.angle = self.current_angles[joint_id]
            msg.joints = []
            
            self.joint_pub.publish(msg)
            print(f"Joint {joint_id}: {msg.angle:.1f}°")
            return True
        return False

    def run(self):
        print(msg)
        print(self.vels())
        
        count = 0
        try:
            while not rospy.is_shutdown():
                key = self.receiver.get_key()
                
                # Check arm controls first
                if self.process_arm_control(key):
                    continue
                
                # Movement controls
                if key == 'k':  # Force stop
                    self.x = 0
                    self.th = 0
                    count = 0
                elif key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                    count = 0
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    count = 0
                    if self.speed > self.linear_limit: self.speed = self.linear_limit
                    if self.turn > self.angular_limit: self.turn = self.angular_limit
                    print(self.vels())
                elif key in ['g', 'G']:
                    self.xspeed_switch = not self.xspeed_switch
                elif key in ['h', 'H']:
                    self.stop_movement = not self.stop_movement
                    print(f"Movement control: {'disabled' if self.stop_movement else 'enabled'}")
                else:
                    count = count + 1
                    if count > 4:
                        self.x = 0
                        self.th = 0
                
                # Publish movement command
                if not self.stop_movement:
                    twist = Twist()
                    if self.xspeed_switch:
                        twist.linear.x = self.speed * self.x
                    else:
                        twist.linear.y = self.speed * self.x
                    twist.angular.z = self.turn * self.th
                    self.move_pub.publish(twist)
                
                rospy.sleep(0.1)  # Small delay to prevent CPU overuse

        except Exception as e:
            print(e)
        finally:
            # Stop everything
            self.move_pub.publish(Twist())
            self.receiver.stop()

if __name__ == '__main__':
    try:
        controller = CombinedRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass