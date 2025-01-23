#Please read Keyboard_Control_Instructions.txt first, dont uncomment net line of code. It is a line for Python 3 interpreter.
#!/usr/bin/env python3
import rospy                                        # ROS library
from geometry_msgs.msg import Twist                 # Twist is used here for the message type in robot movement commands (Yahboom)
from yahboomcar_msgs.msg import ArmJoint            # Imports yahboom's custom message type for arm joint control (rostopic ArmJoint)
from keyboard_receiver import KeyboardReceiver      # Imports our library that we made

# Multi-line string message for control instructions
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

# Dictionary for movement controls
# Format: "key": (linear_velocity, angular_velocity)
moveBindings = {
    'i': (1, 0),    # Forward
    'o': (1, -1),   # Forward + Right
    'u': (1, 1),    # Forward + Left
    'l': (0, -1),   # Right
    'j': (0, 1),    # Left
    ',': (-1, 0),   # Backward
    '.': (-1, -1),  # Backward + Right
    'm': (-1, 1),   # Backward + Left, k key is force stop but mentioned later 
}

# Dictionary for speed adjustment controls
# Format: "key": (linear_speed_multiplier, angular_speed_multiplier)
speedBindings = {
    'a': (1.1, 1.1),    # Increase both speeds by 10% (Capital letters also)
    'z': (.9, .9),      # Decrease both speeds by 10%
    's': (1.1, 1),      # Increase linear speed by 10%
    'x': (.9, 1),       # Decrease linear speed by 10%
    'd': (1, 1.1),      # Increase angular speed by 10%
    'c': (1, .9),       # Decrease angular speed by 10%
}

class CombinedRobotController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('combined_controller')
        
        # Movement control setup (parameters)
        self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)         # Publisher for movement commands 
        self.linear_limit = rospy.get_param('~linear_speed_limit', 1.0)         # Gets max linear speed from ROS parameter
        self.angular_limit = rospy.get_param('~angular_speed_limit', 5.0)       # Gest max angular speed from ROS parameter
        self.speed = 0.2                # Initial linear speed
        self.turn = 1.0                 # Initial angular speed
        self.x = 0                      # Current linear velocity
        self.th = 0                     # Current angular velocity
        self.xspeed_switch = True       # Toggle for x/y axis movement (inverted)
        self.stop_movement = False      # Movement control enable/disable flag
        
        # Arm control setup
        self.joint_pub = rospy.Publisher('/TargetAngle', ArmJoint, queue_size=10)       # Subscribes to the /TargetAngle rostopic
        self.step_size = 2.0                                                            # Can change degree steps to your liking, 2 degrees seems the smoothest
        self.current_angles = {1: 90.0, 2: 80.0, 3: 80.0, 4: 80.0, 5: 90.0, 6: 30.0}    # Starting angles, I used these angles to avoid collisions in beginning  
        
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
        
        # Initializes keyboard receiver, the script I made to be imported
        self.receiver = KeyboardReceiver()

    # Returns current speed settings as a string        
    def vels(self):
        return f"currently:\tspeed {self.speed}\tturn {self.turn}"

    # This ensures that joint angles stay within their limits    
    def enforce_limits(self, joint_id, angle):
        min_angle, max_angle = self.joint_limits[joint_id]      # Gets the minimum and maximum angles for the specified joint
        return max(min_angle, min(angle, max_angle))            # Returns the angle clamped between min and max values

    # Method to process arm control commands    
    def process_arm_control(self, key):                         
        
        # Checks if the pressed key is in the arm control keys
        if key in self.arm_keys:                                
            
            # Get joint ID and movement direction for the pressed key
            joint_id, direction = self.arm_keys[key]

            # Updates the joint angle by adding step size multiplied by direction            
            self.current_angles[joint_id] += direction * self.step_size

            # Ensures the new angle is within joint limits
            self.current_angles[joint_id] = self.enforce_limits(joint_id, self.current_angles[joint_id])
            
            # Create new ArmJoint message
            msg = ArmJoint()
            msg.id = joint_id                                   # Sets the joint ID
            msg.run_time = 10                                   # Sets the time for movement execution
            msg.angle = self.current_angles[joint_id]           # Sets the target angle
            msg.joints = []                                     # Initialize empty joints list (required by message type)
            
            # Publish the arm joint message
            self.joint_pub.publish(msg) 
            print(f"Joint {joint_id}: {msg.angle:.1f}°")        # Prints the current joint angle
            return True                                         # Return True to indicate key was processed
        return False                                            # Return False if key wasn't an arm control key

    # Main control loop method
    def run(self):
        print(msg)              # Prints the control instructions
        print(self.vels())      # Prints the current speed settings
        
        count = 0               # Initialize counter for key timeout

        # Main control loop
        try:
            while not rospy.is_shutdown():              # Continue while ROS is running
                key = self.receiver.get_key()           # Gets the next keyboard input
                
                # Checks arm controls first
                if self.process_arm_control(key):
                    continue                            # Skips movement processing if key was for arm control
                
                # Movement controls
                if key == 'k':          # Checks for Force Stop key "k" which stops all movement
                    self.x = 0          # If key "k" is pressed, linear velocity is set to 0        
                    self.th = 0         # if key "k" is pressed, angular velocity is set to 0
                    count = 0           # Counter set to 0
                
                # Checks for movement control keys
                elif key in moveBindings.keys():

                    # Sets linear and angular velocities based on key
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                    count = 0

                # Checks for speed adjustment keys    
                elif key in speedBindings.keys():

                    # Adjusts speeds based on key
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    count = 0

                    # Enforces speed limits
                    if self.speed > self.linear_limit: self.speed = self.linear_limit
                    if self.turn > self.angular_limit: self.turn = self.angular_limit
                    print(self.vels())                                                  # Prints new speed settings
                
                elif key in ['g', 'G']:
                    self.xspeed_switch = not self.xspeed_switch                         # This function inverses the x and y speed direction by pressing g or G
                
                elif key in ['h', 'H']:
                    self.stop_movement = not self.stop_movement                         # If h or H key is pressed, no longer control the robot even if the movement keys are pressed
                    print(f"Movement control: {'disabled' if self.stop_movement else 'enabled'}")
                
                else:

                    # Increment counter for no recognized key
                    count = count + 1

                    # Stops movement if no key pressed for a while
                    if count > 4:
                        self.x = 0
                        self.th = 0
                
                # Publish movement command, if movement is enabled, if not will not publish
                if not self.stop_movement:
                    twist = Twist()                                 # Creates new Twist message
                    
                    # Sets linear velocity based on movement axis (x/y)
                    if self.xspeed_switch:                          
                        twist.linear.x = self.speed * self.x
                    else:
                        twist.linear.y = self.speed * self.x
                    
                    # Sets angular velocity
                    twist.angular.z = self.turn * self.th           
                    
                    # Publishes movement command
                    self.move_pub.publish(twist)
                
                # Small delay to prevent CPU overuse
                rospy.sleep(0.1)                                    

        # Handles any exceptions
        except Exception as e:
            print(e)                                # Prints error (e)
        finally:
            # Stops everything
            self.move_pub.publish(Twist())          # Ensures the robot stops moving
            self.receiver.stop()                    # Stops the keyboard receiver

# Main
if __name__ == '__main__':
    try:
        controller = CombinedRobotController()      # Creates controller instance
        controller.run()                            # Starts the control loop
    except rospy.ROSInterruptException:
        pass                                        # Handles ROS shutdown