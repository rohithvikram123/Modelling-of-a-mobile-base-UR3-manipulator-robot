#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

# Define key codes
LIN_VEL_STEP_SIZE = 1
ANG_VEL_STEP_SIZE = 0.1
LINK_VEL_STEP_SIZE = 0.01

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel=0.0
        steer_angle=0.0
        link1_angle = 0.0
        link2_angle = 0.0
        link3_angle = 0.0
        link4_angle = 0.0
        link5_angle = 0.0
        link6_angle = 0.0


        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_angle=0.0
                    link1_angle = 0.0
                    link2_angle = 0.0
                    link3_angle = 0.0
                    link4_angle = 0.0
                    link5_angle = 0.0
                    link6_angle = 0.0
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE
                elif key == 'r':  # Right
                    link1_angle -= LINK_VEL_STEP_SIZE
                elif key == 'f':  # Left
                    link1_angle += LINK_VEL_STEP_SIZE
                elif key == 't':  # Right
                    link2_angle -= LINK_VEL_STEP_SIZE
                elif key == 'g':  # Left
                    link2_angle += LINK_VEL_STEP_SIZE
                elif key == 'y':  # Right
                    link3_angle -= LINK_VEL_STEP_SIZE
                elif key == 'h':  # Left
                    link3_angle += LINK_VEL_STEP_SIZE
                elif key == 'u':  # Right
                    link4_angle -= LINK_VEL_STEP_SIZE
                elif key == 'j':  # Left
                    link4_angle += LINK_VEL_STEP_SIZE
                elif key == 'i':  # Right
                    link5_angle -= LINK_VEL_STEP_SIZE
                elif key == 'k':  # Left
                    link5_angle += LINK_VEL_STEP_SIZE
                elif key == 'o':  # Right
                    link6_angle -= LINK_VEL_STEP_SIZE
                elif key == 'l':  # Left
                    link6_angle += LINK_VEL_STEP_SIZE


                if steer_angle>1.5 :
                    steer_angle=1.5
                if steer_angle<-1.5:
                    steer_angle=-1.5
        

                print("Steer Angle",steer_angle)
                print("Linear Velocity",linear_vel)
                print("link 1 Angle",link1_angle)
                print("link 2 Angle",link2_angle)
                print("link 3 Angle",link3_angle)
                print("link 4 Angle",link4_angle)
                print("link 5 Angle",link5_angle)
                print("link 6 Angle",link6_angle)
                # Publish the twist message
                wheel_velocities.data = [linear_vel, linear_vel, linear_vel, linear_vel,0.0,0.0,0.0,0.0,0.0,0.0]
                joint_positions.data = [steer_angle, steer_angle,link1_angle,link2_angle,link3_angle,link4_angle,link5_angle,link6_angle]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()