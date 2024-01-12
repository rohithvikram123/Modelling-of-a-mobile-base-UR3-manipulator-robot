import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time

from sympy import *
from sympy import pprint as pp
import numpy as np
import matplotlib.pyplot as plt


class Robot_control(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)


    def arm_pose(self):
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        
        
        
        joint_positions.data = []
        wheel_velocities.data = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
        time.sleep(0.5)
        print(joint_positions.data)

def main(args=None):
    rclpy.init(args=args)

    robot_control = Robot_control()
    robot_control.arm_pose()
    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()