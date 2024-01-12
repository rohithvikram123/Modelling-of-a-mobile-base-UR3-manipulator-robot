#!/usr/bin/env python3

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

def H(alpha, t, d, a):
    A = Matrix([[cos(t), -sin(t)*cos(alpha), sin(t)*sin(alpha), a*cos(t)],
                   [sin(t), cos(t)*cos(alpha), -cos(t)*sin(alpha), a*sin(t)],
                   [0, sin(alpha), cos(alpha), d],
                   [0, 0, 0, 1]])
    return A

t1 = Symbol("theta1")
A1 = H(-90*(pi/180), t1, 0.152, 0)

t2 = Symbol("theta2")
A2 = H(0, t2-(90*(pi/180)), 0, 0.244)
# A2 = H(0, t2, 0, 0.6127)
t3 = Symbol("theta3")
A3 = H(0, t3, 0, 0.213)

t4 = Symbol("theta4")
A4 = H(-90*(pi/180), t4-(90*(pi/180)), 0.083, 0)
# A4 = H(-90*(pi/180), t4, 0.1639, 0)
t5 = Symbol("theta5")
A5 = H(90*(pi/180), t5, 0.083, 0)

t6 = Symbol("theta6")
A6 = H(0, t6, 0.092, 0)


Trans_0_1 = A1
Trans_0_2 = A1*A2
Trans_0_3 = A1*A2*A3
Trans_0_4 = A1*A2*A3*A4
Trans_0_5 = A1*A2*A3*A4*A5
Trans_0_6 = A1*A2*A3*A4*A5*A6

# Z value
print("Z1")
Z1 = Matrix([ [Trans_0_1[2]], [Trans_0_1[6]], [Trans_0_1[10]] ])
print(Z1)
print("Z2")
Z2 = Matrix([ [Trans_0_2[2]], [Trans_0_2[6]], [Trans_0_2[10]] ])
print(Z2)
print("Z3")
Z3 = Matrix([ [Trans_0_3[2]], [Trans_0_3[6]], [Trans_0_3[10]] ])
print(Z3)
print("Z4")
Z4 = Matrix([ [Trans_0_4[2]], [Trans_0_4[6]], [Trans_0_4[10]] ])
print(Z4)
print("Z5")
Z5 = Matrix([ [Trans_0_5[2]], [Trans_0_5[6]], [Trans_0_5[10]] ])
print(Z5)
print("Z6")
Z6 = Matrix([ [Trans_0_6[2]], [Trans_0_6[6]], [Trans_0_6[10]] ])
print(Z6)
# Xp
print("Xp")
Xp = Matrix([ [Trans_0_6[3]], [Trans_0_6[7]], [Trans_0_6[11]] ])
print(Xp)
Xp1 = diff(Xp, t1)
Xp2 = diff(Xp, t2)
Xp3 = diff(Xp, t3)
Xp4 = diff(Xp, t4)
Xp5 = diff(Xp, t5)
Xp6 = diff(Xp, t6)

#Calculating the Jacobian for each i value
J1 = Matrix([ [Xp1], [Z1] ])
J2 = Matrix([ [Xp2], [Z2] ])
J3 = Matrix([ [Xp3], [Z3] ])
J4 = Matrix([ [Xp4], [Z4] ])
J5 = Matrix([ [Xp5], [Z5] ])
J6 = Matrix([ [Xp6], [Z6] ])

# Jacobian Matrix

J = Matrix([ [J1, J2, J3, J4, J5, J6] ])

pprint("Jacobian Matrix")
pprint(J)

q1 = 0
q2 = 0
q3 = 0
q4 = 0
q5 = 0
q6 = 0

Three_D_plot = plt.figure(figsize=(4,4))
circle = Three_D_plot.add_subplot(111, projection='3d')

x = []                                          # List for storing X-component of the Xp matrix (T6_F[3])
y = []                                          # List for storing Y-component of the Xp matrix (T6_F[7])
z = []  


Vx = -0.1 * sin(45*(pi/180)) * (2*(pi/200))
Vy = 0
Vz = 0.1 * cos(45*(pi/180)) * (2*(pi/200))
Wx = 0
Wy = 0
Wz = 0

xdot = Matrix([[Vx], [Vy], [Vz], [Wx], [Wy], [Wz]])
J_final = J.evalf(5,subs={t1:q1, t2:q2, t3:q3, t4:q4, t5:q5, t6:q6 })

    
T_final = Trans_0_6.evalf(5,subs={t1:q1, t2:q2, t3:q3, t4:q4, t5:q5, t6:q6 })



J_inverse = J_final.pinv()
q_dot = J_inverse*xdot
    
q1_dot = q_dot[0]
q2_dot = q_dot[1]
q3_dot = q_dot[2]
q4_dot = q_dot[3]
q5_dot = q_dot[4]
q6_dot = q_dot[5]

    

q1 = q1 + (q1_dot)
q2 = q2 + (q2_dot)
q3 = q3 + (q3_dot)
q4 = q4 + (q4_dot)
q5 = q5 + (q5_dot)
q6 = q6 + (q6_dot)
    


q1 = q1.evalf(3)                             # Simplifying the value of q1
q2 = q2.evalf(3)                             # Simplifying the value of q2                             
q3 = q3.evalf(3)                             # Simplifying the value of q4
q4 = q4.evalf(3)                             # Simplifying the value of q5 
q5 = q5.evalf(3)                             # Simplifying the value of q6
q6 = q6.evalf(3)  
    
q_array=[0.0,0.0,float(q1), float(q2), float(q3), float(q4), float(q5), float(q6)]
    # print(q_array)
    # print("Printing q3")
    # print(q3)
   
class Robot_control(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)


    def arm_pose(self,joint_pose):
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        
        
        
        joint_positions.data = joint_pose
        wheel_velocities.data = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
        time.sleep(0.5)
        print(joint_positions.data)

def main(args=None):
    rclpy.init(args=args)

    robot_control = Robot_control()
    robot_control.arm_pose(q_array)
    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
