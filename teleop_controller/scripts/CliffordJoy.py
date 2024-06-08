#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose as Pose

class CliffordJoystickControl(Node):

    def __init__(self):
        super().__init__('Clifford_Joystick')
        self.get_logger().info('Controller is online.')

        self.clifford_publish_velocity = self.create_publisher(Twist, 'cmd_vel', 1)
        self.clifford_pose_publisher = self.create_publisher(Pose, 'clifford_pose', 1)
        
        self.joystick_subscriber = self.create_subscription(Joy, 'joy', self.joystick_callback, 1)

        self.declare_parameter('speed', 0.5)
        self.speed = self.get_parameter('speed').value

        self.declare_parameter('turn', 1.0)
        self.turn = self.get_parameter('turn').value

    """
    Table of index number of /joy.buttons:

    Index   Button name on the actual controller

    0       A

    1       B

    2       X

    3       Y

    4       LB

    5       RB

    6       back

    7       start

    8       power

    9       Button stick left

    10      Button stick right

    ==========================================

    Table of index number of /joy.axes:

    Index   Axis name on the actual controller

    0       Left/Right Axis stick left

    1        Up/Down Axis stick left

    2       LT

    3       Left/Right Axis stick right

    4       Up/Down Axis stick right

    5       RT

    6       cross key left/right

    7       cross key up/down 
    
    """
    def joystick_callback(self, data):
        twist = Twist()

        #self.get_logger().info(f'LSSTICK: {data.axes[1]}')
        twist.linear.x = data.axes[1] * self.speed          #left stick up/down
        twist.linear.y = data.axes[4] * self.speed          #right stick up/down
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.clifford_publish_velocity.publish(twist)

        
def main(args=None):
    rclpy.init(args=args)
    joy_stick_control = CliffordJoystickControl()
    rclpy.spin(joy_stick_control) #find out more about this?
    rclpy.shutdown()

if __name__ == '__main__':
    main()
