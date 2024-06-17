#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from time import sleep as sleep
from geometry_msgs.msg import Twist


class SpotMicro(Node):
    
    def __init__(self):
        super().__init__('SpotMicro RVIZ')
        self.get_logger().info("Joint state publisher is online")
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10) #create publisher to post transforms
        self.timer = self.create_timer(0.1, self.publish_joint_states) #look into
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel', self.cmd_vel_callback,10) #create instance of subscribing to cmd_vel/ topic
        self.current_position = 0.0


def main (args=None):
    rclpy.init(args=args)
    spotmicro_rviz = SpotMicro()
    rclpy.spin(spotmicro_rviz)
    spotmicro_rviz.destroy_node
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()