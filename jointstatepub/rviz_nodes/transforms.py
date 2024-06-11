#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from time import sleep as sleep
from geometry_msgs.msg import Twist

#Create a JointStatePublisher
class LegJointPublisher(Node):

    def __init__(self):
        super().__init__('leg_mover') 
        self.get_logger().info('Leg Joint Publisher is online.')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10) #create publisher to post transforms
        self.timer = self.create_timer(0.1, self.publish_joint_states) #look into
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel', self.cmd_vel_callback,10) #create instance of subscribing to cmd_vel/ topic
        self.current_position = 0.0

    def cmd_vel_callback(self,msg):
        # Scale linear.x to fit within the range of -1.57 to 1.57
        #scaled_x = max(min(msg.linear.x, 1.0), -1.0) * 1.57
        scaled_x = max(min(msg.linear.x, 0.2), -0.2) * 0.8  # Adjust the scaling factor as per your requirement

        self.current_position += scaled_x
    
        # Clamp current_position within the range of -1.57 to 1.57
        self.current_position = max(min(self.current_position, 1.57), -1.57)
    
        #Publish Joint states
        self.publish_joint_states()

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint_1', 'joint_2']  # Replace with the names of your joints
        joint_state_msg.position = [self.current_position, 0.0]
        self.joint_state_pub.publish(joint_state_msg)
        
        #joint_state_msg.position = [0.0, 1.0, 0.0]  # Set the positions of your joints here
        # i = -1.57
        # while (i < 1.40):
        #     joint_state_msg.position = [i,0.0]
        #     #joint_state_msg.velocity = [0.0, 0.0, 0.0]  # Set the velocities of your joints here
        #     #joint_state_msg.effort = [0.0, 0.0, 0.0]  # Set the efforts of your joints here

        #     self.joint_state_pub.publish(joint_state_msg)
        #     i = i + 0.1
        #     sleep(0.5)
       
        #now we are going to create this dog moving off the controller at certain speeds
        
        #TO DOOOOOO
        #some calculation on how to move it.
        #a max and min condition to not overload and I guess like a reset button.
       
       # self.joint_state_pub.publish(joint_state_msg)


    #   lower="-1.57"
    #   upper="1.57"
    #   effort="300"
    #   velocity="3" />

def main(args=None):
    rclpy.init(args=args)
    leg_publisher = LegJointPublisher()
    rclpy.spin(leg_publisher)
    leg_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
