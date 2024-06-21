#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from time import sleep as sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math as math

#Create a JointStatePublisher
class LegJointPublisher(Node):

    #definition of global variables    
    #JOINT_1_LEN = 100.0 #mm
    #JOINT_2_LEN = 70.0 #mm
    #A,B,C,a,b,c = 0.0

    def __init__(self):
        super().__init__('leg_mover') 
        self.get_logger().info('Leg Joint Publisher is online.')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10) #create publisher to post transforms
        self.timer = self.create_timer(0.1, self.publish_joint_states) #look into
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel', self.cmd_vel_callback,10) #create instance of subscribing to cmd_vel/ topic
        self.joy_sub = self.create_subscription(Joy,'joy', self.reset_joints,10)

        #init our variables this defines the starting position
        self.current_position_x = 0.7232 #joint_1
        self.current_position_z = -0.0855 #joint_2

        self.joint_1_len = 107.00
        self.joint_2_len = 130.43
        self.desired_height = 90

        #To do Define min and max for self.desired_height

        #default position notes
        #joint_1: 0.868
        #joint_2:0.0
        #define joint lengths
        #define joint lengths #2

    #To Code:
    #   Create reset button for base starting position
    #   Incorporate kinematics.

    #reset rviz joints as needed
    def reset_joints(self, data):
        angle_B = 0.0
        angle_C = 0.0

        #self.get_logger().info("in reset joints")
        if data.buttons[2] == 1:
            self.get_logger().info("condition hit")
            self.current_position_x = 0.7232 #joint_1
            self.current_position_z = -0.0855 #joint_2


        if data.buttons[1] == 1:
            z_cord = 0.08 * 1000
            x_cord = 0.0809 * 1000
            b_len = math.sqrt( pow(x_cord,2) + pow(z_cord,2))
            #angle of joint_1 ik calculation
            # angle_B = math.acos( ( pow(self.joint_1_len,2) + pow(self.desired_height,2) - pow(self.joint_2_len,2) ) 
            #                     / (2 * self.joint_1_len * self.desired_height) )

            # angle_C = math.acos( ( pow(self.joint_1_len,2) + pow(self.joint_2_len,2) - pow(self.desired_height,2) ) 
            #                     / (2 * self.joint_1_len * self.joint_2_len) )
            beta_1 = math.atan(z_cord/x_cord)
            beta_2 = math.acos( ( pow(self.joint_1_len,2) + pow(b_len,2) - pow(self.joint_2_len,2) ) 
                                / (2 * self.joint_1_len * b_len) )

            beta_3 = math.acos( ( pow(self.joint_1_len,2) + pow(self.joint_2_len,2) - pow(b_len,2) ) 
                                / (2 * self.joint_1_len * self.joint_2_len) )

            self.get_logger().info(f"beta_1:{beta_1}")
            self.get_logger().info(f"beta_2:{beta_2}")
            self.get_logger().info(f"beta_3:{beta_3}")

            b_prime = (math.pi  * 2)- beta_1
            theta_2 = b_prime - beta_2
            theta_2 = theta_2 - math.pi
            theta_2 = (math.pi/2) - theta_2

            theta_3 = math.pi - beta_3
            theta_3 = (math.pi/2) - theta_3
            
            self.get_logger().info(f"theta_2:{theta_2}")
            self.get_logger().info(f"theta_3: {theta_3}")

            #angle_B = 0.9623 #45
            #angle_B = 0.6084
            #angle_C = 0.0626

            self.get_logger().info(f"angle_B: {angle_B}")
            self.get_logger().info(f"angle_C: {angle_C}")

            self.current_position_x = theta_2
            self.current_position_z = theta_3


        #self.publish_joint_states()

    #def ik(self):
    #    print("hello")


    def cmd_vel_callback(self,msg):
       # self.get_logger().info("test")
        # Scale linear.x to fit within the range of -1.57 to 1.57 joint_2
        scaled_x = max(min(msg.linear.x, 0.2), -0.2) * 0.3 # Adjust the scaling factor as per your requirement

        #scale linear.x to fit within the range of -1.57 to 1.57 joint_2
        scaled_z = max(min(msg.linear.z, 0.2), -0.2) * 0.3 

        self.current_position_x += scaled_x
        self.current_position_z += scaled_z
       # self.get_logger().info({f"Scaled Z: {self.current_position_z}"})
        # Clamp current_position within the range of -1.57 to 1.57
        self.current_position_x = max(min(self.current_position_x, 1.57), -1.57)
        self.current_position_z = max(min(self.current_position_z, 1.57), -1.57)

        #Plan for walking
        #

        #Publish Joint states
        self.publish_joint_states()

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint_1', 'joint_2']  # Replace with the names of your joints
        joint_state_msg.position = [self.current_position_x, self.current_position_z] #self.current_position_y]
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
