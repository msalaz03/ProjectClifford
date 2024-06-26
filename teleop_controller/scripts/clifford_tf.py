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
#Designed by Matthew Salazar.

class LegJointPublisher(Node):

    def __init__(self):
        super().__init__('leg_mover') 
        self.get_logger().info('Leg Joint Publisher is online.')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10) #create publisher to post transforms
        self.timer = self.create_timer(0.1, self.publish_joint_states) #look into
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel', self.cmd_vel_callback,10) #create instance of subscribing to cmd_vel/ topic
        self.joy_sub = self.create_subscription(Joy,'joy', self.general_joy_callback,10)

        #init our variables this defines the starting position
        #self.current_position_joint_1 = 0.7232 #joint_1
        #self.current_position_joint_2 = -0.0855 #joint_2

        self.current_position_joint_1 = 0.7103
        self.current_position_joint_2 = -0.0866
        #THESE ARE DEFINED VALUES OF LENGTH OF EACH JOINT 
        self.joint_1_len = 107.00
        self.joint_2_len = 130.43


    #A function created in order to add hotkey functionality on the controller.
    def general_joy_callback(self, data):

        #If the triangle button is hit, reset all joints to general stance.
        
        
        if data.buttons[2] == 1:
            self.get_logger().info("Resetting Joints...")
            cords = [36.1,0.0,157.3]

            #self.current_position_joint_1 = 0.7232 #joint_1
            #self.current_position_joint_2 = -0.0855 #joint_2

            theta_joint_1,theta_joint_2 = self.solve_ik(cords)

            self.current_position_joint_1 = theta_joint_1
            self.current_position_joint_2 = theta_joint_2

        #Circle button condition
        if data.buttons[1] == 1:
            self.get_logger().info('WALKING GAIT POS:2')
            cords = [40.9, 0.0, 80.0] #xyz
            
            theta_joint_1,theta_joint_2 = self.solve_ik(cords)

            
            self.current_position_joint_1 = theta_joint_1
            self.current_position_joint_2 = theta_joint_2

        #X button condition
        if data.buttons[0] == 1:
            self.get_logger().info('WALKING GAIT POS:3')
            cords = [80.9, 0.0, 80.0]

            theta_joint_1,theta_joint_2 = self.solve_ik(cords)

            self.current_position_joint_1 = theta_joint_1
            self.current_position_joint_2 = theta_joint_2

        #Square button condition
        if data.buttons[3] == 1:
            self.get_logger().info('WALKING GAIT POS:4')
            cords = [80.9, 0.0, 157.3]
            theta_joint_1,theta_joint_2 = self.solve_ik(cords)

            self.current_position_joint_1 = theta_joint_1
            self.current_position_joint_2 = theta_joint_2

    
    def solve_ik(self,cords):
        # These kinematics calculations will try to be as descripitional as possible but please refer
        # to sheet of calculations by Cameron Bauman. 
        #UNIT: RADIANS
        
        x_cord = cords[0] #x cord value
        y_cord = cords[1] #y cord value
        z_cord = cords[2] #z cord value

        #Find length B using Pythagorean's Theorem
        b_len = math.sqrt( pow(x_cord,2) + pow(z_cord,2) ) 
        
        # Angle of B
        beta_1 = math.atan(z_cord/x_cord)
        
        #Calculations for 'joint_1' and 'joint_2' applied through cosine law.

        #This is the angle of which joint_1 is set. This is necessary for calculating how the long will be SET
        beta_2 = math.acos( ( pow(self.joint_1_len,2) + pow(b_len,2) - pow(self.joint_2_len,2) ) 
                                / (2 * self.joint_1_len * b_len) )

        #This is the angle of which joint_2 is set.
        beta_3 = math.acos( ( pow(self.joint_1_len,2) + pow(self.joint_2_len,2) - pow(b_len,2) ) 
                                / (2 * self.joint_1_len * self.joint_2_len) )
        
        #Shouldn't be too relevant to calculations besides for RVIZ, but this is to make the calculations relative to their axes.
        b_prime = (math.pi  * 2) - beta_1
        theta_2 = b_prime - beta_2
        theta_2 = theta_2 - math.pi
        theta_2 = (math.pi/2) - theta_2  #Final value of joint_1

        theta_3 = math.pi - beta_3
        theta_3 = (math.pi/2) - theta_3 #Final value of joint_2
        
        return [theta_2,theta_3]


    def cmd_vel_callback(self,msg):
        # Scale linear.x to fit within the range of -1.57 to 1.57 joint_2
        scaled_x = max(min(msg.linear.x, 0.2), -0.2) * 0.3 # Adjust the scaling factor as per your requirement

        #scale linear.x to fit within the range of -1.57 to 1.57 joint_2
        scaled_z = max(min(msg.linear.z, 0.2), -0.2) * 0.3 

        self.current_position_joint_1 += scaled_x
        self.current_position_joint_2 += scaled_z
       # self.get_logger().info({f"Scaled Z: {self.current_position_joint_2}"})
        # Clamp current_position within the range of -1.57 to 1.57
        self.current_position_joint_1 = max(min(self.current_position_joint_1, 1.57), -1.57)
        self.current_position_joint_2 = max(min(self.current_position_joint_2, 1.57), -1.57)

        #Publish Joint states
        self.publish_joint_states()

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint_1', 'joint_2']  # Replace with the names of your joints
        joint_state_msg.position = [self.current_position_joint_1, self.current_position_joint_2] #self.current_position_y]
        self.joint_state_pub.publish(joint_state_msg)
      
    #   lower="-1.57"
    #   upper="1.57"
    #   effort="300"
    #   velocity="3" 


def main(args=None):
    rclpy.init(args=args)
    leg_publisher = LegJointPublisher()
    rclpy.spin(leg_publisher)
    leg_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()








#CALCULATIONS FOR KINEMATICS
# b_len = math.sqrt( pow(x_cord,2) + pow(z_cord,2) )
            #angle of joint_1 ik calculation
            # angle_B = math.acos( ( pow(self.joint_1_len,2) + pow(self.desired_height,2) - pow(self.joint_2_len,2) ) 
            #                     / (2 * self.joint_1_len * self.desired_height) )

            # angle_C = math.acos( ( pow(self.joint_1_len,2) + pow(self.joint_2_len,2) - pow(self.desired_height,2) ) 
            #                     / (2 * self.joint_1_len * self.joint_2_len) )
            # beta_1 = math.atan(z_cord/x_cord)
            # beta_2 = math.acos( ( pow(self.joint_1_len,2) + pow(b_len,2) - pow(self.joint_2_len,2) ) 
            #                     / (2 * self.joint_1_len * b_len) )

            # beta_3 = math.acos( ( pow(self.joint_1_len,2) + pow(self.joint_2_len,2) - pow(b_len,2) ) 
            #                     / (2 * self.joint_1_len * self.joint_2_len) )

            # self.get_logger().info(f"beta_1:{beta_1}")
            # self.get_logger().info(f"beta_2:{beta_2}")
            # self.get_logger().info(f"beta_3:{beta_3}")

            # b_prime = (math.pi  * 2) - beta_1
            # theta_2 = b_prime - beta_2
            # theta_2 = theta_2 - math.pi
            # theta_2 = (math.pi/2) - theta_2

            # theta_3 = math.pi - beta_3
            # theta_3 = (math.pi/2) - theta_3