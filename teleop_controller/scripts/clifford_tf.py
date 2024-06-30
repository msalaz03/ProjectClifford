#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from time import sleep as sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math as math

#JointStatePublisher 
#Programmed by Matthew Salazar.

class LegJointPublisher(Node):

    def __init__(self):
        super().__init__('leg_mover') 
        self.get_logger().info('Leg Joint Publisher is online.')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10) #create publisher for 'joint_states'
        self.timer = self.create_timer(0.1, self.publish_joint_states) #look into
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel', self.cmd_vel_callback,10) #create instance of subscribing to cmd_vel/ topic
        self.joy_sub = self.create_subscription(Joy,'joy', self.general_joy_callback,10) #create susbcription for ps4-controller 'joy'

        #DEFINED POSITIONAL VALUES FOR JOINTS
        self.current_position_joint_1 = 0.7103
        self.current_position_joint_2 = -0.0866

        #DEFINED LENGTHS OF JOINTS
        self.joint_1_len = 107.00
        self.joint_2_len = 130.43

        #DEFINED COORDINATES FOR POS1 - POS4
        self.coordinates = [
            [36.1,0.0,157.3],
            [36.1,0.0,80.0],
            [80.9,0.0,80.0],
            [80.9,0.0,157.3]
        ]
        self.gait_walk_index = 0 #this is an index for coordinate system to know what position we are.
        self.target_index = self.gait_walk_index + 1 #this will be where our leg should be ended up.
        self.current_coords = self.coordinates[self.gait_walk_index] #this is a variable to keep track of where we are.


    #A function created in order to add hotkey functionality on the controller.
    def general_joy_callback(self, data):

        #If the triangle button is hit, reset all joints to general stance.
        if data.buttons[2] == 1:
            self.get_logger().info("Resetting Joints...")
            self.reset_walk_gait()
            
        #Circle button condition
        elif data.buttons[1] == 1:
            self.get_logger().info('BUTTON FEATURE 2')
            
        #X button condition
        elif data.buttons[0] == 1:
            self.get_logger().info('BUTTON FEATURE 3')
          
        #Square button condition
        elif data.buttons[3] == 1:
            self.get_logger().info('BUTTON FEATURE 3')


    def cmd_vel_callback(self,msg):
        self.get_logger().info('cmd_vel topic callback')
        walk_speed = abs(msg.linear.x) * 7.0  # define our walking speed
        # Determine direction of movement
        forward = msg.linear.x > 0

        if self.gait_walk_index == 0:
            # WALKING GAIT POS 1
            self.get_logger().info('gait_walk_index hit 1')
            new_z = self.current_coords[2] - walk_speed if forward else self.current_coords[2] + walk_speed

            if (forward and self.current_coords[2] >= self.coordinates[self.target_index][2]) or \
            (not forward and self.current_coords[2] <= self.coordinates[3][2]):
                self.current_coords = [self.current_coords[0], self.current_coords[1], new_z]
                theta_joint_1, theta_joint_2 = self.solve_ik(self.current_coords)
                self.current_position_joint_1 = theta_joint_1
                self.current_position_joint_2 = theta_joint_2
                self.get_logger().info('updated z coordinate')
            else:
                self.get_logger().info('Transitioning gait index')
                #CONDITIONS TO SWITCH OUR INDEXS AND TARGETS
                self.gait_walk_index = 1 if forward else 3
                self.target_index = 2 if forward else 0

        elif self.gait_walk_index == 1:
            # WALKING GAIT POS 2
            self.get_logger().info('gait_walk_index hit 2')
            new_x = self.current_coords[0] + walk_speed if forward else self.current_coords[0] - walk_speed

            if (forward and self.current_coords[0] < self.coordinates[self.target_index][0]) or \
            (not forward and self.current_coords[0] > self.coordinates[0][0]):
                self.current_coords = [new_x, self.current_coords[1], self.current_coords[2]]
                theta_joint_1, theta_joint_2 = self.solve_ik(self.current_coords)
                self.current_position_joint_1 = theta_joint_1
                self.current_position_joint_2 = theta_joint_2

                self.get_logger().info('updated x coordinate')
            else:
                self.get_logger().info('Transitioning gait index')
                self.gait_walk_index = 2 if forward else 0
                self.target_index = 3 if forward else 1
                
        elif self.gait_walk_index == 2:
            # WALKING GAIT POS 3
            self.get_logger().info('gait_walk_index hit 3')
            new_z = self.current_coords[2] + walk_speed if forward else self.current_coords[2] - walk_speed

            if (forward and self.current_coords[2] < self.coordinates[self.target_index][2]) or \
            (not forward and self.current_coords[2] > self.coordinates[1][2]):
                self.current_coords = [self.current_coords[0], self.current_coords[1], new_z]
                theta_joint_1, theta_joint_2 = self.solve_ik(self.current_coords)
                self.current_position_joint_1 = theta_joint_1
                self.current_position_joint_2 = theta_joint_2

                self.get_logger().info('updated z coordinate')
            else:
                self.get_logger().info('Transitioning gait index')
                self.gait_walk_index = 3 if forward else 1
                self.target_index = 0 if forward else 2

        elif self.gait_walk_index == 3:
            # WALKING GAIT POS 4
            self.get_logger().info('gait_walk_index hit 4')
            new_x = self.current_coords[0] - walk_speed if forward else self.current_coords[0] + walk_speed

            if (forward and self.current_coords[0] > self.coordinates[0][0]) or \
            (not forward and self.current_coords[0] < self.coordinates[2][0]):
                self.current_coords = [new_x, self.current_coords[1], self.current_coords[2]]
                theta_joint_1, theta_joint_2 = self.solve_ik(self.current_coords)
                self.current_position_joint_1 = theta_joint_1
                self.current_position_joint_2 = theta_joint_2

                self.get_logger().info('updated x coordinate')
            else:
                self.get_logger().info('Transitioning gait index')
                self.gait_walk_index = 0 if forward else 2
                self.target_index = 1 if forward else 3

        else:
            self.get_logger().info("unexpected condition hit.")

        self.publish_joint_states()

    
    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint_1', 'joint_2']  # Replace with the names of your joints
        joint_state_msg.position = [self.current_position_joint_1, self.current_position_joint_2] #self.current_position_y]
        self.joint_state_pub.publish(joint_state_msg)
      
    def reset_walk_gait(self):
        self.gait_walk_index = 0 #reinit
        self.target_index = self.gait_walk_index + 1 #reinit
        self.current_coords = self.coordinates[self.gait_walk_index] #starting position

        theta_joint_1,theta_joint_2 = self.solve_ik(self.current_coords) #call our ik solver func
        self.current_position_joint_1 = theta_joint_1 #set those positions joint_1
        self.current_position_joint_2 = theta_joint_2 #set those positions joint_2

    def solve_ik(self,cords):
        # These kinematics calculations will try to be as descripitional as possible but please refer
        # to sheet of calculations by Cameron Bauman. 
        #UNIT: RADIANS
        
        x_cord = cords[0] #x cord value
        y_cord = cords[1] #y cord value not really relevant rn.
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

def main(args=None):
    rclpy.init(args=args)
    leg_publisher = LegJointPublisher()
    rclpy.spin(leg_publisher)
    leg_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
