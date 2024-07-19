#!/usr/bin/env python

#ROS IMPORTS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

#HARDWARE IMPORTS
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import RPi.GPIO as GPIO

#IMPORT KINEMATIC LIBRARIES
import math as math

#GENERAL IMPORTS
from time import sleep as sleep

class ServoDriver(Node):
    
    #   TO DO
    #   1. Figure out Sudo Privileges for this script.
    #   2. Create Joystick Functionality (4 Buttons) 
    #   3. Create Seperate Kinematics Library
    #   4. Add Dependecies in Package.xml
    #   5. Create Launch File
    #   6. Create Helper Functions

    def __init__(self):
        super().__init__('ServoDriver')
        self.get_logger().info('Clifford Servo Driver Online...')

        #Declare our Publishers & Subscribers
        self.clifford_joy_sub = self.create_subscription( Joy, 'joy', self.clifford_joystick_callback,1)

        #ServoDriver Parameters
        self.declare_parameter('min_pulse', 600) # As specified per Datasheet.
        self.declare_parameter('max_pulse', 2400)

        #INIT OUR HARDWARE
        i2c = board.I2C() # Init the I2C bus interface
        pca = PCA9685(i2c) # Create an instance of PCA9685
        pca.frequency = 60 # Set the PWM frequency to 60Hz
        
        #INIT SERVO DUTY
        for i in range (12):
            pca.channels[i].duty_cycle = 0x7FFF #50% Duty Cycle

        #INIT CORRESPONDING CHANNELS
        self.servo0 = servo.Servo(pca.channels[0])
        self.servo1 = servo.Servo(pca.channels[1])

        #INIT PULSE PARAMETERS
        self.servo0.set_pulse_width_range( 600,2400 )
        self.servo1.set_pulse_width_range( 600,2400 )

        #INIT SERVO RELATIVE COODS + MISC
        self.universal_shoulder_len = 58.17
        self.universal_arm_len = 107.00
        self.universal_wrist_len = 130.43

        #self.test_coords =  [36.1,0.0,157.3]
        self.ref_coords= [130.43, 0.0, 107.0]
        

        self.ref_coords_test =  [150.43,0.0, 107.0]



    def clifford_joystick_callback(self, data):
        #self.get_logger().info('Clifford Joystick Callback')

        # X button condition
        if data.buttons[0] == 1:
            self.get_logger().info('X Pressed...')
            self.zero_servos()

        # Circle button condition
        # elif data.buttons[1] == 1:
        #     self.get_logger().info("Circle Pressed...")
        #     self.test_servos()

        #Triangle button condition
        elif data.buttons[2] == 1:
            self.get_logger().info("Triangle Pressed...")
            theta_2, theta_3 = self.solve_ik(self.ref_coords_test)


            #HARD CODED VALUE TO ADJUST LEFT WRISTS (RIGHT NOT APPLICABLE)
            theta_3 = 90 - (theta_3 - 90)

            self.get_logger().info(f'THETA_2: {theta_2}')
            self.get_logger().info(f'THETA_3: {theta_3}')

            #self.servo0.angle = theta_2
            #self.servo1.angle = theta_3

          
        # Square button condition
        elif data.buttons[3] == 1:
            self.get_logger().info('Sqaure Pressed...')
            
    def zero_servos(self):
        self.get_logger().info("Helper Function: 'zero_servos' called")
        sleep(1)
        #self.servo0.angle = 0
        sleep(2)
        self.servo0.angle = 90
        self.servo1.angle = 90


    # Helper function to test servo range of motion (FINISH IMPLEMENTATION ONCE REST ARE INSTALLED).
    def test_servos(self): 
        self.get_logger().info("Helper Function: 'test_servos' called.")
        sleep(1)
        self.servo0.angle = 45 # Move to 45 degrees (technically -45)
        sleep(1)
        self.servo0.angle = 135 #Move to 135 degrees (technically +45)
        sleep(1)
        self.servo0.angle = 90 # Move to 90 degrees (technically 0)
    
    #DEFINE NEW FUNC RESET SERVOS

    def solve_ik(self,cords):
            # These kinematics calculations will try to be as descripitional as possible but please refer
            # to sheet of calculations by Cameron Bauman. 
            #UNIT: RADIANS & mm
            x_cord = cords[0] #x cord value
            y_cord = cords[1] #y cord value not really relevant rn.
            z_cord = cords[2] #z cord value

            #Find length B using Pythagorean's Theorem
            b_len = math.sqrt( pow(x_cord,2) + pow(z_cord,2) ) 
            
            # Angle of B
            beta_1 = math.atan(z_cord/x_cord)
            
            #Calculations for 'right_arm' and 'right_wrist' applied through cosine law.

            #This is the angle of which right_arm is set. This is necessary for calculating how the long will be SET
            beta_2 = math.acos( ( pow(self.universal_arm_len,2) + pow(b_len,2) - pow(self.universal_wrist_len,2) ) 
                                    / (2 * self.universal_arm_len * b_len) )

            #This is the angle of which right_wrist is set.
            beta_3 = math.acos( ( pow(self.universal_arm_len,2) + pow(self.universal_wrist_len,2) - pow(b_len,2) ) 
                                    / (2 * self.universal_arm_len * self.universal_wrist_len) )
            
            #Shouldn't be too relevant to calculations besides for RVIZ, but this is to make the calculations relative to their axes.
            b_prime = (math.pi  * 2) - beta_1
            theta_2 = b_prime - beta_2
            theta_2 = theta_2 - math.pi

            #theta_2 = (math.pi * 2 ) - (beta_1 + beta_2)


            #THIS VALUE IS HARD CODED
            #theta_2 = (math.pi/4) - theta_2  #Final value of right_arm 

            #self.get_logger().info(f'BETA_3: {beta_3}')
            theta_3 = math.pi - beta_3

            #FINAL VALUE FOR RVIZ
            #theta_3 = (math.pi/2) - theta_3 #Final value of right_wrist
            #theta_3 = (math.pi/2) + beta_3

            theta_2 = theta_2 * (180/math.pi)
            theta_3 = theta_3 * (180/math.pi)

            self.get_logger().info(f"THETA_3 before math corrects: {theta_3}")
            
            return [theta_2,theta_3]

    




def main(args=None):
    rclpy.init(args=args)
    driveServos = ServoDriver()
    rclpy.spin(driveServos)
    driveServos.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
