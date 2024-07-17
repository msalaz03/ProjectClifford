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
        self.clifford_joy_sub = self.create_subscription( Joy, 'joy', self.clifford_joystick_callback)

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
        self.servo0.set_pulse_width_range( self.get_parameter('min_pulse'),
                                      self.get_parameter('max_pulse') )
        self.servo1.set_pulse_width_range( self.get_parameter('min_pulse'),
                                      self.get_parameter('max_pulse') )

    def clifford_joystick_callback(self, data):
        self.get_logger().info('Clifford Joystick Callback')

        # X button condition
        if data.buttons[0] == 1:
            self.get_logger().info('X Pressed...')

        # Circle button condition
        elif data.buttons[1] == 1:
            self.get_logger().info("Circle Pressed...")

        # Triangle button condition
        elif data.buttons[2] == 1:
            self.get_logger().info("Triangle Pressed...")
          
        # Square button condition
        elif data.buttons[3] == 1:
            self.get_logger().info('Sqaure Pressed...')
            
    
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


def main(args=None):
    rclpy.init(args=args)
    driveServos = ServoDriver()
    rclpy.spin(driveServos)
    driveServos.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
