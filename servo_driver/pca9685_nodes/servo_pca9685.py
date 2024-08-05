#!/usr/bin/env python

#ROS IMPORTS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

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
        self.clifford_joy_sub = self.create_subscription( Joy, 'joy', self.clifford_joystick_callback,10)
        #self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel',self.cmd_vel_callback,10)

        #INIT OUR HARDWARE
        i2c = board.I2C() # Init the I2C bus interface
        pca = PCA9685(i2c, address=0x40) # Create an instance of PCA9685
        pca.frequency = 60 # Set the PWM frequency to 60Hz

        
       
        
        #INIT CORRESPONDING CHANNELS
        self.front_left_shoulder = servo.Servo(pca.channels[12])
        self.front_left_arm = servo.Servo(pca.channels[13])
        self.front_left_wrist = servo.Servo(pca.channels[14])

        self.front_right_shoulder = servo.Servo(pca.channels[8])
        self.front_right_arm = servo.Servo(pca.channels[9])
        self.front_right_wrist = servo.Servo(pca.channels[10])

        self.back_left_shoulder = servo.Servo(pca.channels[4])
        self.back_left_arm = servo.Servo(pca.channels[5])
        self.back_left_wrist = servo.Servo(pca.channels[6])

        self.back_right_shoulder = servo.Servo(pca.channels[0])
        self.back_right_arm = servo.Servo(pca.channels[1])
        self.back_right_wrist = servo.Servo(pca.channels[2])

        #INIT PULSE PARAMETERS
        self.front_left_shoulder.set_pulse_width_range( 600,2400 )
        self.front_left_arm.set_pulse_width_range( 600,2400 )
        self.front_left_wrist.set_pulse_width_range( 600,2400 )

        self.front_right_shoulder.set_pulse_width_range( 600,2400 )
        self.front_right_arm.set_pulse_width_range( 600,2400 )
        self.front_right_wrist.set_pulse_width_range( 600,2400 )

        self.back_left_shoulder.set_pulse_width_range( 600,2400 )
        self.back_left_arm.set_pulse_width_range( 600,2400 )
        self.back_left_wrist.set_pulse_width_range( 600,2400 )

        self.back_right_shoulder.set_pulse_width_range( 600,2400 )
        self.back_right_arm.set_pulse_width_range( 600,2400 )
        self.back_right_wrist.set_pulse_width_range( 600,2400 )

        #INIT SERVO DUTY
        pca.channels[0].duty_cycle = 0x7FFF #50% Duty Cycle
        pca.channels[1].duty_cycle = 0x7FFF 
        pca.channels[2].duty_cycle = 0x7FFF 

        pca.channels[4].duty_cycle = 0x7FFF 
        pca.channels[5].duty_cycle = 0x7FFF 
        pca.channels[6].duty_cycle = 0x7FFF 
         
        pca.channels[8].duty_cycle = 0x7FFF 
        pca.channels[9].duty_cycle = 0x7FFF 
        pca.channels[10].duty_cycle = 0x7FFF 
        
        pca.channels[12].duty_cycle = 0x7FFF 
        pca.channels[13].duty_cycle = 0x7FFF 
        pca.channels[14].duty_cycle = 0x7FFF 
        
        #INIT SERVO RELATIVE COODS + MISC
        self.universal_shoulder_len = 58.17
        self.universal_arm_len = 107.00
        self.universal_wrist_len = 130.43

        #EXTEND 8.11m
        #INIT OUR SERVOS TO CORRECT POSITIONS
        #self.init_servos()
        self.front_left_shoulder.angle = 87
        self.front_left_arm.angle = 85+45
        self.front_left_wrist.angle = 107

        self.back_right_shoulder.angle = 105
        self.back_right_arm.angle = 55
        self.back_right_wrist.angle = 95

        self.back_left_shoulder.angle = 100
        self.back_left_arm.angle = 77 + 45
        self.back_left_wrist.angle = 107

        self.front_right_shoulder.angle = 100
        self.front_right_arm.angle = 58
        self.front_right_wrist.angle = 95

        

        #INIT OUR SERVO COORDINATE SYSTEM (TO DO)
        self.coordinates_front_left = [
            [13.43,58.17,172.0],
            [3.43,58.17,172.0],
            [3.43,58.17,162.0],
            [13.43,58.17,162.0]
        ]
        
        self.coordinates_front_right = [
            [49.43,58.17,150.0],
            [49.53,58.17,140.0],
            [59.43,58.17,140.0],
            [59.43,58.17,150.0]
        ]

        self.coordinates_back_left = [
            [15.43,58.17,172.0],
            [15.43,58.17,162.0],
            [25.43,58.17,162.0],
            [25.43,58.17,172.0]
        ]

        self.coordinates_back_right = [
            [52.43,58.17,154.0],
            [42.43,58.17,154.0],
            [42.43,58.17,144.0],
            [52.43,58.17,144.0]
        ]

        
        #FLAGS FOR CLIFFORD DIFFERENT MODES DIFFERENT MODES
        self.idle_mode = 0
        self.walk_mode = 1
       
        #TESTING VARIABLES FOR SINGLE LEG MOTION (07/24/24) / FRONT RIGHT
        self.speed_param = 4.0
        self.gait_walk_index = 0
        self.target_index = 1
        
        #GAIT VARIABLES
        self.set1_walk_index = 0
        self.set2_walk_index = 0
        self.set1_target_index = 1
        self.set2_target_index = 1

        self.set1_current_coords = self.coordinates_front_right[0]
        self.set2_current_coords = self.coordinates_front_left[0]


        self.front_left_current_coords = self.coordinates_front_left[0]
        self.front_right_current_coords = self.coordinates_front_right[0]
        self.back_left_current_coords = self.coordinates_back_left[0]
        self.back_right_current_coords = self.coordinates_back_right[0]

        self.set1_coordinates = self.coordinates_front_right

        self.set2_coordinates = self.coordinates_front_left
        
        #HARD VARIABLES FOR FIXING OFFSET OF LEFT.
        self.front_left_hard_arm = -5
        self.front_left_hard_wrist = 17

        self.back_left_hard_arm = -13
        self.back_left_hard_wrist = 17
 

    def clifford_joystick_callback(self, data):
        #self.get_logger().info('Clifford Joystick Callback')
        
        
        """
        back left: [15.43 - 10.0, 58.17,176.0 + 3.5]
        back right: [37.43 - 10.0 , 58.17, 154.0 + 3.5]
        """

        # X button condition
        if data.buttons[0] == 1:
            self.get_logger().info('X Pressed...')
            #[15.43,58.17,172.0],
            self.back_left_arm_angle, self.back_left_wrist_angle = self.solve_ik_left( [15.43 - 10.0,58.17,176.0 + 3.5])
            self.back_left_arm.angle =  self.back_left_arm_angle + self.back_left_hard_arm
            self.back_left_wrist.angle = self.back_left_wrist_angle + self.back_left_hard_wrist
            self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_right([37.43 - 10.0,58.17,154.0 + 3.5])
            #self.walk_mode = 1
            #self.idle_mode = 0
            #sleep(2)
            
            #self.init_servos()

        # Circle button condition
        elif data.buttons[1] == 1:
            self.get_logger().info("Circle Pressed...")
            # #sleep(2
            # self.back_left_arm_theta, self.back_left_wrist_theta = self.solve_ik_left([25.43,58.17,172.0])
            # self.back_left_arm.angle = self.back_left_arm_theta + 5
            # self.back_left_wrist.angle = self.back_left_wrist_theta + 17
            # self.get_logger().info(f'arm angle: {self.back_left_arm.angle}')
            # self.get_logger().info(f'wrist angle: {self.back_left_wrist.angle}')

        #Triangle button condition
        elif data.buttons[2] == 1:
            self.get_logger().info("Triangle Pressed...")
            #sleep(2)
            # self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right([109.43,58.17,100.0])
            # self.get_logger().info(f'arm angle: {self.front_right_arm.angle}')
            # self.get_logger().info(f'wrist angle: {self.front_right_wrist.angle}')
          
        # Square button condition
        elif data.buttons[3] == 1:
            self.get_logger().info('Sqaure Pressed...')
            # self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right([109.43,58.17,150.0])
            # self.get_logger().info(f'arm angle: {self.front_right_arm.angle}')
            # self.get_logger().info(f'wrist angle: {self.front_right_wrist.angle}')

        if self.walk_mode == 1:     
            #self.get_logger().info('Clifford Command Vel Topic')
            speed_factor = 3.0
            walk_speed = abs(data.axes[1] * self.speed_param) * 1.0
            
          

            forward = data.axes[1] >= 0

            if self.set1_walk_index == 0 or self.set1_walk_index == 1 or self.set1_walk_index == 2:
            
                if self.set1_walk_index == 0:
                    #SET 1 COORDINATES
                    self.front_right_current_coords[2] = self.front_right_current_coords[2] - walk_speed if forward else self.front_right_current_coords[2] + walk_speed
                    self.back_left_current_coords[2] = self.back_left_current_coords[2] - walk_speed if forward else self.back_left_current_coords[2] + walk_speed
                
                    #TO DO debatabling we wanna go backwards here.
                    #SET 2 COORDINATES
                    self.front_left_current_coords[0] = self.front_left_current_coords[0] - (walk_speed/speed_factor) if forward else self.front_left_current_coords[0] + (walk_speed/speed_factor)
                    self.back_right_current_coords[0] = self.back_right_current_coords[0] - (walk_speed/speed_factor) if forward else self.back_right_current_coords[0] + (walk_speed/speed_factor)

                    

                    #set1_coordinates and set2 are just variables to keep the code less confusing but really they are front_right/front_left
                    if forward and self.front_right_current_coords[2] >= self.set1_coordinates[self.set1_target_index][2] or \
                        (not forward and self.front_right_current_coords[2] <= self.set1_coordinates[3][2]):
                        
                        
                        #GET ALL SET1 INFO
            
                        #fix value being calculated may have to change to individual
                        self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right(self.front_right_current_coords)
                        rear_left_arm_angle, rear_left_wrist_angle = self.solve_ik_left(self.back_left_current_coords)
                        self.back_left_arm.angle = rear_left_arm_angle + self.back_left_hard_arm
                        self.back_left_wrist.angle = rear_left_wrist_angle + self.back_left_hard_wrist

                    

                        #ALL COORDINATES CALCULATED TO SET1 ARE BASED OFF FRONT_LEFT

                        #HANDLE OTHER DIRECTION LATER.
                        self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_right(self.back_right_current_coords)
                        front_left_arm_angle, front_left_wrist_angle = self.solve_ik_left(self.front_left_current_coords)
                        self.front_left_arm.angle = front_left_arm_angle + self.front_left_hard_arm
                        self.front_left_wrist.angle = front_left_wrist_angle + self.front_left_hard_wrist
                        self.get_logger().info(' set1 walk index = 0')
                        #self.get_logger().info(f'FRONT LEFT COORDS: {self.front_left_current_coords}')
                        #self.get_logger().info(f'BACK RIGHT COORDS: {self.back_right_current_coords}')


                    else:
                        #ENTERING THIS CONDITION
                        #self.get_logger().info('ELSE HIT')

                        #consider clamping to target after every turn 
                        self.set1_walk_index = 1 if forward else 3
                        self.set1_target_index = 2 if forward else 0

                        # self.front_right_current_coords = self.coordinates_front_right[1] if forward else self.coordinates_front_right[0]
                        # self.back_left_current_coords = self.coordinates_back_left[1] if forward else self.coordinates_back_left[0]

                        self.get_logger().info(f'FRONT RIGHT {self.front_right_current_coords}') 
                        self.get_logger().info(f'BACK LEFT {self.back_left_current_coords}') 


                        if not forward:
                            self.set2_walk_index = 3
                            self.set2_target_index = 0
                
                elif self.set1_walk_index == 1:
                    self.get_logger().info(' set1 walk index = 1')
                    #SET 1 COORDINATES
                    self.front_right_current_coords[0] = self.front_right_current_coords[0] + walk_speed if forward else self.front_right_current_coords[0] - walk_speed
                    self.back_left_current_coords[0] = self.back_left_current_coords[0] + walk_speed if forward else self.back_left_current_coords[0] - walk_speed
                
                    #TO DO debatabling we wanna go backwards here.
                    #SET 2 COORDINATES
                    self.front_left_current_coords[0] = self.front_left_current_coords[0] - (walk_speed/speed_factor) if forward else self.front_left_current_coords[0] + (walk_speed/speed_factor)
                    self.back_right_current_coords[0] = self.back_right_current_coords[0] - (walk_speed/speed_factor) if forward else self.back_right_current_coords[0] + (walk_speed/speed_factor)


                    #set1_coordinates and set2 are just variables to keep the code less confusing but really they are front_right/front_left
                    if forward and self.front_right_current_coords[0] <= self.set1_coordinates[self.set1_target_index][0] or \
                        (not forward and self.front_right_current_coords[0] >= self.set1_coordinates[1][0]):
                        self.get_logger().info(f'set1 coordinaes {self.set1_coordinates[1][0]}')
                        
                        #GET ALL SET1 INFO
            
                        #fix value being calculated may have to change to individual
                        self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right(self.front_right_current_coords)
                        rear_left_arm_angle, rear_left_wrist_angle = self.solve_ik_left(self.back_left_current_coords)
                        self.back_left_arm.angle = rear_left_arm_angle + self.back_left_hard_arm
                        self.back_left_wrist.angle = rear_left_wrist_angle + self.back_left_hard_wrist

                        #self.get_logger().info(f'FRONT RIGHT COORDS: {self.front_right_current_coords}')
                        #self.get_logger().info(f'BACK LEFT COORDS: {self.back_left_current_coords}')

                        #ALL COORDINATES CALCULATED TO SET1 ARE BASED OFF FRONT_LEFT

                        #HANDLE OTHER DIRECTION LATER.
                        self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_right(self.back_right_current_coords)
                        front_left_arm_angle, front_left_wrist_angle = self.solve_ik_left(self.front_left_current_coords)
                        self.front_left_arm.angle = front_left_arm_angle + self.front_left_hard_arm
                        self.front_left_wrist.angle = front_left_wrist_angle + self.front_left_hard_wrist

                        #self.get_logger().info(f'FRONT LEFT COORDS: {self.front_left_current_coords}')
                        #self.get_logger().info(f'BACK RIGHT COORDS: {self.back_right_current_coords}')


                        self.get_logger().info(f'FRONT riGHT {self.front_right_current_coords}')
                        self.get_logger().info(f'BACK LEFT {self.back_left_current_coords}') 
                    else:
                        #ENTERING THIS CONDITION
                        self.get_logger().info('ELSE HIT')
                        self.set1_walk_index = 2 if forward else 0
                        self.set1_target_index = 3 if forward else 1

                        #self.front_right_current_coords = self.coordinates_front_right[2] if forward else self.coordinates_front_right[1]
                        #self.back_left_current_coords = self.coordinates_back_left[2] if forward else self.coordinates_back_left[1]

                        self.get_logger().info(f'FRONT RIGHT {self.front_right_current_coords[0]}') 
                        self.get_logger().info(f'BACK LEFT {self.back_left_current_coords[0]}') 
             
                elif self.set1_walk_index == 2:
                    #SET 1 COORDINATES
                    self.get_logger().info(' set1 walk index = 2')
                    self.front_right_current_coords[2] = self.front_right_current_coords[2] + walk_speed if forward else self.front_right_current_coords[2] - walk_speed
                    self.back_left_current_coords[2] = self.back_left_current_coords[2] + walk_speed if forward else self.back_left_current_coords[2] - walk_speed
                
                    #TO DO debatabling we wanna go backwards here.
                    #SET 2 COORDINATES
                    self.front_left_current_coords[0] = self.front_left_current_coords[0] - (walk_speed/speed_factor) if forward else self.front_left_current_coords[0] + (walk_speed/speed_factor)
                    self.back_right_current_coords[0] = self.back_right_current_coords[0] - (walk_speed/speed_factor) if forward else self.back_right_current_coords[0] + (walk_speed/speed_factor)


                    #set1_coordinates and set2 are just variables to keep the code less confusing but really they are front_right/front_left
                    if forward and self.front_right_current_coords[2] <= self.set1_coordinates[self.set1_target_index][2] or \
                        (not forward and self.front_right_current_coords[2] >= self.set1_coordinates[2][2]):
                        
                        
                        #GET ALL SET1 INFO
            
                        #fix value being calculated may have to change to individual
                        self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right(self.front_right_current_coords)
                        rear_left_arm_angle, rear_left_wrist_angle = self.solve_ik_left(self.back_left_current_coords)
                        self.back_left_arm.angle = rear_left_arm_angle + self.back_left_hard_arm
                        self.back_left_wrist.angle = rear_left_wrist_angle + self.back_left_hard_wrist

                        #self.get_logger().info(f'FRONT RIGHT COORDS: {self.front_right_current_coords}')
                        #self.get_logger().info(f'BACK LEFT COORDS: {self.back_left_current_coords}')

                        #ALL COORDINATES CALCULATED TO SET1 ARE BASED OFF FRONT_LEFT

                        #HANDLE OTHER DIRECTION LATER.
                        self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_right(self.back_right_current_coords)
                        front_left_arm_angle, front_left_wrist_angle = self.solve_ik_left(self.front_left_current_coords)
                        self.front_left_arm.angle = front_left_arm_angle + self.front_left_hard_arm
                        self.front_left_wrist.angle = front_left_wrist_angle + self.front_left_hard_wrist

                        #self.get_logger().info(f'FRONT LEFT COORDS: {self.front_left_current_coords}')
                        #self.get_logger().info(f'BACK RIGHT COORDS: {self.back_right_current_coords}')


                    else:
                        #ENTERING THIS CONDITION
                        self.get_logger().info('ELSE HIT')
                        self.set1_walk_index = 3 if forward else 1
                        self.set1_target_index = 0 if forward else 2

                        if self.front_left_current_coords[0] <= 0:
                            self.front_left_current_coords[0] = self.coordinates_front_left[1][0]

                        # self.front_right_current_coords = self.set1_coordinates[self.set1_walk_index]
                        # self.back_left_current_coords = self.coordinates_back_left[self.set1_walk_index]
                        # self.front_right_current_coords = self.coordinates_front_right[3] if forward else self.coordinates_front_right[2]
                        # self.back_left_current_coords = self.coordinates_back_left[3] if forward else self.coordinates_back_left[2]

                        if forward:
                            self.set2_walk_index = 1
                            self.set2_target_index = 2

                            # self.front_left_current_coords = self.coordinates_front_left[1]
                            # self.back_right_current_coords = self.coordinates_back_right[1]
                
            elif self.set2_walk_index == 1 or self.set2_walk_index == 2 or self.set2_walk_index == 3:

                if self.set2_walk_index == 1:
                    #self.get_logger().info('SET CONDITION HIT')
                    #SET 2 COORDINATES
                    self.front_left_current_coords[2] = self.front_left_current_coords[2] - walk_speed if forward else self.front_left_current_coords[2] + (walk_speed)
                    self.back_right_current_coords[2] = self.back_right_current_coords[2] - walk_speed if forward else self.back_right_current_coords[2] + (walk_speed)

                    #self.get_logger().info(f'FRONT LEFT COORDS: {self.front_left_current_coords}')
                    #SET 1 COORDINATES
                    self.front_right_current_coords[0] = self.front_right_current_coords[0] - (walk_speed/speed_factor) if forward else self.front_right_current_coords[0] + (walk_speed/speed_factor)
                    self.back_left_current_coords[0] = self.back_left_current_coords[0] - (walk_speed/speed_factor)if forward else self.back_left_current_coords[0] + (walk_speed/speed_factor)
                
                    #TO DO debatabling we wanna go backwards here.
                      
                    #self.get_logger().info(f'TARGETZ: {self.set2_coordinates[self.set2_target_index][2]}')
                    #self.get_logger().info(f'PREVIOUS Z: {self.set2_coordinates[1][2]}')    

                    #set1_coordinates and set2 are just variables to keep the code less confusing but really they are front_right/front_left
                    if forward and self.front_left_current_coords[2] >= self.set2_coordinates[self.set2_target_index][2] or \
                        (not forward and self.front_left_current_coords[2] <= self.set2_coordinates[1][2]):
                    
                        #GET ALL SET2 INFO
                        self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_right(self.back_right_current_coords)
                        front_left_arm_angle, front_left_wrist_angle = self.solve_ik_left(self.front_left_current_coords)
                        self.front_left_arm.angle = front_left_arm_angle + self.front_left_hard_arm
                        self.front_left_wrist.angle = front_left_wrist_angle + self.front_left_hard_wrist



                        #fix value being calculated may have to change to individual
                        #self.get_logger().info(f'FRONT RIGHT COORDS: {self.front_right_current_coords}')
                        #self.get_logger().info(f'BACK LEFT COORDS: {self.back_left_current_coords}')
                        self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right(self.front_right_current_coords)
                        rear_left_arm_angle, rear_left_wrist_angle = self.solve_ik_left(self.back_left_current_coords)
                        self.back_left_arm.angle = rear_left_arm_angle + self.back_left_hard_arm
                        self.back_left_wrist.angle = rear_left_wrist_angle + self.back_left_hard_wrist

                        #self.get_logger().info(f'FRONT RIGHT COORDS: {self.front_right_current_coords}')
                        #self.get_logger().info(f'BACK LEFT COORDS: {self.back_left_current_coords}')

                        #ALL COORDINATES CALCULATED TO SET1 ARE BASED OFF FRONT_LEFT

                        #HANDLE OTHER DIRECTION LATER.
                        

                        #self.get_logger().info(f'FRONT LEFT COORDS: {self.front_left_current_coords}')
                        #self.get_logger().info(f'BACK RIGHT COORDS: {self.back_right_current_coords}')


                    else:
                        #ENTERING THIS CONDITION
                        self.get_logger().info('ELSE HIT')
                        self.set2_walk_index = 2 if forward else 0
                        self.set2_target_index = 3 if forward else 1

                        if self.front_left_current_coords[0] + walk_speed <= 0:
                            self.front_left_current_coords[0] = self.coordinates_front_left[1][0]

                        if not forward:
                            self.set1_walk_index = 2 
                            self.set1_target_index = 3
                
                elif self.set2_walk_index == 2:

                    #index 2 and 0 giving out of bound error.
                    self.get_logger().info(' set2 walk index = 2')
                    #SET 2 COORDINATES
                    self.front_left_current_coords[0] = self.front_left_current_coords[0] + walk_speed if forward else self.front_left_current_coords[0] - (walk_speed)
                    self.back_right_current_coords[0] = self.back_right_current_coords[0] + walk_speed if forward else self.back_right_current_coords[0] - (walk_speed)

                    #self.get_logger().info(f'FRONT LEFT COORDS: {self.front_left_current_coords}')
                    #SET 1 COORDINATES
                    self.front_right_current_coords[0] = self.front_right_current_coords[0] - (walk_speed/speed_factor) if forward else self.front_right_current_coords[0] + (walk_speed/speed_factor)
                    self.back_left_current_coords[0] = self.back_left_current_coords[0] - (walk_speed/speed_factor)if forward else self.back_left_current_coords[0] + (walk_speed/speed_factor)
                
                    #TO DO debatabling we wanna go backwards here.
                      
                    # self.get_logger().info(f'TARGETZ: {self.set2_coordinates[self.set2_target_index][2]}')
                    # self.get_logger().info(f'PREVIOUS Z: {self.set2_coordinates[0][2]}')    

                    #set1_coordinates and set2 are just variables to keep the code less confusing but really they are front_right/front_left
                    if forward and self.front_left_current_coords[0] <= self.set2_coordinates[self.set2_target_index][0] or \
                        (not forward and self.front_left_current_coords[0] >= self.set2_coordinates[2][0]):
                        
                        #GET ALL SET2 INFO
                        self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_right(self.back_right_current_coords)
                        front_left_arm_angle, front_left_wrist_angle = self.solve_ik_left(self.front_left_current_coords)
                        self.front_left_arm.angle = front_left_arm_angle + self.front_left_hard_arm
                        self.front_left_wrist.angle = front_left_wrist_angle + self.front_left_hard_wrist

                        #fix value being calculated may have to change to individual
                        #self.get_logger().info(f'FRONT RIGHT COORDS: {self.front_right_current_coords}')
                        #self.get_logger().info(f'BACK LEFT COORDS: {self.back_left_current_coords}')
                        self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right(self.front_right_current_coords)
                        rear_left_arm_angle, rear_left_wrist_angle = self.solve_ik_left(self.back_left_current_coords)
                        self.back_left_arm.angle = rear_left_arm_angle + self.back_left_hard_arm
                        self.back_left_wrist.angle = rear_left_wrist_angle + self.back_left_hard_wrist

                        #self.get_logger().info(f'FRONT RIGHT COORDS: {self.front_right_current_coords}')
                        #self.get_logger().info(f'BACK LEFT COORDS: {self.back_left_current_coords}')

                     


                    else:
                        #ENTERING THIS CONDITION
                        self.get_logger().info('ELSE HIT')

                        if self.front_left_current_coords[0]<= 0:
                            self.front_left_current_coords[0] = self.coordinates_front_left[1][0]

                        self.set2_walk_index = 3 if forward else 1
                        self.set2_target_index = 0 if forward else 2
                
                elif self.set2_walk_index == 3:
                       # self.get_logger().info('SET CONDITION HIT')
                        #SET 2 COORDINATES
                        self.front_left_current_coords[2] = self.front_left_current_coords[2] + walk_speed if forward else self.front_left_current_coords[2] - (walk_speed)
                        self.back_right_current_coords[2] = self.back_right_current_coords[2] + walk_speed if forward else self.back_right_current_coords[2] - (walk_speed)

                       # self.get_logger().info(f'FRONT LEFT COORDS: {self.front_left_current_coords}')
                        #SET 1 COORDINATES
                        self.front_right_current_coords[0] = self.front_right_current_coords[0] - (walk_speed/speed_factor) if forward else self.front_right_current_coords[0] + (walk_speed/speed_factor)
                        self.back_left_current_coords[0] = self.back_left_current_coords[0] - (walk_speed/speed_factor)if forward else self.back_left_current_coords[0] + (walk_speed/speed_factor)
                    
                        #TO DO debatabling we wanna go backwards here.
                        
                       # self.get_logger().info(f'TARGETZ: {self.set2_coordinates[self.set2_target_index][2]}')
                        # self.get_logger().info(f'PREVIOUS Z: {self.set2_coordinates[0][2]}')    
                        #self.coordinates_front_left[0][2]
                        #set1_coordinates and set2 are just variables to keep the code less confusing but really they are front_right/front_left
                        if forward and self.front_left_current_coords[2] <=  172.0 or \
                            (not forward and self.front_left_current_coords[2] >= self.set2_coordinates[2][2]):
                            
                            #GET ALL SET2 INFO
                            self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_right(self.back_right_current_coords)
                            front_left_arm_angle, front_left_wrist_angle = self.solve_ik_left(self.front_left_current_coords)
                            self.front_left_arm.angle = front_left_arm_angle + self.front_left_hard_arm
                            self.front_left_wrist.angle = front_left_wrist_angle + self.front_left_hard_wrist

                            #fix value being calculated may have to change to individual
                           # self.get_logger().info(f'FRONT RIGHT COORDS: {self.front_right_current_coords}')
                            #self.get_logger().info(f'BACK LEFT COORDS: {self.back_left_current_coords}')
                            self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right(self.front_right_current_coords)
                            rear_left_arm_angle, rear_left_wrist_angle = self.solve_ik_left(self.back_left_current_coords)
                            self.back_left_arm.angle = rear_left_arm_angle + self.back_left_hard_arm
                            self.back_left_wrist.angle = rear_left_wrist_angle + self.back_left_hard_wrist

                            #self.get_logger().info(f'FRONT RIGHT COORDS: {self.front_right_current_coords}')
                            #self.get_logger().info(f'BACK LEFT COORDS: {self.back_left_current_coords}')

                        


                        else:
                            #ENTERING THIS CONDITION
                            self.get_logger().info('ELSE HIT')
                            self.set2_walk_index = 0 if forward else 2
                            self.set2_target_index = 1 if forward else 3

                            if forward:
                                self.set1_walk_index = 0
                                self.set1_target_index = 1

                            # self.set2_walk_index = 3 if forward else 1
                            # self.set2_target_index = 0 if forward else 2


    def standing_pos(self):
        self.walk_mode = 0
        self.idle_mode = 1

        # I should probably incorporate the hard angle differences inside the calculations instead lol

        self.back_left_current_coords = self.coordinates_back_left[0]
        self.back_right_current_coords = self.coordinates_back_right[0]
        self.front_left_current_coords = self.coordinates_front_left[0]
        self.front_right_current_coords = self.coordinates_front_right[0]

        sleep(1)

        self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right (self.front_right_current_coords)
        self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_right (self.back_right_current_coords)

        front_left_arm_angle , front_left_wrist_angle = self.solve_ik_left(self.front_left_current_coords)
        self.front_left_arm.angle = front_left_arm_angle + self.front_left_hard_arm
        self.front_left_wrist.angle = front_left_wrist_angle + self.front_left_hard_wrist

        back_left_arm_angle , back_left_wrist_angle = self.solve_ik_left(self.back_left_current_coords)
        self.back_left_arm.angle = back_left_arm_angle + self.back_left_hard_arm
        self.back_left_wrist.angle = back_left_wrist_angle + self.back_left_hard_wrist

    def laydown_pos(self):
        self.back_left_current_coords = self.coordinates_back_left[0]
        self.back_right_current_coords = self.coordinates_back_right[0]
        self.front_left_current_coords = self.coordinates_front_left[0]
        self.front_right_current_coords = self.coordinates_front_right[0]

        sleep(1)
        step_size = 1.0
        while(step_size <= 40.0):
            self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right (self.front_right_current_coords)
            self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_right (self.back_right_current_coords)

            front_left_arm_angle , front_left_wrist_angle = self.solve_ik_left(self.front_left_current_coords)
            self.front_left_arm.angle = front_left_arm_angle + self.front_left_hard_arm
            self.front_left_wrist.angle = front_left_wrist_angle + self.front_left_hard_wrist

            back_left_arm_angle , back_left_wrist_angle = self.solve_ik_left(self.back_left_current_coords)
            self.back_left_arm.angle = back_left_arm_angle + self.back_left_hard_arm
            self.back_left_wrist.angle = back_left_wrist_angle + self.back_left_hard_wrist

            self.front_right_current_coords[0] = self.front_right_current_coords[0] - step_size
            self.front_left_current_coords[0] = self.front_left_current_coords[0] - step_size
            self.back_right_current_coords[0] = self.back_right_current_coords[0] - step_size
            self.back_left_current_coords[0] = self.back_left_current_coords[0] - step_size
            
            step_size = step_size + 1.0
            sleep(0.25)

    # Helper function to test servo range of motion (FINISH IMPLEMENTATION ONCE REST ARE INSTALLED).
    def init_servos(self): 
        self.get_logger().info("Helper Function: 'init_servos' called.")
        sleep(2.5)
        self.front_left_shoulder.angle = 100
        self.front_left_arm.angle = 145
        self.front_left_wrist.angle = 105
        sleep(2.5)
        self.front_right_shoulder.angle = 110
        self.front_right_arm.angle = 58
        self.front_right_wrist.angle = 95
        sleep(2.5)
        self.back_left_shoulder.angle = 100
        self.back_left_arm.angle = 140
        self.back_left_wrist.angle = 110
        sleep(2.5)
        self.back_right_shoulder.angle = 105
        self.back_right_arm.angle = 55
        self.back_right_wrist.angle = 95

    #DEFINE NEW FUNC RESET SERVOS

    def solve_ik_right(self,cords):
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
            theta_2 = math.pi - (beta_2 + beta_1)
            theta_3 = math.pi - beta_3

            #FINAL VALUE FOR RVIZ
            #theta_3 = (math.pi/2) - theta_3 #Final value of right_wrist
            #theta_3 = (math.pi/2) + beta_3

            theta_2 = theta_2 * (180/math.pi)
            theta_3 = theta_3 * (180/math.pi)

            return [theta_2,theta_3]

    def solve_ik_left(self,cords):
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
            theta_2 = beta_2 + beta_1
            theta_3 = beta_3

            theta_2 = theta_2 * (180/math.pi)
            theta_3 = theta_3 * (180/math.pi)

           # self.get_logger().info(f"THETA_3 before math corrects: {theta_3}")
            
            return [theta_2,theta_3]

    def solve_pitch(self, coords):
        self.get_logger().info(f"SOLVE FOR PITCH")

        C_value = math.sqrt( ( math.pow(coords[2],2) ) + math.pow(coords[1],2) )
        D_value = math.sqrt( math.pow(C_value,2) - math.pow(self.universal_shoulder_len,2) )
        self.get_logger().info(f"D VALUE: {D_value}")
            

        alpha = math.atan( coords[1] / coords[2] )
        beta = math.atan( D_value / self.universal_shoulder_len )
            
        self.get_logger().info(f"ALPHA VALUE: {alpha}")
        self.get_logger().info(f"BETA VALUE: {beta}")

        omega = alpha + beta
        theta_1 = math.pi - omega
        self.get_logger().info(f"OMEGA VALUE: {omega}")


        #define right shoulder = omega
        coords[2] = D_value #update only the z value

        theta_2,theta_3 = self.solve_ik_left(coords) #returned values of just z updated   
        theta_1 = theta_1 * (180/math.pi)

        return [theta_1,theta_2,theta_3]
 

def main(args=None):
    rclpy.init(args=args)
    driveServos = ServoDriver()
    rclpy.spin(driveServos)
    driveServos.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()