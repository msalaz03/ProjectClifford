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
        pca = PCA9685(i2c) # Create an instance of PCA9685
        pca.frequency = 60 # Set the PWM frequency to 60Hz
        
        #INIT SERVO DUTY
        for i in range (12):
            pca.channels[i].duty_cycle = 0x7FFF #50% Duty Cycle

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
        
        #INIT SERVO RELATIVE COODS + MISC
        self.universal_shoulder_len = 58.17
        self.universal_arm_len = 107.00
        self.universal_wrist_len = 130.43


        #INIT OUR SERVOS TO CORRECT POSITIONS
        #self.init_servos()

        #INIT OUR SERVO COORDINATE SYSTEM (TO DO)
        self.coordinates_front_left = [130.43,58.17,107.0]
        self.coordinates_front_right = [
            [59.43,58.17,150.0],
            [59.53,58.17,100.0],
            [109.53,58.17,100.0],
            [109.43,58.17,150.0],
            ]
        self.coordinates_back_left = [130.43,58.17,107.0]
        self.coordinates_back_right = [50.2,58.17,154]

        #FLAGS FOR CLIFFORD DIFFERENT MODES DIFFERENT MODES
        self.idle_mode = 0
        self.walk_mode = 1

        #DEFINE INDEXS AND TRACKING VARIABLES FOR WALK GAIT
        self.set1_walk_index = 0
        self.set2_walk_index = 0
        self.set1_target_index = 1
        self.set2_target_index = 1
       
        #TESTING VARIABLES FOR SINGLE LEG MOTION (07/24/24) / FRONT RIGHT
        self.speed_param = 2.0
        self.gait_walk_index = 0
        self.target_index = 1
        self.current_coords = [59.43,58.17,150.0]
 

    def clifford_joystick_callback(self, data):
        #self.get_logger().info('Clifford Joystick Callback')

        # X button condition
        if data.buttons[0] == 1:
            self.get_logger().info('X Pressed...')
            #self.walk_mode = 1
            #self.idle_mode = 0
          #  sleep(2)
            # self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right([59.43,58.17,150.0])
            # self.get_logger().info(f'arm angle: {self.front_right_arm.angle}')
            # self.get_logger().info(f'wrist angle: {self.front_right_wrist.angle}')
            self.init_servos()

        # Circle button condition
        elif data.buttons[1] == 1:
            self.get_logger().info("Circle Pressed...")
           # sleep(2)
            # self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_right([59.43,58.17,100.0])
            # self.get_logger().info(f'arm angle: {self.front_right_arm.angle}')
            # self.get_logger().info(f'wrist angle: {self.front_right_wrist.angle}')

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

        if self.walk_mode:
            
            walk_speed = abs(data.axes[1]) * self.speed_param # define our walking speed
            forward = data.axes[1] >= 0 # Determine direction of movement
            self.get_logger().info(f'walk_speed: {walk_speed}')
            if self.gait_walk_index == 0:
                #WALKING GAIT POS 1
                new_z = self.current_coords[2] - walk_speed if forward else self.current_coords[2] + walk_speed

                if (forward and self.current_coords[2] >= self.coordinates_front_right[self.target_index][2]) or \
                (not forward and self.current_coords[2] <= self.coordinates_front_right[3][2]):
                    self.current_coords[2] = new_z
                    self.front_right_arm.angle,self.front_right_wrist.angle = self.solve_ik_right(self.current_coords)
                else:
                    self.get_logger().info('Transitioning gait index')
                    #CONDITIONS TO SWITCH OUR INDEXS AND TARGETS
                    self.gait_walk_index = 1 if forward else 3
                    self.target_index = 2 if forward else 0

            elif self.gait_walk_index == 1:
                # WALKING GAIT POS 2
                new_x = self.current_coords[0] + walk_speed if forward else self.current_coords[0] - walk_speed

                if (forward and self.current_coords[0] < self.coordinates_front_right[self.target_index][0]) or \
                (not forward and self.current_coords[0] > self.coordinates_front_right[0][0]):
                    self.current_coords[0] = new_x
                    self.front_right_arm.angle,self.front_right_wrist.angle = self.solve_ik_right(self.current_coords)    
                else:
                    self.get_logger().info('Transitioning gait index')
                    self.gait_walk_index = 2 if forward else 0
                    self.target_index = 3 if forward else 1
                    
            elif self.gait_walk_index == 2:
                # WALKING GAIT POS 3
                new_z = self.current_coords[2] + walk_speed if forward else self.current_coords[2] - walk_speed

                if (forward and self.current_coords[2] < self.coordinates_front_right[self.target_index][2]) or \
                (not forward and self.current_coords[2] > self.coordinates_front_right[1][2]):
                    self.current_coords[2] = new_z
                    self.front_right_arm.angle,self.front_right_wrist.angle = self.solve_ik_right(self.current_coords)
                else:
                    self.get_logger().info('Transitioning gait index')
                    self.gait_walk_index = 3 if forward else 1
                    self.target_index = 0 if forward else 2

            elif self.gait_walk_index == 3:
                # WALKING GAIT POS 4
                new_x = self.current_coords[0] - walk_speed if forward else self.current_coords[0] + walk_speed

                if (forward and self.current_coords[0] > self.coordinates_front_right[0][0]) or \
                (not forward and self.current_coords[0] < self.coordinates_front_right[2][0]):
                    self.current_coords[0] = new_x
                    self.front_right_arm.angle,self.front_right_wrist.angle = self.solve_ik_right(self.current_coords)
                  

                    self.get_logger().info('updated x coordinate')
                else:
                    self.get_logger().info('Transitioning gait index')
                    self.gait_walk_index = 0 if forward else 2
                    self.target_index = 1 if forward else 3

            else:
                self.get_logger().info("unexpected condition hit.")

                

    

    def cmd_vel_callback1(self, msg):
        
        #QUESTIONABLE SETUP FOR CHOOSING WHETHER TO LISTEN TO CMD_VEL TOPIC
        if not self.walk_mode:
            return 

        self.get_logger().info('Clifford Command Vel Topic')
        speed_factor = 1.0
        walk_speed = abs(msg.linear.x) * 1.0
        forward = msg.linear.x >= 0


        if self.set1_walk_index == 0 or self.set1_walk_index == 1 or self.set1_walk_index == 2:
            self.get_logger().info('SET1_WALK_INDEX: 0-2')
            #set1_new_z = self.current_coords_set1[2] - walk_speed if forward else self.current_coords_set1[2] + walk_speed
                

            if self.set1_walk_index == 0:
                self.get_logger().info('SET1 INDEX = 0')
                set1_new_z = self.current_coords_set1[2] - walk_speed if forward else self.current_coords_set1[2] + walk_speed
                set2_new_x = self.current_coords_set2[0] - (walk_speed / speed_factor) if forward else self.current_coords_set2[0] + walk_speed

                self.get_logger().info(f'SET2 NEWX: {set2_new_x}')
                self.get_logger().info(f'SET1 coordinate z {set1_new_z}')
                if forward and self.current_coords_set1[2] >= self.coordinates_front_right_set1[self.target_index_set1][2]:
                    
                   
                    self.get_logger().info('UPDATING COORDINATES SET1 AS MAIN')
                    #GET ALL SET1 INFO
                    #self.current_coords_set1 = [self.current_coords_set1[0], self.current_coords_set1[1],set1_new_z]
                    self.current_coords_set1[2] = set1_new_z
                    self.current_position_right_arm, self.current_position_right_wrist = self.solve_ik(self.current_coords_set1)
                    self.current_position_rear_left_arm, self.current_position_rear_left_wrist = -self.current_position_right_arm, -self.current_position_right_wrist

                    #GET ALL SET2 INFO
                    self.get_logger().info(f'CURRENT COORDS FOR SET2: {self.current_coords_set2}')
                    self.current_coords_set2[0] = set2_new_x
        
                    self.current_position_rear_right_arm, self.current_position_rear_right_wrist = self.solve_ik(self.current_coords_set2)
                    self.current_position_front_left_arm, self.current_position_front_left_wrist = -self.current_position_rear_right_arm, -self.current_position_rear_right_wrist

                else:
                    #ENTERING THIS CONDITION
                    self.get_logger().info('ELSE HIT')
                    self.get_logger().info(f'PIECE OF SHIT FAILING CONDITION {self.coordinates_set1[self.target_index_set1][2]:}')
                    self.set1_walk_index = 1
                    self.target_index_set1 = 2
            
            
    def zero_servos(self):
        self.get_logger().info("Helper Function: 'zero_servos' called")
        sleep(1)
        #self.servo0.angle = 0
        sleep(2)
        self.servo0.angle = 90
        self.servo1.angle = 90

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

            self.get_logger().info(f"THETA_3 before math corrects: {theta_3}")
            
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