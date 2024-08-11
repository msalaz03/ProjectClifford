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
        self.get_logger().info('Clifford Servo2 Driver Online...')

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
        self.front_left_arm.angle = 85  #OFFSET
        self.front_left_wrist.angle = 105  #OFFSET

        self.back_right_shoulder.angle = 105
        self.back_right_arm.angle = 100 #OFFSET
        self.back_right_wrist.angle = 93  #OFFSET

        self.back_left_shoulder.angle = 100
        self.back_left_arm.angle = 77  #OFFSET
        self.back_left_wrist.angle = 109  #OFFSET

        self.front_right_shoulder.angle = 100
        self.front_right_arm.angle = 102  #OFFSET
        self.front_right_wrist.angle = 93  #OFFSET

        

        #INIT OUR SERVO COORDINATE SYSTEM (TO DO)
        # self.coordinates_front_left = [
          
        # ]
        
        # self.coordinates_front_right = [
            
        # ]

        # self.coordinates_back_left = [
          
        # ]

        # self.coordinates_back_right = [
          
        # ]

        
        #FLAGS FOR CLIFFORD DIFFERENT MODES DIFFERENT MODES
        self.idle_mode = 0
        self.walk_mode = 1
       
        #TESTING VARIABLES FOR SINGLE LEG MOTION (07/24/24) / FRONT RIGHT
        self.speed_param = 6.5
        self.gait_walk_index = 0
        self.target_index = 1
        

        #GAIT VARIABLES
        self.set1_walk_index = 0
        self.set2_walk_index = 0
        self.set1_target_index = 1
        self.set2_target_index = 1
                
    
        #TESTING VARIABLES
        self.zero_coords = [130.43,58.17,107.0]

    def clifford_joystick_callback(self, data):
        #self.get_logger().info('Clifford Joystick Callback')
        
        # X button condition
        if data.buttons[0] == 1:
            self.get_logger().info('X Pressed...')
            z_cord = 5.0
            self.zero_coords[2] += z_cord

            self.front_left_arm.angle, self.front_left_wrist.angle = self.solve_ik_front_left(self.zero_coords)
            self.back_left_arm.angle, self.back_left_wrist.angle = self.solve_ik_back_left(self.zero_coords)

            self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_front_right(self.zero_coords)
            self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_back_right(self.zero_coords)
            
         
           
        # Circle button condition
        elif data.buttons[1] == 1:
            self.get_logger().info("Circle Pressed...")
            x_cord = -5.0
            self.zero_coords[0] += x_cord

            self.front_left_arm.angle, self.front_left_wrist.angle = self.solve_ik_front_left(self.zero_coords)
            self.back_left_arm.angle, self.back_left_wrist.angle = self.solve_ik_back_left(self.zero_coords)

            self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_front_right(self.zero_coords)
            self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_back_right(self.zero_coords)
            

            self.get_logger().info(f"CURRENT COORDS {self.zero_coords}")
         
        #Triangle button condition
        elif data.buttons[2] == 1:
            self.get_logger().info("Triangle Pressed...")
            z_cord = -5.0
            self.zero_coords[2] += z_cord

            self.front_left_arm.angle, self.front_left_wrist.angle = self.solve_ik_front_left(self.zero_coords)
            self.back_left_arm.angle, self.back_left_wrist.angle = self.solve_ik_back_left(self.zero_coords)

            self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_front_right(self.zero_coords)
            self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_back_right(self.zero_coords)
            
     
        # Square button condition
        elif data.buttons[3] == 1:
            self.get_logger().info('Sqaure Pressed...')
            x_cord = 5.0
            self.zero_coords[0] += x_cord

            self.front_left_arm.angle, self.front_left_wrist.angle = self.solve_ik_front_left(self.zero_coords)
            self.back_left_arm.angle, self.back_left_wrist.angle = self.solve_ik_back_left(self.zero_coords)

            self.front_right_arm.angle, self.front_right_wrist.angle = self.solve_ik_front_right(self.zero_coords)
            self.back_right_arm.angle, self.back_right_wrist.angle = self.solve_ik_back_right(self.zero_coords)
            
      
   

    def reset_walk(self):
        self.get_logger().info("reset hit")
        self.set1_walk_index = 0
        self.set2_walk_index = 0
        self.set1_target_index = 1
        self.set2_target_index = 1
        #self.get_logger().info(f"set1 target {s}")

        self.set1_current_coords = self.coordinates_front_right[0]
        self.set2_current_coords = self.coordinates_front_left[0]


        self.front_left_current_coords = self.coordinates_front_left[0]
        self.front_right_current_coords = self.coordinates_front_right[0]
        self.back_left_current_coords = self.coordinates_back_left[0]
        self.back_right_current_coords = self.coordinates_back_right[0]
        self.get_logger().info(f"{self.front_left_current_coords}")
        self.set1_coordinates = self.coordinates_front_right
        self.set2_coordinates = self.coordinates_front_left
        sleep(2)

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

    def solve_ik_front_right(self,cords):
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
            arm_offset = 12
            wrist_offset = 3 

            theta_2 = (theta_2 * (180/math.pi)) + arm_offset
            theta_3 = (theta_3 * (180/math.pi)) + wrist_offset

            return [theta_2,theta_3]

    def solve_ik_front_left(self,cords):
        
            # These kinematics calculations will try to be as descripitional as possible but please refer
            # to sheet of calculations by Cameron Bauman. 
            #UNIT: RADIANS & mm
            #self.get_logger().info(f"LEG HAS FAILED AT {cords}")

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

            arm_offset = -5 
            wrist_offset = 15

            theta_2 = (theta_2 * (180/math.pi)) + arm_offset
            theta_3 = (theta_3 * (180/math.pi)) + wrist_offset

           # self.get_logger().info(f"THETA_3 before math corrects: {theta_3}")
            
            return [theta_2,theta_3]

    def solve_ik_back_right(self,cords):
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

            arm_offset = 10
            wrist_offset = 3

            theta_2 = (theta_2 * (180/math.pi)) + arm_offset
            theta_3 = (theta_3 * (180/math.pi)) + wrist_offset 

            return [theta_2,theta_3]

    def solve_ik_back_left(self,cords):
        
            # These kinematics calculations will try to be as descripitional as possible but please refer
            # to sheet of calculations by Cameron Bauman. 
            #UNIT: RADIANS & mm
           # self.get_logger().info(f"LEG HAS FAILED AT {cords}")

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

            arm_offset = -13
            wrist_offset = 19

            theta_2 = (theta_2 * (180/math.pi)) + arm_offset
            theta_3 = (theta_3 * (180/math.pi)) + wrist_offset

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