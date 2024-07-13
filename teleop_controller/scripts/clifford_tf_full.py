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

class CliffordJointStatePublisher(Node):
    #TO DO FIX FRONT LEFT NOT BEING INIT CORRECT

    def __init__(self):
        super().__init__('Clifford') 
        self.get_logger().info('Clifford Joint Publisher is online.')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10) #create publisher for 'joint_states'
        self.timer = self.create_timer(0.1, self.publish_joint_states) #look into
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel', self.cmd_vel_callback,10) #create instance of subscribing to cmd_vel/ topic
        self.joy_sub = self.create_subscription(Joy,'joy', self.general_joy_callback,10) #create susbcription for ps4-controller 'joy'

        #DEFINED POSITIONAL VALUES FOR JOINTS
        self.current_position_right_shoulder = 0.0
        self.current_position_right_arm = -0.07507
        self.current_position_right_wrist = -0.0866
        #self.current_position_right_arm = 0.0
        #self.current_position_right_wrist = 0.0

        self.current_position_front_left_shoulder = 0.0
        self.current_position_front_left_arm = 0.07507
        self.current_position_front_left_wrist = 0.0866

        self.current_position_rear_left_shoulder = 0.0
        self.current_position_rear_left_arm = 0.07507
        self.current_position_rear_left_wrist = 0.0866

        self.current_position_rear_right_shoulder = 0.0
        self.current_position_rear_right_arm = -0.07507
        self.current_position_rear_right_wrist = -0.0866

        #DEFINED LENGTHS OF JOINTS
        self.right_shoulder_len = 58.17
        self.right_arm_len = 107.00
        self.right_wrist_len = 130.43

        #DEFINED COORDINATES FOR POS1 - POS4 FOR ALL LEG SETS
        self.coordinates = [
            [36.1,0.0,157.3],
            [36.1,0.0,80.0],
            [80.9,0.0,80.0],
            [80.9,0.0,157.3]
        ]

        self.coordinates_set1 = [
            [36.1,0.0,157.3],
            [36.1,0.0,80.0],
            [80.9,0.0,80.0],
            [80.9,0.0,157.3]
        ]

        self.coordinates_set2 = [
            [36.1,0.0,157.3],
            [0.5,0.0,157.3],
            [0.5,0.0,80.0],
            [36.1,0.0,80.0]
        ]

        # self.coordinates_set2 = [
        #     [80.9,0.0,157.3],
        #     [36.1,0.0,157.3],
        #     [36.1,0.0,80.0],
        #     [80.9,0.0,80.0],
        # ]

        self.leg_flag = True #TEST FLAG

        self.gait_walk_index = 0 #this is an index for coordinate system to know what position we are.
        self.target_index = self.gait_walk_index + 1 #this will be where our leg should be ended up.
        self.current_coords = self.coordinates[self.gait_walk_index] #this is a variable to keep track of where we are.
        
        #DEFINE OUR TRACKING FOR THE TWO LEG SETS (TESTING PURPOSES)
        self.set1_walk_index = 0
        self.set2_walk_index = 0
        self.current_coords_set1 = self.coordinates_set1[self.set1_walk_index]
        self.current_coords_set2 = self.coordinates_set2[self.set2_walk_index]
        self.target_index_set1 = 1
        self.target_index_set2 = 1

       

    #A function created in order to add hotkey functionality on the controller.
    def general_joy_callback(self, data):

        #If the triangle button is hit, reset all joints to general stance.
        if data.buttons[2] == 1:
            self.get_logger().info("Resetting Joints...")
            self.reset_walk_gait()
            
        #Circle button condition
        elif data.buttons[1] == 1:
            self.get_logger().info('BUTTON FEATURE 2')
            HARD_X = 39.143 #offsets from origin to axis.
            HARD_Z = 212.78 #z offset 
            test_cords = [-186.0, 21.05, 86.5] #actual coordinates

            C_value = math.sqrt( ( math.pow(test_cords[2],2) ) + math.pow(test_cords[0],2) )
            #smaller the x the further away
            #smaller z the lower it was.
            test_cords = [186.0 - HARD_X, 21.05, HARD_Z - 120] #adjusted coordinates

            self.get_logger().info(f'X VALUE {test_cords[0]}')
            self.get_logger().info(f'Z VALUE {test_cords[2]}')

            self.get_logger().info(f"C VALUE: {C_value}")

            D_value = math.sqrt( math.pow(C_value,2) - math.pow(self.right_shoulder_len,2) )
            self.get_logger().info(f"D VALUE: {D_value}")
            

            alpha = math.atan( test_cords[2] / test_cords[0] )
            beta = math.atan( D_value / self.right_shoulder_len )
            
            self.get_logger().info(f"ALPHA VALUE: {alpha}")
            self.get_logger().info(f"BETA VALUE: {beta}")

            omega = (math.pi/2) - (alpha + beta)
            self.get_logger().info(f"OMEGA VALUE: {omega}")

            self.current_position_right_shoulder = omega #set shoulder servo pos

            self.current_coords[2] = D_value #update only the z value
            self.get_logger().info(f"Z VALUE BEFORE IK {self.current_coords}")

            theta_1,theta_2 = self.solve_ik(self.current_coords) #returned values of just z updated


            #TO DO LOok into signs.
            self.current_position_right_arm = theta_1 #update joint_states for arm
            self.current_position_right_wrist = theta_2 #update joint_States for wrist.

            #we need a coordinate system
            #subtract relative positions (ONLY FOR RVIZ)
            #calculate C, then D

            #use that to find alpha and beta -> used to calculate omega
            #then find omega -> omega tells us the actual servo positions (adjust for relativity)
            #D is a value to determine how far the arm should for a point of contact respective to the gnd

            

        #X button condition
        elif data.buttons[0] == 1:
            self.get_logger().info('BUTTON FEATURE 3')
          
        #Square button condition
        elif data.buttons[3] == 1:
            self.get_logger().info('BUTTON FEATURE 4')

    def cmd_vel_callback(self,msg):
        self.get_logger().info('cmd_vel topic callback final gait')
        

        #80.9 -> 36.1
        #lets get set1 to begin walking first.
        walk_speed = abs(msg.linear.x) * 7.0
        forward = msg.linear.x >= 0

        
        #first condition
        #set1 will be in charge of most of the movement 
        #set2 will also be moving during this time but at slower pace
        #lets make sure they end at roughly the same time.

        if self.set1_walk_index == 0 or self.set1_walk_index == 1 or self.set1_walk_index == 2:
            self.get_logger().info('SET1_WALK_INDEX: 0-2')
            #set1_new_z = self.current_coords_set1[2] - walk_speed if forward else self.current_coords_set1[2] + walk_speed
                

            if self.set1_walk_index == 0:
                self.get_logger().info('SET1 INDEX = 0')
                set1_new_z = self.current_coords_set1[2] - walk_speed if forward else self.current_coords_set1[2] + walk_speed
                set2_new_x = self.current_coords_set2[0] - (walk_speed / 6) if forward else self.current_coords_set2[0] + walk_speed

                self.get_logger().info(f'SET2 NEWX: {set2_new_x}')
                self.get_logger().info(f'SET1 coordinate z {set1_new_z}')
                if forward and self.current_coords_set1[2] >= self.coordinates_set1[self.target_index_set1][2]:
                    
                   
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
            
            elif self.set1_walk_index == 1:
                #TO DO FINISH CASE 2
                self.get_logger().info('SET1 INDEX = 1')
                set1_new_x = self.current_coords_set1[0] + walk_speed if forward else self.current_coords_set1[0] - walk_speed
                set2_new_x = self.current_coords_set2[0] - (walk_speed / 6) if forward else self.current_coords_set2[0] + walk_speed

                #self.get_logger().info(f'SET2 NEWX: {set1_new_z}')

                if forward and self.current_coords_set1[0] < self.coordinates_set1[self.target_index_set1][0]:
                    
                    self.get_logger().info('UPDATING COORDINATES SET1 AS MAIN')
                    #GET ALL SET1 INFO
                    self.current_coords_set1[0] = set1_new_x
                    self.current_position_right_arm, self.current_position_right_wrist = self.solve_ik(self.current_coords_set1)
                    self.current_position_rear_left_arm, self.current_position_rear_left_wrist = -self.current_position_right_arm, -self.current_position_right_wrist
                    
                    #GET ALL SET2 INFO
                    self.get_logger().info(f'CURRENT COORDS FOR SET2: {self.current_coords_set2}')
                    self.current_coords_set2[0] = set2_new_x
                    self.current_position_rear_right_arm, self.current_position_rear_right_wrist = self.solve_ik(self.current_coords_set2)
                    self.current_position_front_left_arm, self.current_position_front_left_wrist = -self.current_position_rear_right_arm, -self.current_position_rear_right_wrist
                else:
                    self.set1_walk_index = 2
                    self.target_index_set1 = 3

            elif self.set1_walk_index == 2:
                #TO DO FINISH CASE 2
                self.get_logger().info('SET1 INDEX = 2')
                set1_new_z = self.current_coords_set1[2] + walk_speed if forward else self.current_coords_set1[2] + walk_speed
                set2_new_x = self.current_coords_set2[0] - (walk_speed / 6) if forward else self.current_coords_set2[0] + walk_speed

                self.get_logger().info(f'SET2 NEWX: {set2_new_x}')
                #self.get_logger().info(f'SET1 coordinate z {set1_new_z}')
                if forward and self.current_coords_set1[2] <= self.coordinates_set1[self.target_index_set1][2]:
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
                    self.set1_walk_index = 3
                    self.target_index_set1 = 0

                    self.set2_walk_index = 1
                    self.target_index_set2 = 2

        elif self.set2_walk_index == 1 or self.set2_walk_index == 2 or self.set2_walk_index == 3:
                self.get_logger().info('SET2_WALK_INDEX: 1-3')

                #157.3->80.0
                if self.set2_walk_index == 1:
                    self.get_logger().info('SET2 INDEX = 0')
                    set2_new_z = self.current_coords_set2[2] - walk_speed if forward else self.current_coords_set2[2] + walk_speed
                    set1_new_x = self.current_coords_set1[0] - (walk_speed / 6) if forward else self.current_coords_set1[0] + walk_speed

                    if forward and self.current_coords_set2[2] >= self.coordinates_set2[self.target_index_set2][2]:
                        
                        self.get_logger().info('UPDATING COORDINATES SET2 AS MAIN')
                        #GET ALL SET1 INFO
                        #self.current_coords_set1 = [self.current_coords_set1[0], self.current_coords_set1[1],set1_new_z]
                        self.current_coords_set2[2] = set2_new_z
                        self.current_position_rear_right_arm, self.current_position_rear_right_wrist = self.solve_ik(self.current_coords_set2)
                        self.current_position_front_left_arm, self.current_position_front_left_wrist = -self.current_position_rear_right_arm, -self.current_position_rear_right_wrist
                        #GET ALL SET2 INFO
                        #self.get_logger().info(f'CURRENT COORDS FOR SET2: {self.current_coords_set2}')
                        self.current_coords_set1[0] = set1_new_x
            
                        self.current_position_right_arm, self.current_position_right_wrist = self.solve_ik(self.current_coords_set1)
                        self.current_position_rear_left_arm, self.current_position_rear_left_wrist = -self.current_position_right_arm, -self.current_position_right_wrist
                    else:
                        #ENTERING THIS CONDITION
                        self.get_logger().info('ELSE HIT')
                        self.set2_walk_index = 2
                        self.target_index_set2 = 3

                elif self.set2_walk_index == 2:
                    #0.5->36.1

                    set2_new_x = self.current_coords_set2[0] + walk_speed if forward else self.current_coords_set2[0] - walk_speed
                    set1_new_x = self.current_coords_set1[0] - (walk_speed / 6) if forward else self.current_coords_set1[0] + walk_speed

                    self.get_logger().info(f'SELF COORDINATES SET2 {self.coordinates_set2[self.target_index_set2]}')
                    if forward and self.current_coords_set2[0] <= self.coordinates_set2[self.target_index_set2][0]:
                            self.current_coords_set2[0] = set2_new_x
                            self.current_position_rear_right_arm, self.current_position_rear_right_wrist = self.solve_ik(self.current_coords_set2)
                            self.current_position_front_left_arm, self.current_position_front_left_wrist = -self.current_position_rear_right_arm, -self.current_position_rear_right_wrist

                            self.current_coords_set1[0] = set1_new_x
                            self.current_position_right_arm, self.current_position_right_wrist = self.solve_ik(self.current_coords_set1)
                            self.current_position_rear_left_arm, self.current_position_rear_left_wrist = -self.current_position_right_arm, -self.current_position_right_wrist
                    else:
                        self.get_logger().info('ELSE HIT')
                        self.set2_walk_index = 3
                        self.target_index_set2 = 0

                elif self.set2_walk_index == 3:
                    #80 -> 157.3

                    set2_new_z = self.current_coords_set2[2] + walk_speed if forward else self.current_coords_set2[2] - walk_speed
                    set1_new_x = self.current_coords_set1[0] - (walk_speed / 6) if forward else self.current_coords_set1[0] + walk_speed

                    self.get_logger().info(f'COORDINATES 2 CURRENT: {self.current_coords_set2}')
                    self.get_logger().info(f'COORDINATES 2 CONDITION {self.coordinates_set2[self.target_index_set2][2]}')
                    if forward and self.current_coords_set2[2] <= 157.3:

                        self.get_logger().info(f'COORDINATES FOR SET1 {self.current_coords_set1}')
                        self.current_coords_set2[2] = set2_new_z
                        self.current_position_rear_right_arm, self.current_position_rear_right_wrist = self.solve_ik(self.current_coords_set2)
                        self.current_position_front_left_arm, self.current_position_front_left_wrist = -self.current_position_rear_right_arm, -self.current_position_rear_right_wrist

                        self.current_coords_set1[0] = set1_new_x
                        self.current_position_right_arm, self.current_position_right_wrist = self.solve_ik(self.current_coords_set1)
                        self.current_position_rear_left_arm, self.current_position_rear_left_wrist = -self.current_position_right_arm, -self.current_position_right_wrist
                    else:
                        self.get_logger().info('ELSE HIT')
                        self.target_index_set1 = 1
                        self.set1_walk_index = 0

                        self.set2_walk_index = 0
                        self.target_index_set2 = 1

        #second condition 
        #set2 will be in charge of most of the movement
        #set1 will be also moving during this time but at a slower pace.



    def cmd_vel_callback1(self,msg):
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
                theta_right_arm, theta_right_wrist = self.solve_ik(self.current_coords)

                if self.leg_flag:
                    self.current_position_right_arm = theta_right_arm
                    self.current_position_right_wrist = theta_right_wrist

                    self.current_position_rear_left_arm = -theta_right_arm
                    self.current_position_rear_left_wrist = -theta_right_wrist
                else:            
                    self.current_position_rear_right_arm = theta_right_arm
                    self.current_position_rear_right_wrist = theta_right_wrist

                    self.current_position_front_left_arm = -theta_right_arm
                    self.current_position_front_left_wrist = -theta_right_wrist              
                
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
                theta_right_arm, theta_right_wrist = self.solve_ik(self.current_coords)

                if self.leg_flag:
                    self.current_position_right_arm = theta_right_arm
                    self.current_position_right_wrist = theta_right_wrist

                    self.current_position_rear_left_arm = -theta_right_arm
                    self.current_position_rear_left_wrist = -theta_right_wrist
                else:            
                    self.current_position_rear_right_arm = theta_right_arm
                    self.current_position_rear_right_wrist = theta_right_wrist

                    self.current_position_front_left_arm = -theta_right_arm
                    self.current_position_front_left_wrist = -theta_right_wrist

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
                theta_right_arm, theta_right_wrist = self.solve_ik(self.current_coords)
              
                if self.leg_flag:
                    self.current_position_right_arm = theta_right_arm
                    self.current_position_right_wrist = theta_right_wrist

                    self.current_position_rear_left_arm = -theta_right_arm
                    self.current_position_rear_left_wrist = -theta_right_wrist
                else:            
                    self.current_position_rear_right_arm = theta_right_arm
                    self.current_position_rear_right_wrist = theta_right_wrist

                    self.current_position_front_left_arm = -theta_right_arm
                    self.current_position_front_left_wrist = -theta_right_wrist


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
                theta_right_arm, theta_right_wrist = self.solve_ik(self.current_coords)
                
                if self.leg_flag:
                    self.current_position_right_arm = theta_right_arm
                    self.current_position_right_wrist = theta_right_wrist

                    self.current_position_rear_left_arm = -theta_right_arm
                    self.current_position_rear_left_wrist = -theta_right_wrist
                else:            
                    self.current_position_rear_right_arm = theta_right_arm
                    self.current_position_rear_right_wrist = theta_right_wrist

                    self.current_position_front_left_arm = -theta_right_arm
                    self.current_position_front_left_wrist = -theta_right_wrist


                self.get_logger().info('updated x coordinate')
            else:
                self.get_logger().info('Transitioning gait index')

                #TO DO FIX THIS THING BACKWARDS WONKY.
                self.gait_walk_index = 0 if forward else 2
                self.target_index = 1 if forward else 3
                self.leg_flag = not self.leg_flag
                self.get_logger().info(f"FLAG IS SET {self.leg_flag}")
                #sleep(1500)

        else:
            self.get_logger().info("unexpected condition hit.")

        self.publish_joint_states()

    
    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['front_right_shoulder-joint', 'front_right_arm-joint','front_right_wrist-joint',
                                'front_left_shoulder-joint', 'front_left_arm-joint','front_left_wrist-joint',
                                'rear_left_shoulder-joint', 'rear_left_arm-joint', 'rear_left_wrist-joint',
                                'rear_right_shoulder-joint', 'rear_right_arm-joint', 'rear_right_wrist-joint']
        
        joint_state_msg.position = [self.current_position_right_shoulder,self.current_position_right_arm, self.current_position_right_wrist, 
                                    self.current_position_front_left_shoulder, self.current_position_front_left_arm, self.current_position_front_left_wrist,           
                                    self.current_position_rear_left_shoulder, self.current_position_rear_left_arm, self.current_position_rear_left_wrist,
                                    self.current_position_rear_right_shoulder, self.current_position_rear_right_arm, self.current_position_rear_right_wrist] #self.current_position_y]
        
        self.joint_state_pub.publish(joint_state_msg)

        #TO DO FIX INCONSISTENT RESET ON BACKRIGHT

    def reset_walk_gait(self):
        self.gait_walk_index = 0 #reinit
        self.target_index = self.gait_walk_index + 1 #reinit

        sleep(0.05)
        self.current_coords[0] = self.coordinates[self.gait_walk_index][0] #starting position
        self.current_coords[1] =self.coordinates[self.gait_walk_index][1]
        self.current_coords[2] = self.coordinates[self.gait_walk_index][2]

        self.get_logger().info(f'Current Cords are {self.current_coords}')
        theta_right_arm,theta_right_wrist = self.solve_ik(self.current_coords) #call our ik solver func
        self.get_logger().info(f'theta_right_arm {theta_right_arm}')
        self.get_logger().info(f'theta_right_arm {theta_right_wrist}')

        self.current_position_right_shoulder = 0.0
        self.current_position_right_arm = theta_right_arm #set those positions right_arm
        self.current_position_right_wrist = theta_right_wrist #set those positions right_wrist
        
        self.current_position_rear_right_arm = theta_right_arm #set those positions right_arm
        self.current_position_rear_right_wrist = theta_right_wrist #set those positions right_wrist

        self.current_position_front_left_arm = -theta_right_arm #set those positions right_arm
        self.current_position_front_left_wrist = -theta_right_wrist #set those positions right_wrist

        self.current_position_rear_left_arm = -theta_right_arm #set those positions right_arm
        self.current_position_rear_left_wrist = -theta_right_wrist #set those positions right_wrist

        #sleep(2)
        self.leg_flag = True

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
        beta_2 = math.acos( ( pow(self.right_arm_len,2) + pow(b_len,2) - pow(self.right_wrist_len,2) ) 
                                / (2 * self.right_arm_len * b_len) )

        #This is the angle of which right_wrist is set.
        beta_3 = math.acos( ( pow(self.right_arm_len,2) + pow(self.right_wrist_len,2) - pow(b_len,2) ) 
                                / (2 * self.right_arm_len * self.right_wrist_len) )
        
        #Shouldn't be too relevant to calculations besides for RVIZ, but this is to make the calculations relative to their axes.
        b_prime = (math.pi  * 2) - beta_1
        theta_2 = b_prime - beta_2
        theta_2 = theta_2 - math.pi


        #THIS VALUE IS HARD CODED
        theta_2 = (math.pi/4) - theta_2  #Final value of right_arm 

        theta_3 = math.pi - beta_3
        theta_3 = (math.pi/2) - theta_3 #Final value of right_wrist
        
        return [theta_2,theta_3]

    #def pitch_cancer(self):


def main(args=None):
    rclpy.init(args=args)
    full_model_publisher = CliffordJointStatePublisher()
    rclpy.spin(full_model_publisher)
    CliffordJointStatePublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
