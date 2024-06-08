import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped


#Create a JointStatePublisher
class LegJointPublisher(Node):

    def __init__(self):
        super().__init__('leg_mover') 
        self.get_logger().info('Leg Joint Publisher is online.')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10) #create publisher to post transforms
        self.timer = self.create_timer(0.1, self.publish_joint_states) #look into

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['base_link', 'link_1', 'link_2']  # Replace with the names of your joints
        joint_state_msg.position = [0.0, 1, 0.0]  # Set the positions of your joints here
        joint_state_msg.velocity = [0.0, 0.0, 0.0]  # Set the velocities of your joints here
        joint_state_msg.effort = [0.0, 0.0, 0.0]  # Set the efforts of your joints here
        self.joint_state_pub.publish(joint_state_msg)


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
