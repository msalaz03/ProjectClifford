import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set the path to this package.
    pkg_share = FindPackageShare(package='left_leg_demo').find('left_leg_demo')
    # Set the path to the URDF file
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/left_leg_demo.urdf')

    # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'rviz_basic_settings.rviz')

    #Set the path to find Joy Params
    teleop_controller_pkg = FindPackageShare(package='teleop_controller')
    joy_params = os.path.join(teleop_controller_pkg, 'config', 'joystick.yaml')

    ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############
    # Launch configuration variables specific to simulation
    gui = LaunchConfiguration('gui')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
       name='rviz_config_file',
       default_value=default_rviz_config_path, 
       description='Full path to the RVIZ config file to use')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    #Subscribe to Joy node in order to update visuals
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            joy_params,
            {'use_sim_time':use_sim_time}
            ]

    )

    CliffordJoy_teleop = Node(
        package='teleop_controller',
        executable='CliffordJoy',
        parameters=[
            {'use_sim_time':use_sim_time}
        ]
    )

    #Subscribe to Joint node in order to get custom visuals based on Joy
    joint_states_node = Node(
        package='teleop_controller',
        executable='CliffordTF',
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', urdf_model]), value_type=str)
        }],
        arguments=[default_urdf_model_path])

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    #Our added actions
    ld.add_action(joint_states_node)
    ld.add_action(joy_node)
    ld.add_action(CliffordJoy_teleop)
    
    return ld
