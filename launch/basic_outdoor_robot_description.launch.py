import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # set path to different files and folders
    pkg_share = FindPackageShare(package='basic_outdoor_robot_description').find('basic_outdoor_robot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'basic_outdoor_robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'basic_outdoor_robot.rviz')
    robot_description = Command(['xacro ', default_model_path])

    # Declare the launch arguments
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='URDF path')
    
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        name='rviz_config', 
        default_value=default_rviz_config_path,
        description='RVIZ config path')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock if true') 

    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        name='use_joint_state_pub',
        default_value='False',
        description='Whether to start the joint state publisher')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_robot_description_cmd = DeclareLaunchArgument(
        name = 'robot_description',
        default_value=robot_description,
        description= 'URDF definition of robot'
    )

    # Start Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'robot_description': LaunchConfiguration('robot_description')}],
        arguments=[default_model_path],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('use_joint_state_pub'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition = IfCondition(LaunchConfiguration('use_rviz'))
    )

    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_joint_state_pub_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
