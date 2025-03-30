import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # path to world file
    pkg_share = get_package_share_directory('basic_outdoor_robot_description')
    world_file_name = 'my_world.sdf'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    print(world_path)

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 
                                      'launch', 'gz_sim.launch.py')]), 
                                      launch_arguments={'gz_args': ['-r -v4 ', world_path], 'on_exit_shutdown':'true'}.items()
    )
    
    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'basic_outdoor_robot',
                   '-x', '278.08',
                   '-y', '-134.22',
                   '-z', '4.16'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # ROS2 Gazebo Bridge
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "imu@sensor_msgs/msg/Imu@gz.msgs.IMU"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(gz_bridge_node)

    return ld