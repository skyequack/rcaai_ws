import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    
    # Declare launch arguments
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('graspnet_gazebo'),
            'worlds',
            'graspnet_world.world'
        ]),
        description='Full path to world file to load'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode if true'
    )
    
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('headless', default='false')),
    )
    
    # Launch robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('graspnet_description'),
                'launch',
                'display.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': 'false'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ur5_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    
    # Add actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_description_launch)
    ld.add_action(spawn_robot_cmd)
    
    return ld
