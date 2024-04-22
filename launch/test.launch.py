from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare launch arguments for easier configuration and reuse
    launch_args = [
        DeclareLaunchArgument(
        'world', default_value=PathJoinSubstitution([
            FindPackageShare('gzsatellite'), 'worlds', 'satellite.world'
        ]), description='Full path to the world model file to load'),
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode for Gazebo'),
        DeclareLaunchArgument('gui', default_value='true', description='Enable Gazebo GUI'),
        DeclareLaunchArgument('paused', default_value='true', description='Start Gazebo in paused mode'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo without a GUI'),
        DeclareLaunchArgument('verbose', default_value='true', description='Enable verbose mode for Gazebo')
    ]

    # Include Gazebo launch file with parameters
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),  # Correct package for gazebo_ros
                'launch',
                'gazebo.launch.py'  # Correct launch file name
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
            'verbose': LaunchConfiguration('verbose')
        }.items()
    )

    return LaunchDescription([
        *launch_args,
        gazebo_launch
    ])
