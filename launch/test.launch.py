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

    set_service_env = SetEnvironmentVariable('GZSATELLITE_SERVICE', 'https://xdworld.vworld.kr/2d/Satellite/service/{z}/{x}/{y}.jpeg')
    set_lat_env = SetEnvironmentVariable('GZSATELLITE_LAT', '36.381365')
    set_lon_env = SetEnvironmentVariable('GZSATELLITE_LON', '127.364937')
    set_zoom_env = SetEnvironmentVariable('GZSATELLITE_ZOOM', '19')
    set_width_env = SetEnvironmentVariable('GZSATELLITE_WIDTH', '256')
    set_height_env = SetEnvironmentVariable('GZSATELLITE_HEIGHT', '256')
    set_shift_x_env = SetEnvironmentVariable('GZSATELLITE_SHIFT_EW', '0')
    set_shift_y_env = SetEnvironmentVariable('GZSATELLITE_SHIFT_NS', '0')
    set_name_env = SetEnvironmentVariable('GZSATELLITE_NAME', 'ETRI')
    set_quality_env = SetEnvironmentVariable('GZSATELLITE_QUALITY', '100')
      
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
        set_service_env,
        set_lat_env,
        set_lon_env,
        set_zoom_env,
        set_width_env,
        set_height_env,
        set_shift_x_env,
        set_shift_y_env,
        set_name_env,
        set_quality_env,
        gazebo_launch
    ])
