from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.substitutions import Command


def generate_launch_description():
    """Launch the vehicle simulation with action server, client, and sensor model"""

    # Sensor model environment config
    sensor_param_config = PathJoinSubstitution([
        FindPackageShare('sensor_model'), 'config',
        [LaunchConfiguration('environment'), '.yaml']
    ])

    # Launch arguments
    start_rviz = LaunchConfiguration('start_rviz')
    drift_velocity = LaunchConfiguration('drift_velocity')       # idle drift speed
    constant_velocity = LaunchConfiguration('constant_velocity')  # action server navigation speed
    time_step = LaunchConfiguration('time_step')
    turn_radius_m = LaunchConfiguration('turn_radius_m')          # vehicle min turn radius
    max_pitch_deg = LaunchConfiguration('max_pitch_deg')          # vehicle max pitch angle
    log_level = LaunchConfiguration('log_level')
    
    # RViz configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('vista_sim'),
        'config',
        'vista_sim.rviz'
    ])
    
    # Process the URDF file using ParameterValue
    package_dir = FindPackageShare('eca_a9_description')
    urdf_path = PathJoinSubstitution([package_dir, 'urdf', 'eca_a9_visual.urdf.xacro'])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    return LaunchDescription([
        # Launch argument declarations
        DeclareLaunchArgument(
            'environment', default_value='environment_basic',
            description='Environment yaml name (no extension) from sensor_model/config/'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 with the simulation'
        ),
        DeclareLaunchArgument(
            'drift_velocity',
            default_value='0.25',
            description='Idle drift velocity when not navigating (m/s)'
        ),
        DeclareLaunchArgument(
            'constant_velocity',
            default_value='1.5',
            description='Navigation velocity for Dubins path following (m/s)'
        ),
        DeclareLaunchArgument(
            'time_step',
            default_value='0.1',
            description='Simulation time step in seconds'
        ),
        DeclareLaunchArgument(
            'turn_radius_m',
            default_value='10.0',
            description='Vehicle minimum turn radius (m); the planner inflates this 20 percent so paths stay feasible'
        ),
        DeclareLaunchArgument(
            'max_pitch_deg',
            default_value='15.0',
            description='Vehicle maximum pitch angle (deg) for planning and dynamics'
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS log level for vista_sim Python nodes (debug, info, warn, error, fatal)'
        ),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_ned_static',
            arguments=['--x', '0', '--y', '0', '--z', '50', '--yaw', '1.57079632679', '--pitch', '0', '--roll', '3.14159265359', '--frame-id', 'map', '--child-frame-id', 'ned'],
        ),

        # Robot State Publisher 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description_content}
            ]
        ),

        # Vehicle action server — plans and follows Dubins paths at navigation
        # velocity, and is the SOLE owner of base_link: it broadcasts the
        # transform during a goal and propagates idle drift (drift_velocity)
        # between goals. Replaces the former separate drift_service +
        # pause/resume handshake, which broadcast base_link from two nodes at
        # once (~20 Hz) and produced the teleport flicker.
        Node(
            package='vista_sim',
            executable='dubins_pose_to_pose_action_server',
            name='vehicle_sim_server',
            output='screen',
            parameters=[
                {
                    'frame_id': 'ned',
                    'time_step': time_step,
                    'constant_velocity': constant_velocity,
                    'drift_velocity': drift_velocity,
                    'turn_radius_m': turn_radius_m,
                    'max_pitch_deg': max_pitch_deg,
                }
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        # Dubins path-length cost server — answers /compute_path_lengths for
        # cost-informed NBV selection. Stateless: turn radius and max pitch come
        # in per-request; it just needs the TF tree (ned, roi_frame, base_link,
        # sonar_optical) published above.
        Node(
            package='vista_sim',
            executable='path_lengths_server',
            name='path_length_service',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        ),

        # Sensor model: sonar → sonar_optical frame convention
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sonar_to_optical_static',
            arguments=['--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                       '--frame-id', 'sonar', '--child-frame-id', 'sonar_optical'],
        ),

        # Sensor model: scene mesh visualisation (RViz-only). Publishes a
        # TRIANGLE_LIST terrain marker whose size scales linearly with area, so
        # it gets heavy at full scale; nothing but RViz consumes it. Gate on
        # start_rviz so headless runs skip it entirely.
        Node(
            package='sensor_model',
            executable='meshes_rviz',
            name='meshes_rviz',
            parameters=[sensor_param_config],
            arguments=['--ros-args', '--log-level', log_level],
            condition=IfCondition(start_rviz)
        ),

        # Sensor model: FLS depth image publisher
        Node(
            package='sensor_model',
            executable='sensor_publisher',
            name='depth_publisher',
            parameters=[sensor_param_config],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        # RViz node with configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(start_rviz)
        )
    ])