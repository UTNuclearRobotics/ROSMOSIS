"""
Launch the full NBV mission stack:
  - vista_sim (simulator, Dubins action server, sensor, drift service, RViz, static TFs)
  - helix_service (Python service for viewpoint sampling)
  - next_best_view_server (NBV scoring via TSDF)
  - run_bt (the behavior tree)
  - rqt_console (filterable log viewer for all nodes)
  - rqt_graph (node/topic graph viewer)

All user-tunable parameters are declared here at the top so this file is the
one-stop shop for tweaking mission settings.
The BT itself waits for required services via the CheckForServers behavior
at the top of MainTree, so no launch-side delay is needed.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- Launch arguments (mission-level config) ----
    environment_arg = DeclareLaunchArgument(
        'environment', default_value='environment_basic',
        description='Environment yaml name (no extension) from sensor_model/config/'
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz', default_value='true',
        description='Start RViz2 with the simulation'
    )
    drift_velocity_arg = DeclareLaunchArgument(
        'drift_velocity', default_value='0.25',
        description='Idle drift velocity when not navigating (m/s)'
    )
    constant_velocity_arg = DeclareLaunchArgument(
        'constant_velocity', default_value='0.5',
        description='Navigation velocity for Dubins path following (m/s)'
    )
    time_step_arg = DeclareLaunchArgument(
        'time_step', default_value='0.1',
        description='Simulation time step (seconds)'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='ROS log level (debug, info, warn, error, fatal). Applies to demo_bt, helix_service, nbv_server.'
    )

    # Capture as LaunchConfiguration substitutions for forwarding
    environment = LaunchConfiguration('environment')
    start_rviz = LaunchConfiguration('start_rviz')
    drift_velocity = LaunchConfiguration('drift_velocity')
    constant_velocity = LaunchConfiguration('constant_velocity')
    time_step = LaunchConfiguration('time_step')
    log_level = LaunchConfiguration('log_level')

    # ---- Includes / Nodes ----
    # Include vista_sim's launch: sim + Dubins action server + sensor + RViz + static TFs.
    # Forward our top-level args so vista_sim picks them up.
    vista_sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vista_sim'), 'launch', 'vista_sim_launch.py')
        ),
        launch_arguments={
            'environment': environment,
            'start_rviz': start_rviz,
            'drift_velocity': drift_velocity,
            'constant_velocity': constant_velocity,
            'time_step': time_step,
            'log_level': log_level,
        }.items()
    )

    # Helix viewpoint sampler service (Python)
    helix_service = Node(
        package='pose_generator',
        executable='helix_service',
        name='helix_service',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Cone viewpoint sampler service (Python; alternative manifold to the helix)
    cone_service = Node(
        package='pose_generator',
        executable='cone_service',
        name='cone_service',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # NBV server (TSDF + scoring), configured via nbv_params.yaml in sensor_model/config
    nbv_params_path = PathJoinSubstitution([
        FindPackageShare('sensor_model'), 'config', 'nbv_params.yaml'
    ])
    nbv_server = Node(
        package='nbv_cpp',
        executable='nbv_server',
        name='next_best_view_server',
        output='screen',
        parameters=[nbv_params_path],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Bayesian search server (probability map + next-waypoint service for the
    # search subtree). Resolves FLS geometry from the map->ned TF at startup.
    bayesian_search_server = Node(
        package='bayesian_search',
        executable='bayesian_search_server',
        name='bayesian_search_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Behavior tree runner - MainTree's CheckForServers gates startup,
    # so no launch-side delay is needed
    bt_runner = Node(
        package='demo_behaviors',
        executable='run_bt',
        name='demo_bt',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Optional debugging GUIs
    rqt_console = Node(
        package='rqt_console',
        executable='rqt_console',
        name='rqt_console',
    )
    rqt_graph = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
    )

    return LaunchDescription([
        # arg declarations
        environment_arg,
        start_rviz_arg,
        drift_velocity_arg,
        constant_velocity_arg,
        time_step_arg,
        log_level_arg,
        # nodes / includes
        vista_sim_include,
        helix_service,
        cone_service,
        nbv_server,
        bayesian_search_server,
        bt_runner,
        # debugging GUIs
        rqt_console,
        rqt_graph,
    ])