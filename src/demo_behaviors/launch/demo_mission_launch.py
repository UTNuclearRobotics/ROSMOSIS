"""
Launch the full NBV mission stack:
  - vista_sim (simulator, Dubins action server, sensor, drift service, RViz, static TFs)
  - helix_service (Python service for viewpoint sampling)
  - next_best_view_server (NBV scoring via TSDF)
  - run_bt (the behavior tree)
  - rqt_console (filterable log viewer for all nodes)   [debug_gui:=true only]
  - rqt_graph (node/topic graph viewer)                [debug_gui:=true only]

All user-tunable parameters are declared here at the top so this file is the
one-stop shop for tweaking mission settings.
The BT itself waits for required services via the CheckForServers behavior
at the top of MainTree, so no launch-side delay is needed.
"""

import datetime
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- Launch arguments (mission-level config) ----
    environment_arg = DeclareLaunchArgument(
        'environment', default_value='env_1000x1000_cluster_seabed',
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
        'constant_velocity', default_value='1.5',
        description='Navigation velocity for Dubins path following (m/s)'
    )
    turn_radius_m_arg = DeclareLaunchArgument(
        'turn_radius_m', default_value='10.0',
        description='Vehicle minimum turn radius (m); planner inflates ~20%. ARL full-scale spec.'
    )
    max_pitch_deg_arg = DeclareLaunchArgument(
        'max_pitch_deg', default_value='15.0',
        description='Vehicle maximum pitch angle (deg) for planning and dynamics.'
    )
    time_step_arg = DeclareLaunchArgument(
        'time_step', default_value='0.1',
        description='Simulation time step (seconds)'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='ROS log level (debug, info, warn, error, fatal). Applies to demo_bt, helix_service, nbv_server.'
    )
    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        description='Record a rosbag of the mission for offline analysis '
                    '(/face_hits /detected_boxes /tf /tf_static).'
    )
    bag_prefix_arg = DeclareLaunchArgument(
        'bag_prefix', default_value='nbv',
        description='Output bag directory prefix; a timestamp is appended.'
    )
    alpha_arg = DeclareLaunchArgument(
        'alpha', default_value='0.5',
        description='CI-NBV cost weight in [0,1] for GetBestViewWithCost: '
                    'utility = (1-alpha)*IG_norm - alpha*cost_norm. '
                    '0 = pure info-gain (greedy), 1 = pure cost. This is the knob to sweep.'
    )
    debug_gui_arg = DeclareLaunchArgument(
        'debug_gui', default_value='false',
        description='Start the rqt_console / rqt_graph debugging GUIs. Leave false '
                    'for headless / batch / parallel runs (they need a display).'
    )

    # Capture as LaunchConfiguration substitutions for forwarding
    environment = LaunchConfiguration('environment')
    start_rviz = LaunchConfiguration('start_rviz')
    drift_velocity = LaunchConfiguration('drift_velocity')
    constant_velocity = LaunchConfiguration('constant_velocity')
    turn_radius_m = LaunchConfiguration('turn_radius_m')
    max_pitch_deg = LaunchConfiguration('max_pitch_deg')
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
            'turn_radius_m': turn_radius_m,
            'max_pitch_deg': max_pitch_deg,
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
        # Publish the RViz belief map only when RViz is up; headless/batch runs
        # (start_rviz:=false) skip the 2 Hz full-grid publish entirely.
        parameters=[{'publish_belief': ParameterValue(start_rviz, value_type=bool)}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Per-run output label, computed once and shared by the bag and the
    # reconstructions so they carry the same name. data/ is at the workspace root
    # (run ros2 launch from there).
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    recon_dir = os.path.join(os.getcwd(), 'data', 'reconstructions')

    # Behavior tree runner - MainTree's CheckForServers gates startup, so no
    # launch-side delay is needed. reconstruction_dir is passed to run_bt (node
    # demo_bt), which seeds it onto the BT blackboard so SaveModelToFile writes
    # each target's mesh under data/reconstructions/<bag_prefix>_<timestamp>/.
    bt_runner = Node(
        package='demo_behaviors',
        executable='run_bt',
        name='demo_bt',
        output='screen',
        parameters=[{
            'reconstruction_dir': ParameterValue(
                [os.path.join(recon_dir, ''), LaunchConfiguration('bag_prefix'), '_', stamp],
                value_type=str,
            ),
            # CI-NBV cost weight, seeded onto the blackboard by run_bt and read by
            # GetBestViewWithCost as {alpha}. This is what the experiment sweep varies.
            'alpha': ParameterValue(LaunchConfiguration('alpha'), value_type=float),
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Rosbag recording for offline analysis. Opt-in via record:=true. Same topic
    # set and storage as the boustrophedon baseline so the two are directly
    # comparable: timing (stamps), detections, face hits (CIR), TF tree. MCAP is
    # self-describing (custom FaceHits decodes without sourcing ROS) and
    # crash-robust. Needs ros-humble-rosbag2-storage-mcap installed.
    # Bags land in <workspace-root>/data/bags/ (run ros2 launch from the
    # workspace root). data/bags/ is gitignored; keepers are uploaded to
    # external storage where needed.
    bags_dir = os.path.join(os.getcwd(), 'data', 'bags')
    os.makedirs(bags_dir, exist_ok=True)
    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=[
            'ros2', 'bag', 'record',
            '/face_hits', '/detected_boxes', '/tf', '/tf_static',
            '--storage', 'mcap',
            '--max-cache-size', '2000000000',
            '--output', [os.path.join(bags_dir, ''), LaunchConfiguration('bag_prefix'), '_', stamp],
        ],
        output='screen',
    )

    # When the BT runner exits (run_bt returns after the tree hits SUCCESS/FAILURE
    # at the root, i.e. mission timeout or all pots inspected), shut the launch
    # down. This SIGINTs every process, cleanly finalizing the bag, and makes the
    # launch self-terminating for scripted/repeated runs.
    mission_done = RegisterEventHandler(
        OnProcessExit(
            target_action=bt_runner,
            on_exit=[EmitEvent(event=Shutdown(reason='mission complete'))],
        )
    )

    # Optional debugging GUIs. Gated by debug_gui (default false) so headless /
    # batch / parallel runs don't try to open windows with no display.
    rqt_console = Node(
        package='rqt_console',
        executable='rqt_console',
        name='rqt_console',
        condition=IfCondition(LaunchConfiguration('debug_gui')),
    )
    rqt_graph = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        condition=IfCondition(LaunchConfiguration('debug_gui')),
    )

    return LaunchDescription([
        # arg declarations
        environment_arg,
        start_rviz_arg,
        drift_velocity_arg,
        constant_velocity_arg,
        turn_radius_m_arg,
        max_pitch_deg_arg,
        time_step_arg,
        log_level_arg,
        record_arg,
        bag_prefix_arg,
        alpha_arg,
        debug_gui_arg,
        # nodes / includes
        vista_sim_include,
        helix_service,
        cone_service,
        nbv_server,
        bayesian_search_server,
        bt_runner,
        rosbag_record,
        mission_done,
        # debugging GUIs
        rqt_console,
        rqt_graph,
    ])