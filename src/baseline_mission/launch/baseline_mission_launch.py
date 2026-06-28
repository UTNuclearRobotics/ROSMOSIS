"""
Launch the boustrophedon baseline survey:
  - vista_sim (simulator, Dubins action server, sensor model, RViz, static TFs)
  - boustrophedon_run (lawnmower action client)

This is the non-adaptive coverage baseline. It reuses the same simulator,
Dubins action server, and sensor model as the NBV mission, so the two are
directly comparable. The client self-gates on the ned<-map TF (to resolve its
survey depth from clearance) and on the pose_to_pose action server, so no
launch-side delay is needed.
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        description='ROS log level (debug, info, warn, error, fatal).'
    )
    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        description='Record a rosbag of the survey for offline analysis '
                    '(/face_hits /detected_boxes /tf /tf_static).'
    )
    bag_prefix_arg = DeclareLaunchArgument(
        'bag_prefix', default_value='boustrophedon',
        description='Output bag directory prefix; a timestamp is appended.'
    )
    debug_gui_arg = DeclareLaunchArgument(
        'debug_gui', default_value='false',
        description='Start the rqt_console / rqt_graph debugging GUIs. Leave false '
                    'for headless / batch / parallel runs (they need a display).'
    )

    environment = LaunchConfiguration('environment')
    start_rviz = LaunchConfiguration('start_rviz')
    drift_velocity = LaunchConfiguration('drift_velocity')
    constant_velocity = LaunchConfiguration('constant_velocity')
    time_step = LaunchConfiguration('time_step')
    log_level = LaunchConfiguration('log_level')

    # ---- Includes / Nodes ----
    # vista_sim: sim + Dubins action server + sensor model + RViz + static TFs.
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

    # Boustrophedon lawnmower client. Self-gates on the ned<-map TF and the
    # pose_to_pose action server, so no launch-side delay is needed.
    boustrophedon_run = Node(
        package='baseline_mission',
        executable='boustrophedon_run',
        name='boustrophedon_run',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # When the survey node exits (it terminates itself after the last waypoint),
    # shut the whole launch down. This SIGINTs every process, which is how
    # `ros2 bag record` cleanly finalizes the bag (flush + metadata.yaml), and
    # makes the launch self-terminating for scripted/repeated runs.
    survey_done = RegisterEventHandler(
        OnProcessExit(
            target_action=boustrophedon_run,
            on_exit=[EmitEvent(event=Shutdown(reason='survey complete'))],
        )
    )

    # Rosbag recording for offline analysis. Opt-in via record:=true. Captures
    # the minimal set needed by the post-processing script: timing (message
    # stamps), lobster-pot detections (labels + positions), face hits (CIR), and
    # the TF tree (vehicle trajectory). MCAP storage: self-describing (embeds
    # message schemas, so custom types like FaceHits decode without sourcing
    # ROS), crash-robust, and chunk-compressed by the plugin. Needs the Humble
    # plugin: apt install ros-humble-rosbag2-storage-mcap.
    # Bags land in <workspace-root>/data/bags/ (run ros2 launch from the
    # workspace root). data/bags/ is gitignored; keepers are uploaded to
    # external storage where needed.
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
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

    # Debugging GUIs (mirrors demo_mission_launch.py): filterable log viewer
    # for all nodes, and the node/topic graph. Gated by debug_gui (default false)
    # so headless / batch / parallel runs don't try to open windows with no display.
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
        time_step_arg,
        log_level_arg,
        record_arg,
        bag_prefix_arg,
        debug_gui_arg,
        # nodes / includes
        vista_sim_include,
        boustrophedon_run,
        rosbag_record,
        survey_done,
        # debugging GUIs
        rqt_console,
        rqt_graph,
    ])