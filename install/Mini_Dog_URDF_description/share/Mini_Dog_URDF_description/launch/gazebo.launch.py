from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessExit
import os
import xacro
import random
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ================================================================
    # Package paths
    # ================================================================
    share_dir = get_package_share_directory('Mini_Dog_URDF_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # ================================================================
    # Process URDF/Xacro with Leg-to-Leg Manufacturing Asymmetry
    # Randomize friction and damping ±10% to simulate 3D print variations
    # ================================================================
    xacro_file = os.path.join(share_dir, 'urdf', 'Mini_Dog_URDF.xacro')
    mappings = {
        'fl_damp_mult': str(random.uniform(0.9, 1.1)),
        'rl_damp_mult': str(random.uniform(0.9, 1.1)),
        'fr_damp_mult': str(random.uniform(0.9, 1.1)),
        'rr_damp_mult': str(random.uniform(0.9, 1.1)),
        'fl_fric_mult': str(random.uniform(0.9, 1.1)),
        'rl_fric_mult': str(random.uniform(0.9, 1.1)),
        'fr_fric_mult': str(random.uniform(0.9, 1.1)),
        'rr_fric_mult': str(random.uniform(0.9, 1.1)),
    }
    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_urdf = robot_description_config.toxml()

    # Controller config
    controllers_yaml = os.path.join(share_dir, 'config', 'controllers.yaml')

    # ================================================================
    # Launch Arguments
    # ================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # ================================================================
    # Robot State Publisher
    # Publishes /robot_description and TF transforms
    # ================================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': True,
        }],
    )

    # ================================================================
    # Gazebo Harmonic (Gz Sim) Server + GUI
    # Using DART physics engine for accurate closed-loop 4-bar linkage
    # ================================================================
    lab_world_path = os.path.join(share_dir, 'config', 'lab_world.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 "{lab_world_path}"',
        }.items()
    )

    # ================================================================
    # Spawn Robot into Gz Sim
    # ================================================================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'Mini_Dog_URDF',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
        ],
        output='screen',
    )

    # ================================================================
    # Gz-ROS Bridge
    # Bridges /clock and /imu/data_raw from Gz to ROS 2
    # ================================================================
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    # ================================================================
    # ros2_control: Load and activate controllers
    # These are spawned sequentially after the robot is loaded
    # ================================================================

    # 1. Joint State Broadcaster — must start first
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 2. Joint Position Controller — starts after joint_state_broadcaster
    load_joint_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_controller"],
        output="screen",
    )

    # ================================================================
    # Event handlers: sequence controller loading
    # spawn_robot → joint_state_broadcaster → joint_position_controller
    # ================================================================
    start_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    start_position_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_position_controller],
        )
    )

    # ================================================================
    # Battery Voltage Sag Simulation Node
    # Reads joint efforts, computes voltage droop, publishes
    # degradation multiplier for servo torque throttling
    # ================================================================
    # Nodes (Battery Sag & Open Loop Visualizer)
    # ================================================================
    battery_sag_node = Node(
        package='Mini_Dog_URDF_description',
        executable='battery_sag_node',
        name='battery_sag_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'v_max': 8.4,              # LiPo fully charged (4.2V × 2)
            'v_nominal': 7.4,          # Nominal voltage
            'v_cutoff': 6.4,           # Low-voltage cutoff (3.2V × 2)
            'r_internal': 0.10,        # Pack internal resistance (Ω)
            'capacity_ah': 0.30,       # 300mAh
            'tau_stall': 0.5394,       # PTK-7465 stall torque (N·m)
            'i_stall': 1.6,            # PTK-7465 stall current (A)
            'i_noload': 0.08,          # PTK-7465 no-load current (A)
            'max_discharge_c': 75.0,   # Tattu 75C rating
            'publish_rate': 50.0,      # Battery state publish rate (Hz)
        }],
    )

    open_loop_visualizer_node = Node(
        package='Mini_Dog_URDF_description',
        executable='open_loop_visualizer_node',
        name='open_loop_visualizer_node',
        output='screen'
    )

    # Start battery sag node after position controller is ready
    start_battery_sag_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_position_controller,
            on_exit=[battery_sag_node],
        )
    )

    # ================================================================
    # Assemble Launch Description
    # ================================================================
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_robot,
        gz_ros_bridge,
        start_joint_state_broadcaster_after_spawn,
        start_position_controller_after_broadcaster,
        start_battery_sag_after_controller,
        open_loop_visualizer_node
    ])
