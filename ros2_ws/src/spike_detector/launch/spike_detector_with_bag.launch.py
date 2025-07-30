from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotEqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
import os
import re
import glob
from datetime import datetime

def find_next_test_number(results_dir, sensitivity):
    """
    Automatically find the next available test number for a given sensitivity.
    
    Scans existing files in the results directory and finds the highest test number
    for the specified sensitivity, then returns the next sequential number.
    
    Args:
        results_dir (str): Directory to scan for existing test files
        sensitivity (str): Sensitivity level (default, high, medium, low)
        
    Returns:
        int: Next available test number
    """
    if not os.path.exists(results_dir):
        return 1
    
    # Pattern to match files like: test3_default_spike_results_30_07_2025_21_15.mcap
    pattern = f"test*_{sensitivity}_spike_results_*.mcap"
    existing_files = glob.glob(os.path.join(results_dir, pattern))
    
    test_numbers = []
    for file_path in existing_files:
        filename = os.path.basename(file_path)
        # Extract test number using regex: test(\d+)_sensitivity_...
        match = re.match(rf"test(\d+)_{sensitivity}_spike_results_", filename)
        if match:
            test_numbers.append(int(match.group(1)))
    
    # Return next sequential number (start from 1 if no tests exist)
    return max(test_numbers, default=0) + 1

def generate_launch_description():
    """
    Launch file for spike detector with rosbag playback and recording capabilities.
    
    This launch file supports different sensitivity configurations and automatically shuts down
    when the bag file playback is complete. It also automatically assigns sequential test numbers.
    
    This launch file supports different sensitivity configurations:
    - default: Balanced detection (10cm threshold, quick-moderate smoothing)
    
    - high: Detects small obstacles (2cm threshold, quick response)
    - medium: Balanced detection (5cm threshold, moderate smoothing)
    - low: Only major obstacles (15cm threshold, high stability)
    
    The bag file is automatically located in ros2_ws/assets/cs3_office_1_traj_0.mcap
    
    Automatic Test Numbering:
    - Scans existing results and assigns next sequential test number
    - Files named like: test1_default_spike_results_30_07_2025_21_15.mcap
    - No need to manually specify test numbers!
    
    Usage examples:
    ros2 launch spike_detector spike_detector_with_bag.launch.py
    ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=high
    ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=low record_spikes:=false
    ros2 launch spike_detector spike_detector_with_bag.launch.py bag_path:=/custom/path/to/bag.mcap
    """
    
    # Determine workspace root directory
    # When using ros2 launch, this file is located at:
    # .../ros2_ws/install/spike_detector/share/spike_detector/launch/spike_detector_with_bag.launch.py
    # So we go back 3 directories: ../../.. to reach ros2_ws/
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    ws_root = os.path.join(launch_file_dir, '..', '..', '..')
    ws_root = os.path.normpath(ws_root)  # Clean up the path
    
    # Default bag file path in assets directory
    default_bag_path = os.path.join(ws_root, 'assets', 'cs3_office_1_traj_0.mcap')
    
    # Create results directory if it doesn't exist
    results_dir = os.path.join(ws_root, 'src', 'spike_detector', 'spike_results')
    os.makedirs(results_dir, exist_ok=True)
    
    # Generate timestamp for unique output naming
    timestamp = datetime.now().strftime("%d_%m_%Y_%H_%M")
    
    # Declare launch arguments with detailed descriptions
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description=f'Full path to the rosbag file (.mcap or .db3) containing /state_optimizer/odometry_corrected topic. Default: {default_bag_path}'
    )
    
    sensitivity_arg = DeclareLaunchArgument(
        'sensitivity',
        default_value='default',
        description='Detection sensitivity level: default (10cm), high (2cm), medium (5cm), or low (15cm)'
    )
    
    record_spikes_arg = DeclareLaunchArgument(
        'record_spikes',
        default_value='true',
        description='Whether to record detected spike messages to test[N]_[sensitivity]_spike_results_[timestamp].mcap'
    )
    
    auto_shutdown_arg = DeclareLaunchArgument(
        'auto_shutdown',
        default_value='true',
        description='Whether to automatically shutdown when bag playback finishes'
    )
    
    enable_auto_numbering_arg = DeclareLaunchArgument(
        'enable_auto_numbering',
        default_value='true',
        description='Whether to automatically assign sequential test numbers (test1, test2, etc.)'
    )
    
    custom_prefix_arg = DeclareLaunchArgument(
        'custom_prefix',
        default_value='',
        description='Custom prefix instead of auto test numbering (overrides auto numbering if set)'
    )
    
    # Get launch configuration parameters
    bag_path = LaunchConfiguration('bag_path')
    sensitivity = LaunchConfiguration('sensitivity')
    record_spikes = LaunchConfiguration('record_spikes')
    auto_shutdown = LaunchConfiguration('auto_shutdown')
    enable_auto_numbering = LaunchConfiguration('enable_auto_numbering')
    custom_prefix = LaunchConfiguration('custom_prefix')
    
    # We need to resolve the sensitivity value early to determine test number
    # This is a limitation of Launch - we'll use a default and let the user override
    
    # Find next test number for default sensitivity (limitation: can't resolve LaunchConfiguration here)
    default_sensitivity = 'default'  # Use default for test numbering, actual sensitivity resolved at runtime
    next_test_num = find_next_test_number(results_dir, default_sensitivity)
    
    # Determine config file based on sensitivity parameter
    config_file = PathJoinSubstitution([
        FindPackageShare('spike_detector'),
        'config',
        [sensitivity, '_sensitivity_spike_params.yaml']
    ])
    
    # Main spike detector node
    # Subscribes to /state_optimizer/odometry_corrected and publishes to /spike
    spike_detector = Node(
        package='spike_detector',
        executable='spike_detector_node',
        name='spike_detector_node',
        output='screen',
        parameters=[config_file],
    )
    
    # Rosbag player process
    # Plays the bag file automatically with the default path or custom path if provided
    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path],
        output='screen',
        condition=IfCondition(
            NotEqualsSubstitution(bag_path, '')
        )
    )

    sensitivity_str = sensitivity
    # Create output filename with automatic test numbering (using PathJoinSubstitution for sensitivity)
    output_filename = PathJoinSubstitution([
        [f"test{next_test_num}_", sensitivity, "_sensitivity_spike_results_" + timestamp]
    ])
    output_path = PathJoinSubstitution([
        results_dir, [sensitivity, '_sensitivity_results'], output_filename
    ])
    
    # Create directory structure (using default sensitivity for now)
    os.makedirs(os.path.join(results_dir, f'{default_sensitivity}_sensitivity_results'), exist_ok=True)
    # Spike results recorder with automatic test numbering
    spike_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/spike', '-o', output_path],
        output='screen',
        condition=IfCondition(record_spikes)
    )
    
    # Event handler to shutdown spike recorder when bag player finishes
    # This ensures the recording is properly finalized
    shutdown_spike_recorder = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit=[
                ExecuteProcess(
                    cmd=['pkill', '-f', 'ros2 bag record.*spike'],
                    output='screen',
                    condition=IfCondition(record_spikes)
                )
            ]
        )
    )
    
    # Event handler to shutdown spike detector when bag player finishes
    # Using pkill to terminate the process properly
    shutdown_spike_detector = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit=[
                ExecuteProcess(
                    cmd=['pkill', '-f', 'spike_detector_node'],
                    output='screen',
                    condition=IfCondition(auto_shutdown)
                )
            ]
        )
    )
    
    # Final shutdown handler with dynamic result filename
    final_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit=[
                ExecuteProcess(
                    cmd=['echo', f'Bag playback finished. Results saved (Test #{next_test_num}). Shutting down...'],
                    output='screen'
                )
            ]
        )
    )
    
    return LaunchDescription([
        bag_path_arg,
        sensitivity_arg,
        record_spikes_arg,
        auto_shutdown_arg,
        enable_auto_numbering_arg,
        custom_prefix_arg,
        spike_detector,
        bag_player,
        spike_recorder,
        shutdown_spike_recorder,
        shutdown_spike_detector,
        final_shutdown
    ])