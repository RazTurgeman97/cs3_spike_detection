from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('spike_detector')
    config_file = os.path.join(pkg_share, 'config', 'spike_params.yaml')
    
    # Get the workspace root (parent of src directory)
    current_dir = os.getcwd()
    if 'src' in current_dir:
        ws_root = current_dir.split('src')[0].rstrip('/')
    else:
        ws_root = current_dir
    
    # Spike detector node
    spike_detector = Node(
        package='spike_detector',
        executable='spike_detector_node',
        name='spike_detector_node',
        output='screen',
        parameters=[config_file]
    )
    
    # Rosbag player - modify this path to your bag file location
    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', 
             # Replace with your actual bag path or use a parameter
             '/path/to/your/bag/cs3_office_1_traj_0.mcap'],
        output='screen'
    )
    
    # Record spike results to workspace root
    spike_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/spike', '-o', 
             os.path.join(ws_root, 'spike_results')],
        output='screen'
    )
    
    return LaunchDescription([
        spike_detector,
        bag_player,
        spike_recorder
    ])
