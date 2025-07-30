import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from spike_msgs.msg import Spike
from collections import deque

class SpikeDetectorNode(Node):
    """
    ROS 2 node for detecting height spikes in odometry data.
    
    A spike is defined as a sharp drop in Z-coordinate followed by a recovery.
    This pattern typically occurs when a handheld scanner passes over obstacles
    like cables, small objects, or surface irregularities.
    
    Algorithm:
    1. Maintains a sliding window of recent Z-coordinates
    2. Detects when Z drops more than threshold compared to previous value
    3. Waits for recovery (Z returns close to reference level)
    4. Publishes spike message with location and timestamp
    """
    
    def __init__(self):
        super().__init__('spike_detector_node')
        
        # Declare ROS parameters with default values
        # These will be overridden by YAML config files passed from launch
        self.declare_parameter('z_drop_threshold', 0.1)  # meters - fallback default
        self.declare_parameter('sliding_window_size', 5)   # number of samples - fallback default
        
        # Get parameter values - these should come from the YAML config file
        self.z_drop_threshold = self.get_parameter('z_drop_threshold').get_parameter_value().double_value
        self.window_size = self.get_parameter('sliding_window_size').get_parameter_value().integer_value
        
        # Validate parameters and provide warnings if using defaults
        if self.z_drop_threshold <= 0.0:
            self.get_logger().warn('Invalid z_drop_threshold, using default: 0.05m')
            self.z_drop_threshold = 0.05
        
        if self.window_size <= 0:
            self.get_logger().warn('Invalid sliding_window_size, using default: 7')
            self.window_size = 7
        
        # Log the active configuration for debugging
        self.get_logger().info('Spike detector initialized with configuration:')
        self.get_logger().info(f'  z_drop_threshold: {self.z_drop_threshold} [m]')
        self.get_logger().info(f'  sliding_window_size: {self.window_size}')
        
        # Log which sensitivity profile is being used (if we can determine it)
        if self.z_drop_threshold == 0.02 and self.window_size == 3:
            self.get_logger().info('  Profile: HIGH sensitivity (detects small obstacles)')
        elif self.z_drop_threshold == 0.05 and self.window_size == 7:
            self.get_logger().info('  Profile: MEDIUM sensitivity (balanced detection)')
        elif self.z_drop_threshold == 0.15 and self.window_size == 10:
            self.get_logger().info('  Profile: LOW sensitivity (major obstacles only)')
        elif self.z_drop_threshold == 0.10 and self.window_size == 5:
            self.get_logger().info('  Profile: DEFAULT sensitivity (general purpose)')
        else:
            self.get_logger().info('  Profile: CUSTOM sensitivity configuration')
        
        # Initialize spike detection state
        self.z_window = deque(maxlen=self.window_size)  # Circular buffer for Z values
        self.drop_detected = False      # Flag: currently tracking a potential spike
        self.drop_reference = None      # Z-coordinate before the drop occurred
        
        # Statistics for debugging
        self.total_messages = 0
        self.spikes_detected = 0
        
        # ROS 2 subscription to odometry topic
        # This topic should contain nav_msgs/Odometry messages with pose information
        self.subscription = self.create_subscription(
            Odometry,
            '/state_optimizer/odometry_corrected',  # Topic from handheld mobile scanner
            self.odom_callback,
            10  # Queue size
        )
        
        # ROS 2 publisher for detected spikes
        # Publishes custom Spike messages containing timestamp and 3D position
        self.publisher = self.create_publisher(Spike, '/spike', 10)
        
        # Timer for periodic statistics reporting
        self.stats_timer = self.create_timer(30.0, self.report_statistics)  # Every 30 seconds
        
        self.get_logger().info('Spike detector node started and ready')

    def odom_callback(self, msg):
        """
        Callback function for processing odometry messages.
        
        Implements the spike detection algorithm:
        1. Extract current position from odometry message
        2. Add Z-coordinate to sliding window
        3. Check for drop condition (Z decreases significantly)
        4. Check for recovery condition (Z returns to near reference level)
        5. Publish spike message when complete spike pattern is detected
        
        Args:
            msg (nav_msgs.msg.Odometry): Incoming odometry message
        """
        # Extract position data from odometry message
        z = msg.pose.pose.position.z  # Current height
        x = msg.pose.pose.position.x  # Current X position
        y = msg.pose.pose.position.y  # Current Y position
        stamp = msg.header.stamp      # Message timestamp
        
        # Update statistics
        self.total_messages += 1
        
        # Add current Z value to sliding window
        self.z_window.append(z)
        
        # Wait until window is full before starting detection
        # This ensures we have enough history for stable comparison
        if len(self.z_window) < self.window_size:
            return
        
        # PHASE 1: Detect spike drop
        # Compare current Z with previous Z value (z_window[-2])
        # If drop exceeds threshold, start tracking potential spike
        if not self.drop_detected and self.z_window[-2] - z > self.z_drop_threshold:
            self.drop_detected = True
            self.drop_reference = self.z_window[-2]  # Remember height before drop
            self.get_logger().info(f'Spike drop detected: {self.drop_reference:.3f}m → {z:.3f}m '
                                 f'(drop: {self.drop_reference - z:.3f}m, threshold: {self.z_drop_threshold}m)')
        
        # PHASE 2: Detect spike recovery
        # Check if Z has returned close to the reference level
        # Recovery threshold is half of drop threshold for hysteresis
        elif self.drop_detected and abs(z - self.drop_reference) < self.z_drop_threshold / 2:
            # Complete spike detected - create and publish spike message
            spike_msg = Spike()
            spike_msg.stamp = stamp  # When the spike was completed
            spike_msg.x = x         # Where the spike occurred (X)
            spike_msg.y = y         # Where the spike occurred (Y)
            spike_msg.z = z         # Final Z after recovery
            
            self.publisher.publish(spike_msg)
            self.spikes_detected += 1
            
            self.get_logger().info(f'Spike #{self.spikes_detected} completed and published at position: '
                                 f'({x:.2f}, {y:.2f}, {z:.2f})')
            
            # Reset detection state for next spike
            self.drop_detected = False
            self.drop_reference = None
    
    def report_statistics(self):
        """Report periodic statistics about detection performance."""
        if self.total_messages > 0:
            detection_rate = (self.spikes_detected / self.total_messages) * 100
            self.get_logger().info(f'Statistics: {self.spikes_detected} spikes detected from '
                                 f'{self.total_messages} odometry messages '
                                 f'(detection rate: {detection_rate:.2f}%)')


def main(args=None):
    """
    Main entry point for the spike detector node.
    
    Initializes ROS 2, creates the node, and starts spinning to process messages.
    Handles graceful shutdown when interrupted.
    """
    rclpy.init(args=args)
    
    node = None
    try:
        node = SpikeDetectorNode()
        rclpy.spin(node)  # Process callbacks until shutdown
    except KeyboardInterrupt:
        pass  # Allow graceful shutdown with Ctrl+C
    finally:
        # Clean up resources
        if node is not None:
            node.get_logger().info('Shutting down spike detector node')
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
