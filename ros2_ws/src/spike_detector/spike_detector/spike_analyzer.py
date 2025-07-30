#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from spike_msgs.msg import Spike
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from datetime import datetime
import os
import json
from collections import Counter

class SpikeAnalyzer(Node):
    """
    ROS 2 node for analyzing detected spike data and generating reports.
    
    This node subscribes to the /spike topic and generates:
    1. Real-time statistics
    2. Spatial distribution plots
    3. Temporal analysis
    4. CSV reports for further analysis
    """
    
    def __init__(self):
        super().__init__('spike_analyzer')
        
        # Data storage
        self.spikes = []
        self.start_time = None
        
        # Analysis parameters
        self.declare_parameter('output_dir', 'spike_analysis')
        self.declare_parameter('save_plots', True)
        self.declare_parameter('save_csv', True)
        
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.save_plots = self.get_parameter('save_plots').get_parameter_value().bool_value
        self.save_csv = self.get_parameter('save_csv').get_parameter_value().bool_value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Subscribe to spike messages
        self.subscription = self.create_subscription(
            Spike,
            '/spike',
            self.spike_callback,
            10
        )
        
        # Timer for periodic analysis updates
        self.analysis_timer = self.create_timer(5.0, self.update_analysis)
        
        self.get_logger().info(f'Spike analyzer started. Output directory: {self.output_dir}')
    
    def spike_callback(self, msg):
        """Process incoming spike messages and store for analysis."""
        if self.start_time is None:
            self.start_time = msg.stamp
        
        # Convert ROS time to seconds since start
        time_since_start = (msg.stamp.sec - self.start_time.sec) + \
                          (msg.stamp.nanosec - self.start_time.nanosec) / 1e9
        
        spike_data = {
            'timestamp': msg.stamp,
            'time_since_start': time_since_start,
            'x': msg.x,
            'y': msg.y,
            'z': msg.z,
            'datetime': datetime.fromtimestamp(msg.stamp.sec)
        }
        
        self.spikes.append(spike_data)
        
        self.get_logger().info(f'Spike #{len(self.spikes)} detected at '
                              f'({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')
    
    def update_analysis(self):
        """Generate updated analysis reports."""
        if len(self.spikes) < 2:
            return
        
        # Generate real-time statistics
        self.generate_statistics()
        
        if self.save_plots:
            self.generate_plots()
        
        if self.save_csv:
            self.save_to_csv()
    
    def generate_statistics(self):
        """Generate and log statistical summary."""
        df = pd.DataFrame(self.spikes)
        
        stats = {
            'total_spikes': len(self.spikes),
            'detection_rate': len(self.spikes) / max(df['time_since_start'].iloc[-1], 1) * 60,  # spikes per minute
            'spatial_spread': {
                'x_range': (df['x'].min(), df['x'].max()),
                'y_range': (df['y'].min(), df['y'].max()),
                'z_range': (df['z'].min(), df['z'].max())
            },
            'z_statistics': {
                'mean': df['z'].mean(),
                'std': df['z'].std(),
                'min': df['z'].min(),
                'max': df['z'].max()
            }
        }
        
        # Save statistics to JSON
        with open(os.path.join(self.output_dir, 'spike_statistics.json'), 'w') as f:
            json.dump(stats, f, indent=2, default=str)
        
        self.get_logger().info(f'Analysis Update: {stats["total_spikes"]} spikes, '
                              f'{stats["detection_rate"]:.1f} spikes/min')
    
    def generate_plots(self):
        """Generate visualization plots."""
        if len(self.spikes) < 2:
            return
        
        df = pd.DataFrame(self.spikes)
        
        # Create figure with subplots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle(f'Spike Detection Analysis - {len(self.spikes)} Spikes Detected', fontsize=16)
        
        # 1. Spatial distribution (X-Y plot)
        scatter = ax1.scatter(df['x'], df['y'], c=df['z'], cmap='viridis', alpha=0.7)
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title('Spatial Distribution of Spikes')
        ax1.grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=ax1, label='Z Height (m)')
        
        # 2. Height distribution histogram
        ax2.hist(df['z'], bins=20, alpha=0.7, edgecolor='black')
        ax2.set_xlabel('Z Height (m)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Height Distribution of Spikes')
        ax2.grid(True, alpha=0.3)
        ax2.axvline(df['z'].mean(), color='red', linestyle='--', label=f'Mean: {df["z"].mean():.3f}m')
        ax2.legend()
        
        # 3. Temporal analysis
        ax3.plot(df['time_since_start'], df['z'], 'o-', alpha=0.7)
        ax3.set_xlabel('Time Since Start (s)')
        ax3.set_ylabel('Z Height (m)')
        ax3.set_title('Spike Heights Over Time')
        ax3.grid(True, alpha=0.3)
        
        # 4. Detection rate over time (sliding window)
        window_size = max(len(df) // 10, 5)
        if len(df) >= window_size:
            detection_rate = df['time_since_start'].rolling(window=window_size).apply(
                lambda x: window_size / (x.iloc[-1] - x.iloc[0]) * 60 if x.iloc[-1] != x.iloc[0] else 0
            )
            ax4.plot(df['time_since_start'], detection_rate, label='Detection Rate')
            ax4.set_xlabel('Time Since Start (s)')
            ax4.set_ylabel('Spikes per Minute')
            ax4.set_title('Detection Rate Over Time')
            ax4.grid(True, alpha=0.3)
            ax4.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'spike_analysis.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def save_to_csv(self):
        """Save spike data to CSV for external analysis."""
        df = pd.DataFrame(self.spikes)
        df.to_csv(os.path.join(self.output_dir, 'spike_data.csv'), index=False)
    
    def generate_final_report(self):
        """Generate comprehensive final report."""
        if not self.spikes:
            return
        
        df = pd.DataFrame(self.spikes)
        
        # Generate comprehensive statistics
        report = {
            'Analysis Summary': {
                'Total Spikes Detected': len(self.spikes),
                'Analysis Duration (s)': df['time_since_start'].iloc[-1],
                'Average Detection Rate (spikes/min)': len(self.spikes) / df['time_since_start'].iloc[-1] * 60,
                'Spatial Coverage': {
                    'X Range (m)': f"{df['x'].min():.2f} to {df['x'].max():.2f}",
                    'Y Range (m)': f"{df['y'].min():.2f} to {df['y'].max():.2f}",
                    'Z Range (m)': f"{df['z'].min():.2f} to {df['z'].max():.2f}"
                }
            },
            'Height Statistics': {
                'Mean Height (m)': df['z'].mean(),
                'Std Deviation (m)': df['z'].std(),
                'Min Height (m)': df['z'].min(),
                'Max Height (m)': df['z'].max(),
                'Median Height (m)': df['z'].median()
            }
        }
        
        # Save final report
        with open(os.path.join(self.output_dir, 'final_report.json'), 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        # Print summary to console
        self.get_logger().info("=== FINAL SPIKE DETECTION REPORT ===")
        self.get_logger().info(f"Total Spikes: {len(self.spikes)}")
        self.get_logger().info(f"Detection Rate: {len(self.spikes) / df['time_since_start'].iloc[-1] * 60:.1f} spikes/min")
        self.get_logger().info(f"Height Range: {df['z'].min():.3f}m to {df['z'].max():.3f}m")
        self.get_logger().info(f"Mean Height: {df['z'].mean():.3f}m ± {df['z'].std():.3f}m")


def main(args=None):
    rclpy.init(args=args)
    
    analyzer = None
    try:
        analyzer = SpikeAnalyzer()
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        if analyzer is not None:
            analyzer.generate_final_report()
            analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()