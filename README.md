# ROS 2 Spike Detection System

A ROS 2 package for detecting height spikes in trajectory data from handheld mobile scanners. This system identifies sudden drops and recoveries in Z-coordinate data, typically caused by obstacles like cables, small objects, or surface irregularities.

## Features

- **Multi-sensitivity Detection**: Configurable sensitivity levels (default, high, medium, low)
- **Automatic Test Numbering**: Sequential test numbering for organized data collection
- **Organized Output Structure**: Results sorted by sensitivity level in dedicated directories
- **Real-time Analysis**: Live spike detection and recording
- **Automated Shutdown**: Clean termination when bag playback completes
- **Timestamped Results**: Unique output files with timestamps

## Package Structure

```
├── spike_detector/               # Main detection package
│   ├── config/                  # Parameter configuration files
│   │   ├── default_sensitivity_spike_params.yaml
│   │   ├── high_sensitivity_spike_params.yaml
│   │   ├── medium_sensitivity_spike_params.yaml
│   │   └── low_sensitivity_spike_params.yaml
│   ├── launch/                  # Launch files
│   │   └── spike_detector_with_bag.launch.py
│   ├── spike_detector/          # Python source code
│   │   ├── spike_detector_node.py
│   │   └── spike_analyzer.py
│   ├── spike_results/           # Auto-generated results directory
│   │   ├── default_sensitivity_results/
│   │   ├── high_sensitivity_results/
│   │   ├── medium_sensitivity_results/
│   │   └── low_sensitivity_results/
│   ├── package.xml
│   └── setup.py
└── spike_msgs/                  # Custom message definitions
    ├── msg/
    │   └── Spike.msg
    ├── CMakeLists.txt
    └── package.xml
```

## Installation

### Prerequisites

- ROS 2 (tested with Jazzy)
- colcon build tools

### Dependencies

```bash
sudo apt install python3-matplotlib python3-pandas python3-numpy
```

### Build Instructions

```bash
# Clone the repository
git clone https://github.com/yourusername/ros2-spike-detection.git
cd ros2-spike-detection

# Build the packages
colcon build --packages-select spike_msgs spike_detector
source install/setup.bash
```

## Usage

### Basic Spike Detection

```bash
# Run with default sensitivity (configurable threshold)
ros2 launch spike_detector spike_detector_with_bag.launch.py

# Run with high sensitivity (2cm threshold)
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=high

# Run with custom bag file
ros2 launch spike_detector spike_detector_with_bag.launch.py bag_path:=/path/to/your/bag.mcap

# Disable recording (analysis only)
ros2 launch spike_detector spike_detector_with_bag.launch.py record_spikes:=false

# Disable auto-shutdown (manual control)
ros2 launch spike_detector spike_detector_with_bag.launch.py auto_shutdown:=false
```

### Sensitivity Configurations

| Sensitivity | Threshold | Window Size | Use Case |
|-------------|-----------|-------------|----------|
| **high**    | 2cm       | 3 samples   | Small obstacles (cables, debris) |
| **medium**  | 5cm       | 7 samples   | Balanced detection |
| **default** | 10cm      | 5 samples   | Same as default |
| **low**     | 15cm      | 10 samples  | Major obstacles only |

### Automatic Test Numbering

The system automatically assigns sequential test numbers for each sensitivity level:

- **First run**: Creates `test1_default_spike_results_30_07_2025_21_15.mcap`
- **Second run**: Creates `test2_default_spike_results_30_07_2025_21_20.mcap`
- **High sensitivity**: Creates `test1_high_spike_results_30_07_2025_21_25.mcap`
- **Continue default**: Creates `test3_default_spike_results_30_07_2025_21_30.mcap`

### Output Structure

Results are automatically organized by sensitivity level:

```
spike_results/
├── default_sensitivity_results/
│   ├── test1_default_spike_results_30_07_2025_21_15.mcap
│   ├── test2_default_spike_results_30_07_2025_21_20.mcap
│   ├── test3_default_spike_results_30_07_2025_21_30.mcap
│   └── ...
├── high_sensitivity_results/
│   ├── test1_high_spike_results_30_07_2025_21_25.mcap
│   ├── test2_high_spike_results_30_07_2025_21_35.mcap
│   └── ...
├── medium_sensitivity_results/
│   └── ...
└── low_sensitivity_results/
    └── ...
```

## Input Data Format

The system expects ROS 2 bag files containing:

- **Topic**: `/state_optimizer/odometry_corrected`
- **Message Type**: `nav_msgs/Odometry`
- **Coordinate System**: Standard ROS coordinate frame (Z-up)

## Algorithm Details

### Spike Detection Algorithm

1. **Sliding Window**: Maintains a configurable window of recent Z-coordinates
2. **Drop Detection**: Identifies when Z decreases by more than the threshold
3. **Recovery Detection**: Waits for Z to return close to the reference level
4. **Spike Publication**: Publishes complete spike data with position and timestamp

### Real-time Feedback

During detection, the system provides live feedback:

```
[spike_detector_node]: Spike drop detected: -0.160m → -0.285m (drop: 0.124m, threshold: 0.1m)
[spike_detector_node]: Spike #1 completed and published at position: (0.62, 3.48, -0.20)
[spike_detector_node]: Statistics: 1 spikes detected from 595 odometry messages (detection rate: 0.17%)
```

### Message Output

```yaml
# spike_msgs/Spike
std_msgs/Header header
float64 x          # X position where spike occurred
float64 y          # Y position where spike occurred  
float64 z          # Z position after recovery
builtin_interfaces/Time stamp  # Timestamp of spike completion
```

## Launch Arguments

### Core Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `bag_path` | `assets/cs3_office_1_traj_0.mcap` | Path to input bag file |
| `sensitivity` | `default` | Detection sensitivity level |
| `record_spikes` | `true` | Whether to record spike messages |
| `auto_shutdown` | `true` | Auto-shutdown when bag finishes |

### Advanced Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_auto_numbering` | `true` | Enable automatic test numbering |
| `custom_prefix` | `''` | Custom prefix (overrides auto numbering) |

### Usage Examples

```bash
# Basic usage with auto-numbering
ros2 launch spike_detector spike_detector_with_bag.launch.py

# High sensitivity batch testing
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=high
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=high
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=high
# Creates: test1_high_..., test2_high_..., test3_high_...

# Compare all sensitivity levels
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=high
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=default  
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=low

```


## Results Analysis

### Viewing Results

```bash
# Play back recorded spikes
ros2 bag play spike_results/default_sensitivity_results/test1_default_spike_results_30_07_2025_21_15.mcap

# List spike messages
ros2 bag info your_spike_results.mcap
```

### Batch Analysis

The organized directory structure makes batch analysis easy:

```bash
# Analyze all high sensitivity tests
for bag in spike_results/high_sensitivity_results/*.mcap; do
    echo "Processing: $bag"
    ros2 bag info "$bag"
done

# Compare sensitivity levels
ls -la spike_results/*/test1_*  # Compare first test of each sensitivity
```

## System Behavior

### Automatic Shutdown Process

1. **Bag playback completes**
2. **Spike recorder terminated** with `pkill`
3. **Spike detector node terminated** with `pkill`
4. **Final status message** shows results location and test number
5. **Launch system exits** cleanly

### Test Numbering Logic

- Scans existing files in sensitivity-specific directory
- Extracts test numbers using regex: `test(\d+)_{sensitivity}_spike_results_`
- Assigns next sequential number (starting from 1)
- Independent numbering for each sensitivity level

### Resetting Test Numbers

```bash
# Remove all results to restart numbering
rm -rf src/spike_detector/spike_results/

# Remove specific sensitivity results
rm -rf src/spike_detector/spike_results/high_sensitivity_results/
```

## Contact

- **Author**: Raz Turgeman
- **Email**: Raz.Turgeman97@gmail.com
