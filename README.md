# Spike Detector ROS 2 Package

## Usage

### Quick Start
```bash
# Place your bag file in ros2_ws/assets/cs3_office_1_traj_0.mcap
ros2 launch spike_detector spike_detector_with_bag.launch.py
```

### Sensitivity Levels

- **Default**: Balanced detection (5cm threshold, moderate smoothing) - DEFAULT
- **High**: Detects small obstacles (2cm threshold, quick response)
- **Medium**: Balanced detection (5cm threshold, moderate smoothing)
- **Low**: Only major obstacles (15cm threshold, high stability)

```bash
# Default sensitivity (5cm threshold)
ros2 launch spike_detector spike_detector_with_bag.launch.py

# High sensitivity (2cm threshold)
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=high

# Medium sensitivity (5cm threshold, same as default)
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=medium

# Low sensitivity (15cm threshold)
ros2 launch spike_detector spike_detector_with_bag.launch.py sensitivity:=low
```
