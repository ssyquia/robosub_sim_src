# Robosub Data Collection Package

A comprehensive ROS2 package for collecting camera images, controlling gate positions, and managing robosub poses in the simulation environment.

## Features

### ✅ **All Required Functionality:**
- 🚪 **Change gate position and rotation** - Control gate using pose/twist commands
- 📍 **Publish pose/twist messages** - Command interface for position control
- 📊 **Collect poses of robosub and gate** - Synchronized data collection
- 📷 **Capture camera output + poses** - Image and pose data synchronized
- 🤖 **Make robosub static** - Stop robosub movement during data collection
- 🎯 **Access camera output** - Process and save images from simulation

## Package Structure

```
robosub_data_collection/
├── robosub_data_collection/
│   ├── __init__.py
│   ├── comprehensive_data_collector.py  # Main data collector
│   ├── simple_camera_collector.py       # Just camera collection
│   └── gate_controller.py               # Gate position control
├── package.xml
├── setup.py
└── README.md
```

## Installation & Build

### **Step 1: Build the Package**
```bash
# In your dave_ws/src/robosub_sim_src directory
cd robosub_data_collection
colcon build --packages-select robosub_data_collection
source install/setup.bash
```

### **Step 2: Install Dependencies**
```bash
# Install CV Bridge if not already installed
sudo apt install ros-jazzy-cv-bridge
```

## Usage

### **Option 1: Simple Camera Collection (Works Now!)**
Just collects camera images - no pose data needed:

```bash
ros2 run robosub_data_collection simple_collector
```

**Output:**
- Saves images to `collected_images/` directory
- Format: `image_XXXXXX_timestamp.png`

### **Option 2: Comprehensive Data Collection**
Collects camera + poses + controls gate:

```bash
ros2 run robosub_data_collection data_collector
```

**Features:**
- ✅ Automatically cycles gate positions every 10 seconds
- ✅ Makes robosub static for consistent data
- ✅ Collects synchronized camera + pose data
- ✅ Saves to `comprehensive_data/` with metadata JSON

### **Option 3: Gate Controller Only**
Just control gate position independently:

```bash
ros2 run robosub_data_collection gate_controller
```

## Data Output

### **Simple Collector:**
```
collected_images/
├── image_000001_1234567890.123.png
├── image_000002_1234567890.456.png
└── ...
```

### **Comprehensive Collector:**
```
comprehensive_data/
├── images/
│   ├── image_000001_1234567890.123456.png
│   └── ...
└── metadata.json  # Contains all pose data + timestamps
```

### **Metadata JSON Structure:**
```json
{
  "collection_info": {
    "start_time": 1234567890.0,
    "total_data_points": 250,
    "images_saved": 250,
    "robosub_static": true,
    "gate_positions_used": [...]
  },
  "data": [
    {
      "timestamp": 1234567890.123456,
      "camera_available": true,
      "robosub_pose_available": false,
      "gate_pose_available": false,
      "image_filename": "image_000001_1234567890.123456.png",
      "gate_position_commanded": {"x": 5.0, "y": 0.0, "z": 2.0, "yaw": 0.0}
    }
  ]
}
```

## Complete Workflow

### **Terminal 1: Start Simulation**
```bash
# In Docker environment
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/jazzy/setup.bash
cd robosub_sim_src
colcon build && source install/setup.bash
cd robosub_worlds/models
export GAZEBO_MODEL_PATH=$PWD:$GAZEBO_MODEL_PATH
ros2 launch high_level_robosub dave_sim.launch.py namespace:='high_level_robosub' world_name:=pool paused:=false
```

### **Terminal 2: Run Data Collection**
```bash
# In Docker environment
cd robosub_sim_src
source install/setup.bash
ros2 run robosub_data_collection data_collector
```

## Functions Provided

### **ComprehensiveDataCollector Class:**
- `set_gate_position(x, y, z, yaw)` - Move gate to specific position
- `cycle_gate_position()` - Cycle through predefined positions
- `set_robosub_static(True/False)` - Control robosub movement
- `get_both_poses()` - Returns (robosub_pose, gate_pose) tuple
- `get_camera_output()` - Returns (metadata, cv_image) tuple
- `capture_synchronized_data()` - Get camera + poses at same time

### **Return Data Formats:**
```python
# Poses returned as geometry_msgs/Pose objects
robosub_pose, gate_pose = collector.get_both_poses()

# Camera data returned as dict + OpenCV image
metadata, cv_image = collector.get_camera_output()
# metadata = {'timestamp', 'height', 'width', 'encoding', 'frame_id'}
# cv_image = numpy array (height, width, channels)

# Synchronized data as dictionary
data_point = collector.capture_synchronized_data()
```

## Topics Used

### **Subscribed Topics:**
- `/robosub/camera/simulated_image` - Camera images (sensor_msgs/Image)
- `/model/high_level_robosub/pose` - Robosub pose (geometry_msgs/Pose)
- `/model/gate/pose` - Gate pose (geometry_msgs/Pose)

### **Published Topics:**
- `/model/gate/pose` - Gate position commands (geometry_msgs/Pose)
- `/model/gate/twist` - Gate movement commands (geometry_msgs/Twist)
- `/model/high_level_robosub/twist` - Robosub movement control (geometry_msgs/Twist)
- `/data_collector/collecting` - Collection status (std_msgs/Bool)

## Troubleshooting

### **No Camera Data:**
```bash
# Check if camera topic exists
ros2 topic list | grep camera
ros2 topic echo /robosub/camera/simulated_image --max-count 1
```

### **No Pose Data:**
```bash
# Check available pose topics
ros2 topic list | grep pose
# If no pose topics, only use simple collector
```

### **Build Issues:**
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --packages-select robosub_data_collection
```

## Tips for Data Collection

1. **Start simple:** Use `simple_collector` first to ensure camera works
2. **Check topics:** Verify what topics are available before running comprehensive collector
3. **Static robosub:** The comprehensive collector automatically makes robosub static
4. **Gate positions:** 5 predefined positions cycle automatically every 10 seconds
5. **Data syncing:** All timestamps are aligned for easy analysis 