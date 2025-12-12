# Smart Campus Delivery TurtleBot3 (ROS 2)

**Individual Assignment Project - Smart Mobility Engineering Lab**

An intelligent autonomous delivery robot system for smart campus environments using TurtleBot3 and ROS 2 Foxy (Ubuntu 20.04). The system enables autonomous package delivery between campus buildings with obstacle detection, health monitoring, and QR-based room docking capabilities. Fully functional with mock data for demonstration without physical hardware.

## Project Theme: Smart Campus Delivery Robot

This project implements a complete delivery robot automation suite designed for campus environments where a TurtleBot3 autonomously delivers packages between buildings while avoiding obstacles and docking at specific rooms using QR codes.

## Features

- **Setup Automation** – One-command environment preparation for ROS2 workspace and TurtleBot3 packages
- **Maintenance Automation** – Continuous health monitoring to prevent delivery failures (battery, diagnostics, alerts)
- **Navigation Automation** – Autonomous patrol between Building A → Building B → Building C with waypoint navigation
- **Object Detection** – Real-time detection of campus obstacles (person, bike, box) using YOLO simulation
- **Custom QR-Docking** – Automatic room detection and docking when QR codes (ROOM_A, ROOM_B, etc.) are scanned
- **Mock Testing** – Complete functionality demonstration without physical robot hardware

## Demo Video

Watch the TurtleBot3 autonomous navigation in action with RViz visualization and Gazebo simulation:

![Campus Delivery Demo](https://github.com/ShohruhShokulov/turtlebot3_automation/blob/main/demo/Demo.mp4)

**Video Location:** `demo/Demo.mp4`

The demo showcases:
- Autonomous navigation between campus waypoints
- Real-time path planning and obstacle avoidance
- RViz visualization of robot pose, map, and trajectory
- Gazebo simulation environment
- Nav2 navigation stack in action

## System Architecture

### How It Works
```
Campus Buildings (A → B → C)
         ↓
   TurtleBot3 Delivery Robot
         ↓
    ┌────────────────────────────────────┐
    │  Smart Campus Delivery System      │
    │                                    │
    │  1. Setup Automation               │
    │     → Auto-install ROS2 packages   │
    │     → Workspace configuration      │
    │                                    │
    │  2. Maintenance Monitor            │
    │     → Battery health checks        │
    │     → Prevent mid-route failures   │
    │                                    │
    │  3. Navigation Manager             │
    │     → Building A → B → C patrol    │
    │     → Autonomous waypoint routing  │
    │                                    │
    │  4. Object Detection               │
    │     → Detect: person, bike, box    │
    │     → Obstacle avoidance           │
    │                                    │
    │  5. QR-Docking System              │
    │     → Scan room QR codes           │
    │     → Auto-dock for delivery       │
    └────────────────────────────────────┘
```

## Quick Start (From Zero)

### 1. Clone and Setup Environment

```bash
# Navigate to your project directory
cd /home/shohruh/University/Smart_Mobility_Engineering_Lab/turtlebot3_automation

# Create conda environment with Python 3.8 (required for ROS Foxy)
conda create -n ros-foxy python=3.8 -y
conda activate ros-foxy

# Install Python dependencies
pip install -r requirements.txt
```

### 2. Install ROS Foxy System Packages

```bash
# Update package lists
sudo apt update

# Install ROS Foxy desktop and TurtleBot3 packages
sudo apt install -y ros-foxy-desktop ros-foxy-turtlebot3* ros-foxy-nav2* ros-foxy-slam-toolbox ros-foxy-vision-msgs

# Install additional build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

### 3. Build the Application

```bash
# Initialize rosdep
sudo rosdep init
rosdep update

# Build the package
colcon build --symlink-install
source install/setup.bash
```

### 4. Run the Campus Delivery System

#### Complete Demonstration (Mock Mode - No Hardware Required)
```bash
# Run the complete campus delivery demonstration
./scripts/demo_campus_delivery.sh
```
This script demonstrates all five modules working together in sequence.

#### Individual Components (Mock Testing)

```bash
# Test 1: Setup Automation
./scripts/test_setup.sh

# Test 2: Maintenance Health Monitoring
./scripts/test_maintenance.sh

# Test 3: Campus Navigation (Building A → B → C)
./scripts/test_navigation.sh

# Test 4: Object Detection (person, bike, box)
./scripts/test_detection.sh

# Test 5: QR-Docking System
./scripts/test_qr_docking.sh
```

#### Launch Individual Nodes (For Real Hardware)
```bash
# Maintenance monitoring only
ros2 launch turtlebot3_automation maintenance.launch.py

# Navigation patrol only
ros2 launch turtlebot3_automation navigation_only.launch.py

# Object detection only
ros2 launch turtlebot3_automation object_detection.launch.py

# QR-docking only
ros2 launch turtlebot3_automation qr_follow.launch.py

# Full system launch
ros2 launch turtlebot3_automation automation_bringup.launch.py
```

## Module Descriptions

### 1. Setup Automation
**Purpose:** Automatically prepares the entire ROS2 environment for campus delivery operations.

**What it does:**
- Creates and configures the ROS2 workspace
- Installs TurtleBot3 packages and dependencies
- Sets up environment variables
- Builds the entire system with one command
- Validates installation success

**How to test:**
```bash
./scripts/test_setup.sh
```

### 2. Maintenance Automation
**Purpose:** Performs periodic health checks to ensure the delivery robot doesn't fail mid-route.

**What it monitors:**
- Battery voltage and percentage
- System diagnostics and errors
- Movement commands
- Publishes alerts when critical issues detected

**Topics:**
- Subscribes: `/battery_state`, `/diagnostics`, `/cmd_vel`
- Publishes: `campus_delivery/alerts`, `campus_delivery/health_status`

**How to test:**
```bash
# Terminal 1: Launch the maintenance node
ros2 launch turtlebot3_automation maintenance.launch.py

# Terminal 2: Run mock battery data
./scripts/test_maintenance.sh
```

### 3. Navigation Automation  
**Purpose:** Controls autonomous movement between campus buildings.

**What it does:**
- Patrols between Building A → Building B → Building C
- Uses predefined waypoints for each building
- Handles obstacle avoidance and path planning
- Continuous loop patrol for package pickup/delivery

**Waypoints:**
- Building A (Engineering): x=0.0, y=0.0
- Building B (Library): x=3.0, y=2.0  
- Building C (Cafeteria): x=6.0, y=0.5

**Topics:**
- Publishes: `campus_delivery/navigation/status`, `campus_delivery/current_building`
- Subscribes: `campus_delivery/navigation/goal`

**How to test:**
```bash
./scripts/test_navigation.sh
```

### 4. Object Detection Integration
**Purpose:** Detects obstacles on campus delivery routes for safety.

**What it detects:**
- **Person**: Students/faculty on pathways
- **Bike**: Bicycles that may obstruct route
- **Box**: Other packages or obstacles

**Technology:** YOLO-based detection (simulated for mock testing)

**Topics:**
- Subscribes: `/camera/color/image_raw`
- Publishes: `campus_delivery/detections`, `campus_delivery/obstacle_alerts`

**How to test:**
```bash
# Terminal 1: Launch detection node
ros2 launch turtlebot3_automation object_detection.launch.py

# Terminal 2: Simulate camera feed with obstacles
./scripts/test_detection.sh
```

### 5. Custom QR-Docking Feature
**Purpose:** Enables automatic room identification and docking for package delivery.

**How it works:**
- Scans QR codes mounted on room doors
- QR codes contain room identifiers (ROOM_A, ROOM_B, ROOM_C, etc.)
- When correct room detected, robot navigates to docking pose
- Completes delivery by positioning in front of door

**Example QR codes:**
- ROOM_A → Engineering Lab 101
- ROOM_B → Library Office 205  
- ROOM_C → Faculty Lounge 310

**Topics:**
- Subscribes: `/camera/color/image_raw`
- Publishes: `campus_delivery/qr_detected`, `campus_delivery/docking_status`

**How to test:**
```bash
./scripts/test_qr_docking.sh
```

## Repository Layout

```
turtlebot3_automation/
├── src/turtlebot3_automation/
│   ├── turtlebot3_automation/           # Main Python package
│   │   ├── setup_automation/            # 1. Setup automation module
│   │   │   └── installer.py             #    → Auto-install & configure
│   │   ├── maintenance/                 # 2. Maintenance module  
│   │   │   └── monitor_node.py          #    → Health monitoring
│   │   ├── navigation/                  # 3. Navigation module
│   │   │   └── autonomy_manager.py      #    → Building A→B→C patrol
│   │   ├── perception/                  # 4. Object detection module
│   │   │   └── object_detection_node.py #    → person/bike/box detection
│   │   ├── custom_features/             # 5. QR-docking module
│   │   │   └── qr_follow_node.py        #    → Room identification
│   │   └── utils/                       # Shared utilities
│   ├── config/                          # YAML configuration files
│   │   ├── maintenance.yaml             # Battery thresholds, alerts
│   │   ├── navigation_params.yaml       # Building waypoints
│   │   ├── object_detection.yaml        # Detection classes
│   │   └── qr_follow.yaml               # QR room mappings
│   ├── launch/                          # ROS 2 launch files
│   │   ├── automation_bringup.launch.py # Full system launch
│   │   ├── maintenance.launch.py        # Health monitoring
│   │   ├── navigation_only.launch.py    # Campus patrol
│   │   ├── object_detection.launch.py   # Obstacle detection
│   │   └── qr_follow.launch.py          # Room docking
│   └── tests/                           # Unit tests
├── scripts/                             # Mock test scripts
│   ├── test_setup.sh                    # Test setup automation
│   ├── test_maintenance.sh              # Test health monitoring
│   ├── test_navigation.sh               # Test Building A→B→C
│   ├── test_detection.sh                # Test obstacle detection
│   ├── test_qr_docking.sh               # Test QR room docking
│   └── demo_campus_delivery.sh          # Complete demo (all modules)
├── requirements.txt                     # Python dependencies
└── README.md                            # This file
```

## Prerequisites

- **Ubuntu 20.04** (Focal Fossa)
- **Python 3.8** (via conda environment)
- **ROS 2 Foxy** (system installation)
- **TurtleBot3 packages** (Optional - works with mock data)
- **No physical robot required** - All tests run with simulated data

## Assignment Requirements Met

This project fulfills all requirements for the Smart Mobility Engineering Lab assignment:

### ✅ 1. Setup Automation
- Automated ROS2 and TurtleBot3 package installation
- Workspace creation and dependency resolution
- Environment configuration
- One-command setup script

### ✅ 2. Maintenance Automation
- Periodic battery health checks
- Diagnostic monitoring
- Alert system for critical issues
- Prevents delivery robot failures mid-route
- Logging mechanism

### ✅ 3. Navigation Automation
- Autonomous patrol between Building A → B → C
- Waypoint-based navigation system
- Simulated campus environment
- Obstacle avoidance integration

### ✅ 4. Object Detection Integration
- YOLO-based detection simulation
- Detects campus-specific obstacles: person, bike, box
- Publishes to ROS2 topics
- RViz visualization support

### ✅ 5. Custom Feature: QR-Docking
- QR code detection on room doors
- Automatic room identification (ROOM_A, ROOM_B, ROOM_C)
- Auto-docking for package delivery
- Integration with navigation system

### ✅ Additional Requirements
- Complete README documentation
- Mock testing scripts for demo without hardware  
- Video demonstration capability
- Modular, maintainable code structure

### Launch Files

- **Complete Navigation Stack** (SLAM + Navigation):
  ```bash
  ros2 launch turtlebot3_automation navigation_only.launch.py
  ```
  This launches the full Nav2 stack with SLAM Toolbox for mapping and autonomous navigation.

- **Maintenance Monitoring**:
  ```bash
  ros2 launch turtlebot3_automation maintenance.launch.py
  ```
  Monitors system health, battery, and diagnostics.

- **Object Detection**:
  ```bash
  ros2 launch turtlebot3_automation object_detection.launch.py
  ```
  Runs YOLOv8 inference on camera streams.

- **QR Code Following**:
  ```bash
  ros2 launch turtlebot3_automation qr_follow.launch.py
  ```
  Enables QR code detection and following behavior.

### Individual Nodes

```bash
# Run maintenance monitor
ros2 run turtlebot3_automation maintenance_monitor

# Run object detector
ros2 run turtlebot3_automation yolo_detector

# Run QR follower
ros2 run turtlebot3_automation qr_follow

# Run navigation manager
ros2 run turtlebot3_automation navigation_manager
```

## Configuration

### Maintenance Monitoring
- **Config file**: `src/turtlebot3_automation/config/maintenance.yaml`
- **Topics**:
  - Publishes alerts on `turtlebot3/alerts`
  - Reports every 30 seconds with battery, diagnostics, and system health
  - Monitors `/battery_state`, `/diagnostics`, `/cmd_vel`

### Object Detection
- **Config file**: `src/turtlebot3_automation/config/object_detection.yaml`
- **Model**: Uses YOLOv8n (`yolov8n.pt`) - downloads automatically on first run
- **Topics**:
  - Detections: `turtlebot3/perception/detections` (`Detection2DArray`)
  - Markers: `turtlebot3/perception/markers` (`MarkerArray`)
  - Labels: `turtlebot3/perception/labels` (`String`)

### Navigation
- **Config file**: `src/turtlebot3_automation/config/navigation_params.yaml`
- **Features**: SLAM mapping, autonomous waypoint navigation, obstacle avoidance
- **Components**: Nav2 stack with behavior trees, recovery behaviors, and lifecycle management

## Testing

### Test Scripts (No Hardware Required)

Use the provided bash scripts to test nodes without physical hardware:

```bash
# Test maintenance monitoring
./scripts/test_maintenance.sh

# Test object detection
./scripts/test_detection.sh

# Test QR following
./scripts/test_qr_follow.sh
```

Each script publishes simulated data to the corresponding ROS 2 topics.

### Monitor Topics
```bash
# View battery status
ros2 topic echo /battery_state

# View object detections
ros2 topic echo turtlebot3/perception/labels

# View maintenance alerts
ros2 topic echo turtlebot3/alerts

# View robot velocity
ros2 topic echo /cmd_vel

# Check topic publishing rates
ros2 topic hz /battery_state
ros2 topic hz turtlebot3/perception/detections
```

### Visualize in RViz
```bash
# Terminal 1: Run nodes
ros2 launch turtlebot3_automation object_detection.launch.py

# Terminal 2: Launch RViz
rviz2
```
In RViz:
1. Set Fixed Frame to `base_link`
2. Add → By topic → `/turtlebot3/perception/markers` → MarkerArray

### Run Unit Tests
Run parameter validation tests:
```bash
colcon test --packages-select turtlebot3_automation
```

## Troubleshooting

### Common Issues

1. **ROS Version Conflicts**: Ensure you're using ROS Foxy (not Rolling) with Python 3.8
2. **Conda Environment**: Always activate the `ros-foxy` conda environment before running
3. **Build Issues**: Run `colcon build --symlink-install` after config changes
4. **Navigation Manager**: Requires `ros-foxy-nav2-simple-commander` (install with sudo if needed)

### Test Script Issues

**Scripts not executable:**
```bash
chmod +x scripts/*.sh
```

**No topic data appearing:**
- Verify the node is running: `ros2 node list`
- Check topic exists: `ros2 topic list`
- Verify topic data: `ros2 topic echo /battery_state`

**Camera topics not available:**
- For object detection and QR follow: Verify camera is publishing
- Check: `ros2 topic hz /camera/color/image_raw`
- If testing without hardware, camera topics won't be available

### Environment Setup Verification

```bash
# Check conda environment
conda activate ros-foxy
python --version  # Should be 3.8.x

# Check ROS
source /opt/ros/foxy/setup.bash
ros2 --version

# Check package
source install/setup.bash
ros2 pkg list | grep turtlebot3_automation

# Verify all executables are available
ros2 pkg executables turtlebot3_automation
```

## Demo Examples

### Example 1: Test Maintenance System
```bash
# Terminal 1: Launch maintenance node
ros2 launch turtlebot3_automation maintenance.launch.py

# Terminal 2: Run battery test
./scripts/test_maintenance.sh
```

### Example 2: Test Object Detection
```bash
# Terminal 1: Launch detection node
ros2 launch turtlebot3_automation object_detection.launch.py

# Terminal 2: Publish fake detections
./scripts/test_detection.sh
```

### Example 3: Monitor Multiple Systems
```bash
# Terminal 1: Run maintenance node
ros2 launch turtlebot3_automation maintenance.launch.py

# Terminal 2: Monitor battery
ros2 topic echo /battery_state

# Terminal 3: Monitor alerts
ros2 topic echo turtlebot3/alerts

# Terminal 4: Run test script
./scripts/test_maintenance.sh
```

## ROS 2 Topics

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/battery_state` | `sensor_msgs/BatteryState` | Battery voltage and percentage |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |
| `turtlebot3/alerts` | `std_msgs/String` | Maintenance alerts and warnings |
| `turtlebot3/perception/detections` | `vision_msgs/Detection2DArray` | Object detection results |
| `turtlebot3/perception/labels` | `std_msgs/String` | Human-readable detection labels |
| `turtlebot3/perception/markers` | `visualization_msgs/MarkerArray` | RViz markers for detections |

### Subscribed Topics

| Node | Subscribes To | Purpose |
|------|---------------|---------|
| Maintenance Monitor | `/battery_state`, `/diagnostics`, `/cmd_vel` | System health monitoring |
| Object Detection | `/camera/color/image_raw` | Image processing |
| QR Follow | `/camera/color/image_raw` | QR code detection |

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     TurtleBot3 Hardware/Simulation              │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │   Battery    │  │    Camera    │  │  Motor Controllers  │   │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬──────────┘   │
└─────────┼──────────────────┼───────────────────────┼────────────┘
          │                  │                       │
          ▼                  ▼                       ▼
    /battery_state    /camera/image_raw         /cmd_vel
          │                  │                       ▲
          │                  │                       │
┌─────────┴──────────────────┴───────────────────────┴────────────┐
│                      ROS 2 Topic Layer                           │
└─────┬──────────┬──────────────┬─────────────────┬───────────────┘
      │          │              │                 │
      ▼          ▼              ▼                 ▼
┌──────────┐ ┌────────────┐ ┌──────────────┐ ┌────────────────┐
│Maintenance│ │  Object    │ │  QR Follow   │ │  Navigation    │
│ Monitor   │ │ Detection  │ │     Node     │ │    Manager     │
└─────┬─────┘ └─────┬──────┘ └──────┬───────┘ └────────────────┘
      │             │                │
      │             ▼                │
      │    turtlebot3/perception/*   │
      │                              │
      ▼                              │
turtlebot3/alerts                    │
                                     │
                                     ▼
                              Robot Movement
```

## Project Structure

```
turtlebot3_automation/
├── Subsystems
│   ├── maintenance/          # Health monitoring
│   ├── perception/           # Object detection (YOLO)
│   ├── navigation/           # Autonomous navigation (Nav2)
│   └── custom_features/      # QR code following
│
├── Support Systems
│   ├── utils/                # Logging, paths
│   └── setup_automation/     # Installation helpers
│
└── Configuration
    ├── config/               # YAML parameters
    ├── launch/               # Launch files
    └── tests/                # Unit tests
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Built on ROS 2 and Nav2 stack
- Uses YOLOv8 from Ultralytics
- Designed for ROBOTIS TurtleBot3 platform

## Contact
For questions or support, please open an issue on GitHub.
