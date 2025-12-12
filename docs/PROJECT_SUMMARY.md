# Smart Campus Delivery TurtleBot3 - Project Summary

## Student Information
- **Project**: Individual Assignment - TurtleBot3 Automation with ROS2 and AI Tools
- **Course**: Smart Mobility Engineering Lab
- **Theme**: Smart Campus Delivery Robot
- **Date**: December 2025

## Project Overview

This project implements a comprehensive autonomous delivery robot system for smart campus environments. The TurtleBot3 robot autonomously delivers packages between campus buildings while detecting obstacles and docking at specific rooms using QR codes. The entire system is demonstrable without physical hardware using mock data.

## Theme: Smart Campus Delivery

**Concept**: An intelligent delivery robot that navigates between campus buildings (Engineering, Library, Cafeteria) to deliver packages, while avoiding obstacles (people, bikes, boxes) and automatically docking at room doors using QR codes.

**Key Features**:
- Automated environment setup
- Continuous health monitoring
- Building-to-building autonomous navigation
- Real-time obstacle detection
- QR code-based room identification and docking

## Assignment Requirements - Completion Status

### ✅ 1. Setup Automation (COMPLETED)
**Module**: `setup_automation/installer.py`

**What it does**:
- Automates installation of ROS2 Foxy and TurtleBot3 packages
- Creates and configures workspace structure
- Installs Python dependencies (YOLO, OpenCV, numpy, etc.)
- Configures environment variables in .bashrc
- One-command setup for entire delivery system

**Testing**: `./scripts/test_setup.sh`

**Key Functions**:
- `ensure_apt_repositories()` - Sets up ROS2 apt sources
- `install_ros_packages()` - Installs TurtleBot3 stack
- `install_python_packages()` - Installs AI/ML dependencies
- `automate_full_setup()` - Complete automation pipeline

---

### ✅ 2. Maintenance Automation (COMPLETED)
**Module**: `maintenance/monitor_node.py`

**What it does**:
- Monitors battery voltage and percentage continuously
- Tracks system diagnostics and errors
- Publishes alerts when battery is too low for delivery
- Prevents mid-route failures during campus operations
- Logs health data for analysis

**Testing**: `./scripts/test_maintenance.sh`

**Key Features**:
- **Critical Alert** (10.8V): Robot must return to charging station
- **Warning** (11.4V): Complete current delivery then recharge
- Reports every 30 seconds
- Publishes to `campus_delivery/alerts` and `campus_delivery/health_status`

**Topics**:
- Subscribes: `/battery_state`, `/diagnostics`, `/cmd_vel`
- Publishes: `campus_delivery/alerts`, `campus_delivery/health_status`

---

### ✅ 3. Navigation Automation (COMPLETED)
**Module**: `navigation/autonomy_manager.py`

**What it does**:
- Autonomous patrol between three campus buildings
- Waypoint-based navigation using Nav2 stack
- Building identification from coordinates
- Continuous loop for package pickup/delivery

**Testing**: `./scripts/test_navigation.sh`

**Route**:
1. **Building A (Engineering)**: x=0.0, y=0.0 - Package pickup
2. **Building B (Library)**: x=3.0, y=2.0 - Package delivery
3. **Building C (Cafeteria)**: x=6.0, y=0.5 - Package delivery

**Topics**:
- Publishes: `campus_delivery/navigation/status`, `campus_delivery/current_building`
- Subscribes: `campus_delivery/navigation/goal`

---

### ✅ 4. Object Detection Integration (COMPLETED)
**Module**: `perception/object_detection_node.py`

**What it does**:
- YOLO-based real-time obstacle detection
- Detects campus-specific obstacles for safe navigation
- Publishes detection results to ROS2 topics
- Provides RViz visualization markers

**Testing**: `./scripts/test_detection.sh`

**Detected Obstacles**:
- **person**: Students, faculty, pedestrians
- **bicycle**: Bikes blocking pathways
- **box** (via backpack/suitcase): Packages or large objects

**Topics**:
- Subscribes: `/camera/color/image_raw`
- Publishes: `campus_delivery/detections`, `campus_delivery/obstacle_alerts`

**Configuration**: 40% confidence threshold, campus-specific class filter

---

### ✅ 5. Custom Feature: QR-Docking System (COMPLETED)
**Module**: `custom_features/qr_follow_node.py`

**What it does**:
- Detects QR codes mounted on room doors
- Automatically identifies room (ROOM_A, ROOM_B, ROOM_C)
- Navigates to docking pose in front of correct room
- Enables precise package delivery to specific locations

**Testing**: `./scripts/test_qr_docking.sh`

**Room Mappings**:
- ROOM_A → Engineering Lab 101
- ROOM_B → Library Office 205
- ROOM_C → Faculty Lounge 310

**Docking Process**:
1. Camera scans for QR codes
2. QR detected and room identified
3. Robot approaches door while centering on QR code
4. Docks at specified distance (0.5m)
5. Ready for package handoff

**Topics**:
- Subscribes: `/camera/color/image_raw`
- Publishes: `campus_delivery/qr_detected`, `campus_delivery/docking_status`

---

## Technology Stack

### ROS2 & Robotics
- **ROS 2 Foxy** - Robot Operating System framework
- **Nav2** - Navigation stack for autonomous movement
- **SLAM Toolbox** - Simultaneous localization and mapping
- **TurtleBot3** - Mobile robot platform

### AI & Computer Vision
- **YOLOv8** (Ultralytics) - Real-time object detection
- **OpenCV** - Computer vision and QR code detection
- **cv_bridge** - ROS-OpenCV integration

### Python Libraries
- **rclpy** - ROS2 Python client library
- **numpy** - Numerical computing
- **PyYAML** - Configuration file parsing

---

## Mock Testing System

All modules work without physical TurtleBot3 hardware:

1. **test_setup.sh** - Simulates setup automation checks
2. **test_maintenance.sh** - Publishes mock battery data to `/battery_state`
3. **test_navigation.sh** - Simulates building-to-building navigation
4. **test_detection.sh** - Publishes simulated obstacle detections
5. **test_qr_docking.sh** - Simulates QR code detection and docking
6. **demo_campus_delivery.sh** - Complete system demonstration

Each script uses ROS2 topics to publish/subscribe mock data, demonstrating full functionality.

---

## Project Structure

```
turtlebot3_automation/
├── src/turtlebot3_automation/
│   ├── turtlebot3_automation/
│   │   ├── setup_automation/       # Module 1: Setup
│   │   ├── maintenance/            # Module 2: Health monitoring
│   │   ├── navigation/             # Module 3: Building navigation
│   │   ├── perception/             # Module 4: Obstacle detection
│   │   ├── custom_features/        # Module 5: QR-docking
│   │   └── utils/                  # Shared utilities
│   ├── config/                     # YAML configurations
│   ├── launch/                     # ROS2 launch files
│   └── tests/                      # Unit tests
├── scripts/                        # Mock test scripts
│   ├── test_setup.sh
│   ├── test_maintenance.sh
│   ├── test_navigation.sh
│   ├── test_detection.sh
│   ├── test_qr_docking.sh
│   └── demo_campus_delivery.sh     # Complete demo
└── README.md                       # Documentation
```

---

## How to Run

### Complete Demonstration
```bash
./scripts/demo_campus_delivery.sh
```

### Individual Module Tests
```bash
./scripts/test_setup.sh         # Test setup automation
./scripts/test_maintenance.sh   # Test health monitoring
./scripts/test_navigation.sh    # Test navigation
./scripts/test_detection.sh     # Test obstacle detection
./scripts/test_qr_docking.sh    # Test QR-docking
```

### Real Hardware (when available)
```bash
# Launch full system
ros2 launch turtlebot3_automation automation_bringup.launch.py

# Or individual modules
ros2 launch turtlebot3_automation maintenance.launch.py
ros2 launch turtlebot3_automation navigation_only.launch.py
ros2 launch turtlebot3_automation object_detection.launch.py
ros2 launch turtlebot3_automation qr_follow.launch.py
```

---

## Challenges & Solutions

### Challenge 1: No Physical Robot
**Problem**: Assignment requires demonstration without TurtleBot3 hardware

**Solution**: Created comprehensive mock testing system using ROS2 topics
- Mock scripts publish simulated sensor data
- Nodes process data identically to real hardware
- Complete functionality demonstration possible

### Challenge 2: Campus-Specific Object Detection
**Problem**: YOLO doesn't have a "box" class for packages

**Solution**: Used proxy classes for campus obstacles
- "backpack" and "suitcase" represent packages
- "person" for pedestrians
- "bicycle" for bikes
- Confidence threshold tuned for campus environment

### Challenge 3: Room Identification System
**Problem**: Need precise docking at specific rooms

**Solution**: QR code-based identification system
- QR codes on doors encode room IDs
- Robot detects, identifies, and docks automatically
- Supports multiple rooms (expandable)

### Challenge 4: Preventing Mid-Route Failures
**Problem**: Battery failures during delivery are unacceptable

**Solution**: Proactive health monitoring
- Continuous battery monitoring
- Two-tier alert system (warning/critical)
- Logging for predictive maintenance

---

## Lessons Learned

1. **Modular Design is Essential**
   - Separated concerns (setup, maintenance, navigation, perception, custom)
   - Easy to test, debug, and extend individual modules
   - Configuration files make tuning straightforward

2. **Mock Testing Enables Development**
   - No need for expensive hardware during development
   - Faster iteration cycles
   - Safer testing environment

3. **ROS2 Topic Architecture**
   - Clean separation of publishers/subscribers
   - Easy integration between modules
   - Standard message types promote interoperability

4. **AI Tools Accelerate Development**
   - YOLO provides robust object detection out-of-the-box
   - OpenCV enables QR code functionality
   - Pre-trained models save significant time

5. **Configuration Over Hard-Coding**
   - YAML configs make system adaptable
   - No code changes needed for different campuses
   - Easy parameter tuning

---

## Future Enhancements

1. **Multi-Robot Coordination**
   - Fleet management for multiple delivery robots
   - Load balancing between robots
   - Collision avoidance between robots

2. **Advanced Path Planning**
   - Dynamic re-routing based on obstacles
   - Time-optimized delivery scheduling
   - Weather-aware route selection

3. **Enhanced Security**
   - Package authentication system
   - Recipient verification via mobile app
   - Secure compartment with access control

4. **Machine Learning Improvements**
   - Custom training for campus-specific objects
   - Predictive maintenance based on usage patterns
   - Adaptive navigation based on traffic patterns

5. **User Interface**
   - Web dashboard for monitoring deliveries
   - Mobile app for delivery requests
   - Real-time tracking system

---

## Conclusion

This project successfully implements a complete smart campus delivery robot system meeting all assignment requirements. The modular architecture, comprehensive mock testing, and detailed documentation make it an excellent foundation for real-world deployment. The system demonstrates practical application of ROS2, autonomous navigation, computer vision, and AI in solving real campus logistics challenges.

**All assignment requirements have been met and exceeded with a functional, demonstrable system.**

---

## Video Demonstration Script

For the 2-5 minute video:

1. **Introduction** (30 seconds)
   - Project theme and purpose
   - Show project structure

2. **Setup Automation** (30 seconds)
   - Run test_setup.sh
   - Show automated checks

3. **Core Modules** (2 minutes)
   - Maintenance: Battery monitoring
   - Navigation: Building A→B→C
   - Detection: Obstacle detection
   - QR-Docking: Room delivery

4. **Complete Demo** (1 minute)
   - Run demo_campus_delivery.sh
   - Show all modules working together

5. **Conclusion** (30 seconds)
   - Summary of capabilities
   - Real-world applications

Total: 4-5 minutes
