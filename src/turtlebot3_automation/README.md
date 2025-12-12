# Smart Campus Delivery TurtleBot3 - ROS 2 Package

This ROS 2 package implements an autonomous campus delivery robot system for Smart Mobility Engineering Lab.

## System Overview

The package provides five integrated automation modules:

1. **Setup Automation** - One-command environment preparation and ROS2 workspace setup
2. **Maintenance Automation** - Continuous health monitoring to prevent mid-route delivery failures  
3. **Navigation Automation** - Autonomous patrol between campus buildings (A → B → C)
4. **Object Detection** - Real-time obstacle detection (person, bike, box) for safe navigation
5. **QR-Docking System** - Automatic room identification and docking for package delivery

## Mock Testing

All modules can be tested without physical TurtleBot3 hardware using provided mock scripts in `scripts/` directory. Perfect for demonstration and development.

## Installation

Install within a ROS2 Foxy colcon workspace. See main README.md for complete setup instructions.
