"""
Campus Delivery Navigation Manager

This module controls autonomous movement between campus buildings for 
package delivery operations. The robot patrols Building A → B → C
using predefined waypoints for pickup and delivery locations.
"""

from __future__ import annotations

from collections import deque
from math import cos, sin
from typing import Deque, Dict, Iterable, Optional

import sys

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from ..utils.logging import configure_logging

try:  # pragma: no cover - optional dependency at runtime
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:  # pragma: no cover - fallback when dependency missing
    BasicNavigator = None  # type: ignore
    TaskResult = None  # type: ignore


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw to a quaternion (z rotation)."""
    q = Quaternion()
    q.z = sin(yaw / 2.0)
    q.w = cos(yaw / 2.0)
    return q


class CampusDeliveryNavigationManager(Node):
    """
    Smart Campus Delivery - Navigation System
    
    Coordinates autonomous missions for package delivery between buildings.
    Patrol route: Building A (Engineering) → Building B (Library) → Building C (Cafeteria)
    """

    def __init__(self) -> None:
        super().__init__("campus_delivery_navigation")
        if BasicNavigator is None:
            raise RuntimeError(
                "nav2_simple_commander is not installed; install ros-foxy-nav2-simple-commander."
            )

        self.declare_parameter("mode", "slam")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("initial_pose", {})
        self.declare_parameter("waypoints", [])
        self.declare_parameter("waypoint_follow_delay", 2.0)

        self._navigator = BasicNavigator()
        self._status_pub = self.create_publisher(String, "campus_delivery/navigation/status", 10)
        self._building_pub = self.create_publisher(String, "campus_delivery/current_building", 10)
        self._planned_path_pub = self.create_publisher(Path, "campus_delivery/navigation/planned_path", 10)
        self._logger = configure_logging(self.get_name())

        self._current_goal: Optional[PoseStamped] = None
        self._waypoints: Deque[PoseStamped] = deque(
            self._load_waypoints(self.get_parameter("waypoints").value)
        )
        self._goal_frame = self.get_parameter("global_frame").value

        initial_pose_param = self.get_parameter("initial_pose").value
        if isinstance(initial_pose_param, dict) and initial_pose_param:
            pose = self._dict_to_pose(initial_pose_param)
            self._navigator.setInitialPose(pose)
            self._logger.info(
                "Initial pose set to x=%.2f y=%.2f yaw=%.2f", pose.pose.position.x, pose.pose.position.y, 0.0
            )

        mode = self.get_parameter("mode").value
        self._logger.info("=== Campus Delivery Navigation System ===")
        self._logger.info(f"Waiting for Nav2 to become active ({mode} mode)...")
        self._navigator.waitUntilNav2Active()
        self._logger.info("Nav2 active! Ready for campus delivery missions.")
        self._logger.info("Patrol Route: Building A → Building B → Building C")

        self._goal_sub = self.create_subscription(
            PoseStamped,
            "campus_delivery/navigation/goal",
            self._on_dynamic_goal,
            10,
        )
        self._feedback_timer = self.create_timer(1.0, self._tick_feedback)

        if self.get_parameter("auto_start").value and self._waypoints:
            self._dispatch_next_waypoint()

    def _load_waypoints(self, raw: Iterable[Dict]) -> Iterable[PoseStamped]:
        for entry in raw or []:
            try:
                yield self._dict_to_pose(entry)
            except (KeyError, TypeError, ValueError) as exc:
                self._logger.error("Invalid waypoint entry %s: %s", entry, exc)

    def _dict_to_pose(self, spec: Dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = spec.get("frame_id", self.get_parameter("global_frame").value)
        pose.pose.position.x = float(spec["position"]["x"])
        pose.pose.position.y = float(spec["position"]["y"])
        pose.pose.position.z = float(spec["position"].get("z", 0.0))
        yaw = float(spec.get("yaw", 0.0))
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def _dispatch_next_waypoint(self) -> None:
        if not self._waypoints:
            self._logger.info("Campus delivery patrol complete. All waypoints visited.")
            return

        waypoint = self._waypoints.popleft()
        self._navigator.goToPose(waypoint)
        self._current_goal = waypoint
        building_name = self._get_building_name(waypoint)
        self._logger.info(
            f"Navigating to {building_name} (x={waypoint.pose.position.x:.2f}, y={waypoint.pose.position.y:.2f})..."
        )
        self._publish_status(f"En route to {building_name}")
        self._publish_building(building_name)
    
    def _get_building_name(self, waypoint: PoseStamped) -> str:
        """Determine building name from waypoint coordinates"""
        x, y = waypoint.pose.position.x, waypoint.pose.position.y
        # Building identification based on coordinates
        if abs(x) < 0.5 and abs(y) < 0.5:
            return "Building A (Engineering)"
        elif abs(x - 3.0) < 0.5 and abs(y - 2.0) < 0.5:
            return "Building B (Library)"
        elif abs(x - 6.0) < 0.5 and abs(y - 0.5) < 0.5:
            return "Building C (Cafeteria)"
        else:
            return f"Waypoint (x={x:.1f}, y={y:.1f})"

    def _on_dynamic_goal(self, goal: PoseStamped) -> None:
        building = self._get_building_name(goal)
        self._logger.info(
            f"Received dynamic delivery goal: {building} (x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f})"
        )
        self._navigator.goToPose(goal)
        self._current_goal = goal
        self._publish_building(building)

    def _tick_feedback(self) -> None:
        if self._current_goal is None:
            return
        feedback = self._navigator.getFeedback()
        if feedback:
            distance = getattr(feedback, "distance_remaining", float("nan"))
            self._publish_status(f"Distance remaining: {distance:.2f} m")
            path = Path()
            path.header.frame_id = self._goal_frame
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = list(feedback.path)
            self._planned_path_pub.publish(path)

        if self._navigator.isTaskComplete():
            result = self._navigator.getResult()
            self._handle_result(result)

    def _handle_result(self, result: TaskResult) -> None:
        if result == TaskResult.SUCCEEDED:
            self._publish_status("Delivery point reached successfully!")
            self._logger.info("Delivery destination reached.")
        elif result == TaskResult.CANCELED:
            self._publish_status("Delivery canceled.")
            self._logger.warning("Delivery goal canceled.")
        else:
            self._publish_status("Failed to reach delivery point.")
            self._logger.error("Delivery navigation failed with result %s", result)

        self._current_goal = None
        delay = float(self.get_parameter("waypoint_follow_delay").value)
        if self._waypoints:
            self._logger.info("Next building in %.1f seconds", delay)
            self.create_timer(delay, self._dispatch_next_waypoint, oneshot=True)

    def _publish_status(self, message: str) -> None:
        msg = String()
        msg.data = message
        self._status_pub.publish(msg)
        self.get_logger().info(message)
    
    def _publish_building(self, building: str) -> None:
        """Publish current building location"""
        msg = String()
        msg.data = building
        self._building_pub.publish(msg)


def main() -> None:
    rclpy.init()
    try:
        node = CampusDeliveryNavigationManager()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)  # pragma: no cover - CLI surface
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        node.get_logger().info("Campus Delivery Navigation stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
