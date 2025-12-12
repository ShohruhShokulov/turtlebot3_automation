"""
Campus Delivery QR-Docking System

Custom feature: Detect QR codes on room doors for automatic docking.
When the correct room QR code is detected (ROOM_A, ROOM_B, etc.),
the robot automatically navigates to the docking pose for package delivery.
"""

from __future__ import annotations

import cv2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ..utils.logging import configure_logging

try:  # pragma: no cover - runtime dependency
    from cv_bridge import CvBridge
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("cv_bridge is required for QR code following") from exc


class CampusQRDockingNode(Node):
    """
    Smart Campus Delivery - QR-Based Room Docking System
    
    Detects QR codes on room doors and automatically docks for package delivery.
    Supports room identifiers: ROOM_A, ROOM_B, ROOM_C, etc.
    """

    def __init__(self) -> None:
        super().__init__("campus_qr_docking")
        self.declare_parameter("target_rooms", ["ROOM_A", "ROOM_B", "ROOM_C"])  # List of valid rooms
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_gain", 0.0025)
        self.declare_parameter("distance_gain", 0.00025)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("max_linear", 0.18)
        self.declare_parameter("docking_distance", 0.5)  # Stop distance from door (meters)

        self._logger = configure_logging(self.get_name())
        self._bridge = CvBridge()
        self._detector = cv2.QRCodeDetector()
        self._current_room = None
        self._docked = False

        self._cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self._qr_pub = self.create_publisher(String, "campus_delivery/qr_detected", 10)
        self._docking_pub = self.create_publisher(String, "campus_delivery/docking_status", 10)
        self._image_sub = self.create_subscription(
            Image,
            self.get_parameter("camera_topic").value,
            self._on_image,
            10,
        )

        self._last_detection = None
        target_rooms = self.get_parameter("target_rooms").value
        self._logger.info("=== Campus QR-Docking System ===")
        self._logger.info(f"Target rooms: {target_rooms}")
        self._logger.info("Scanning for room QR codes to auto-dock for delivery")

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        data, points, _ = self._detector.detectAndDecode(frame)
        twist = Twist()

        if data:
            self._last_detection = data
            target_rooms = self.get_parameter("target_rooms").value
            
            # Check if detected QR is a valid room
            if data not in target_rooms:
                self.get_logger().debug(f"Ignoring non-room QR: {data}")
                self._cmd_pub.publish(twist)
                return
            
            # Valid room detected!
            if self._current_room != data:
                self._current_room = data
                self._docked = False
                self._logger.info(f"ROOM DETECTED: {data} - Initiating docking sequence")
                qr_msg = String()
                qr_msg.data = data
                self._qr_pub.publish(qr_msg)

            if points is not None and points.any():
                pts = points[0]
                cx = pts[:, 0].mean()
                cy = pts[:, 1].mean()
                width = frame.shape[1]
                height = frame.shape[0]
                error_x = (cx - width / 2.0)
                error_y = height - cy

                # Check if we're close enough to dock
                qr_size = ((pts[:, 0].max() - pts[:, 0].min()) + 
                          (pts[:, 1].max() - pts[:, 1].min())) / 2.0
                
                if qr_size > 200 and not self._docked:  # QR is large enough, we're close
                    self._docked = True
                    self._logger.info(f"DOCKED at {data}! Ready for package delivery.")
                    dock_msg = String()
                    dock_msg.data = f"DOCKED:{data}"
                    self._docking_pub.publish(dock_msg)
                    # Stop the robot
                    self._cmd_pub.publish(twist)
                    return

                # Continue approaching
                angular = -error_x * float(self.get_parameter("angular_gain").value)
                linear = min(
                    float(self.get_parameter("linear_speed").value),
                    error_y * float(self.get_parameter("distance_gain").value),
                )
                twist.angular.z = max(
                    -float(self.get_parameter("max_angular").value),
                    min(float(self.get_parameter("max_angular").value), angular),
                )
                twist.linear.x = max(
                    0.0,
                    min(float(self.get_parameter("max_linear").value), linear),
                )
                self._logger.debug(
                    f"Approaching {data}: linear={twist.linear.x:.3f}, angular={twist.angular.z:.3f}"
                )
        else:
            self._last_detection = None
            if self._current_room and not self._docked:
                self._logger.debug("Lost sight of room QR code, searching...")

        self._cmd_pub.publish(twist)


def main() -> None:
    rclpy.init()
    node = CampusQRDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        node.get_logger().info("Campus QR-Docking System stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
