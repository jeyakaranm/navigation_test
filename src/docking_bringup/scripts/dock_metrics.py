#!/usr/bin/env python3
"""Docking metrics node.

Subscribes to Gazebo ground-truth model states and the docking controller
status, and reports the final pose error (range / lateral / yaw) against the
desired dock pose once the controller reports DOCKED. Optionally appends a
CSV row for offline statistics.
"""

import csv
import math
import os

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DockMetrics(Node):
    def __init__(self):
        super().__init__("dock_metrics")
        self.declare_parameter("robot_model_name", "burger")
        self.declare_parameter("dock_face_x", 3.0)
        self.declare_parameter("standoff_distance", 0.45)
        self.declare_parameter("lateral_offset", 0.50)
        self.declare_parameter("csv_path", "")

        self.robot_name = self.get_parameter("robot_model_name").value
        self.x_des = (
            self.get_parameter("dock_face_x").value
            - self.get_parameter("standoff_distance").value
        )
        self.y_des = self.get_parameter("lateral_offset").value
        self.yaw_des = 0.0
        self.csv_path = self.get_parameter("csv_path").value

        self.last_pose = None
        self.reported = False
        self.last_state = ""

        self.create_subscription(
            ModelStates, "/dock/model_states", self.model_cb, 10
        )
        self.create_subscription(String, "/docking/status", self.status_cb, 10)
        self.get_logger().info(
            f"Metrics ready. Desired pose: x={self.x_des:.3f} "
            f"y={self.y_des:.3f} yaw={self.yaw_des:.3f}"
        )

    def model_cb(self, msg):
        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            self.last_pose = msg.pose[idx]

    def status_cb(self, msg):
        state = ""
        for tok in msg.data.split():
            if tok.startswith("state="):
                state = tok.split("=", 1)[1]
        if state and state != self.last_state:
            self.get_logger().info(f"Controller state -> {state}")
            self.last_state = state
        if state == "DOCKED" and not self.reported and self.last_pose is not None:
            self.report()
            self.reported = True
        if state == "ABORT" and not self.reported:
            self.get_logger().warn("Docking ABORTED by controller.")
            self.reported = True

    def report(self):
        p = self.last_pose
        yaw = yaw_from_quat(p.orientation)
        ex = p.position.x - self.x_des
        ey = p.position.y - self.y_des
        eyaw = wrap(yaw - self.yaw_des)
        dist = math.hypot(ex, ey)
        self.get_logger().info("==================== DOCKING RESULT ====================")
        self.get_logger().info(
            f"  ground-truth pose : x={p.position.x:.3f} y={p.position.y:.3f} "
            f"yaw={math.degrees(yaw):.2f} deg"
        )
        self.get_logger().info(
            f"  pose error        : ex={ex*1000:.1f} mm  ey={ey*1000:.1f} mm  "
            f"|d|={dist*1000:.1f} mm  eyaw={math.degrees(eyaw):.2f} deg"
        )
        self.get_logger().info("========================================================")
        if self.csv_path:
            new = not os.path.exists(self.csv_path)
            with open(self.csv_path, "a", newline="") as f:
                w = csv.writer(f)
                if new:
                    w.writerow(["ex_m", "ey_m", "dist_m", "eyaw_rad"])
                w.writerow([f"{ex:.4f}", f"{ey:.4f}", f"{dist:.4f}", f"{eyaw:.4f}"])


def main():
    rclpy.init()
    node = DockMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
