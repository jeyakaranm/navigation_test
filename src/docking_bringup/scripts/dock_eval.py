#!/usr/bin/env python3
"""Multi-trial docking evaluation.

Runs N docking trials. Before each trial the robot is teleported back to the
start pose via Gazebo set_entity_state and docking is (re)started via the
controller service. The final ground-truth pose error and success flag are
recorded, then aggregate statistics (mean / std / max, success rate) are
printed and optionally written to CSV.

Usage:
  ros2 run docking_bringup dock_eval.py --ros-args -p trials:=10
"""

import math
import random
import statistics
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DockEval(Node):
    def __init__(self):
        super().__init__("dock_eval")
        self.declare_parameter("trials", 5)
        self.declare_parameter("robot_model_name", "burger")
        self.declare_parameter("start_x", 1.0)
        self.declare_parameter("start_y", 0.5)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("start_y_jitter", 0.10)
        self.declare_parameter("start_yaw_jitter", 0.15)
        self.declare_parameter("dock_face_x", 3.0)
        self.declare_parameter("standoff_distance", 0.45)
        self.declare_parameter("lateral_offset", 0.50)
        self.declare_parameter("trial_timeout", 60.0)
        self.declare_parameter("success_pos_tol", 0.05)
        self.declare_parameter("success_yaw_tol", 0.05)
        self.declare_parameter("csv_path", "")

        g = self.get_parameter
        self.trials = int(g("trials").value)
        self.robot_name = g("robot_model_name").value
        self.start = (g("start_x").value, g("start_y").value, g("start_yaw").value)
        self.y_jitter = g("start_y_jitter").value
        self.yaw_jitter = g("start_yaw_jitter").value
        self.x_des = g("dock_face_x").value - g("standoff_distance").value
        self.y_des = g("lateral_offset").value
        self.timeout = g("trial_timeout").value
        self.pos_tol = g("success_pos_tol").value
        self.yaw_tol = g("success_yaw_tol").value
        self.csv_path = g("csv_path").value

        self.last_pose = None
        self.state = ""
        self.create_subscription(ModelStates, "/dock/model_states", self._model_cb, 10)
        self.create_subscription(String, "/docking/status", self._status_cb, 10)
        self.set_state = self.create_client(SetEntityState, "/dock/set_entity_state")
        self.start_cli = self.create_client(Trigger, "/docking/start")
        self.cancel_cli = self.create_client(Trigger, "/docking/cancel")

    def _model_cb(self, msg):
        if self.robot_name in msg.name:
            self.last_pose = msg.pose[msg.name.index(self.robot_name)]

    def _status_cb(self, msg):
        for tok in msg.data.split():
            if tok.startswith("state="):
                self.state = tok.split("=", 1)[1]

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _teleport(self):
        # Each trial starts from a slightly different pose: a small random
        # lateral (y) and heading (yaw) offset around the nominal start, so the
        # docking is exercised from a range of off-axis approaches.
        y = self.start[1] + random.uniform(-self.y_jitter, self.y_jitter)
        yaw = self.start[2] + random.uniform(-self.yaw_jitter, self.yaw_jitter)
        self.trial_start = (self.start[0], y, yaw)
        req = SetEntityState.Request()
        req.state.name = self.robot_name
        req.state.pose.position.x = float(self.start[0])
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.01
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = "world"
        fut = self.set_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def _start_docking(self):
        if self.start_cli.service_is_ready():
            fut = self.start_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def _cancel_docking(self):
        if self.cancel_cli.service_is_ready():
            fut = self.cancel_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def _wait_for_engage(self, timeout_sec):
        """Block until the controller reports a fresh, active (non-terminal)
        state, so the trial never latches onto a stale DOCKED/ABORT left over
        from the previous trial. Returns True once engaged."""
        active = ("SEARCH", "APPROACH_PREDOCK", "BACKUP_RETRY", "PREDOCK", "FINAL_APPROACH", "HOLD")
        end = time.time() + timeout_sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state in active:
                return True
        return False

    def run(self):
        for cli in (self.set_state, self.start_cli, self.cancel_cli):
            cli.wait_for_service(timeout_sec=10.0)
        results = []
        for k in range(self.trials):
            self.get_logger().info(f"--- trial {k + 1}/{self.trials} ---")
            # Force the FSM back to IDLE so a DOCKED/ABORT from the previous
            # trial cannot leak into this one, then teleport and settle.
            self._cancel_docking()
            self._spin(0.5)
            self.state = ""
            self._teleport()
            self._spin(2.0)  # let the robot settle
            self.get_logger().info(
                f"  start pose: y={self.trial_start[1]:.3f} m "
                f"yaw={math.degrees(self.trial_start[2]):.1f} deg"
            )
            self.state = ""  # discard any status buffered during teleport
            self._start_docking()
            # Only begin watching for the terminal state once the controller
            # has actually (re)engaged this trial.
            self._wait_for_engage(5.0)

            t_end = time.time() + self.timeout
            while time.time() < t_end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.state in ("DOCKED", "ABORT"):
                    break
            self._spin(0.5)

            p = self.last_pose
            if p is None:
                continue
            yaw = yaw_from_quat(p.orientation)
            ex = p.position.x - self.x_des
            ey = p.position.y - self.y_des
            eyaw = wrap(yaw)
            dist = math.hypot(ex, ey)
            success = (
                self.state == "DOCKED"
                and dist < self.pos_tol
                and abs(eyaw) < self.yaw_tol
            )
            results.append((ex, ey, dist, eyaw, success))
            self.get_logger().info(
                f"  result: dist={dist*1000:.1f} mm eyaw={math.degrees(eyaw):.2f} deg "
                f"state={self.state} success={success}"
            )

        self._summary(results)

    def _summary(self, results):
        if not results:
            self.get_logger().error("No trials completed.")
            return
        dists = [r[2] for r in results]
        yaws = [abs(r[3]) for r in results]
        succ = [r[4] for r in results]
        n = len(results)
        self.get_logger().info("==================== EVAL SUMMARY ====================")
        self.get_logger().info(f"  trials        : {n}")
        self.get_logger().info(f"  success rate  : {sum(succ)}/{n} = {100.0*sum(succ)/n:.1f}%")
        self.get_logger().info(
            f"  pos error  mm : mean={1000*statistics.mean(dists):.1f} "
            f"max={1000*max(dists):.1f} "
            f"std={1000*(statistics.pstdev(dists)):.1f}"
        )
        self.get_logger().info(
            f"  yaw error deg : mean={math.degrees(statistics.mean(yaws)):.2f} "
            f"max={math.degrees(max(yaws)):.2f} "
            f"std={math.degrees(statistics.pstdev(yaws)):.2f}"
        )
        self.get_logger().info("======================================================")
        if self.csv_path:
            import csv

            with open(self.csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["ex_m", "ey_m", "dist_m", "eyaw_rad", "success"])
                for r in results:
                    w.writerow([f"{r[0]:.4f}", f"{r[1]:.4f}", f"{r[2]:.4f}",
                                f"{r[3]:.4f}", int(r[4])])
            self.get_logger().info(f"  wrote CSV -> {self.csv_path}")


def main():
    rclpy.init()
    node = DockEval()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
