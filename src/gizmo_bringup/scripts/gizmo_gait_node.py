#!/usr/bin/env python3
"""Minimal pose/gait driver for the Iteration 1 Gizmo.

Publishes JointTrajectory goals to the gizmo_joint_trajectory_controller.
Pick the action with the `action` ROS parameter:

    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=stand_pose
    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=wave

This is a scaffold - real gait generation comes in a later iteration.
"""
from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_ORDER = [
    "front_left_hip_joint",
    "front_left_knee_joint",
    "front_right_hip_joint",
    "front_right_knee_joint",
    "rear_left_hip_joint",
    "rear_left_knee_joint",
    "rear_right_hip_joint",
    "rear_right_knee_joint",
    "left_arm_joint",
    "right_arm_joint",
]

# A relaxed standing pose: hips slightly forward, knees bent back so the
# foot sits roughly under the hip. Arms hang straight down.
STAND = {
    "front_left_hip_joint":   0.30,
    "front_left_knee_joint": -0.60,
    "front_right_hip_joint":  0.30,
    "front_right_knee_joint": -0.60,
    "rear_left_hip_joint":   -0.30,
    "rear_left_knee_joint":  -0.60,
    "rear_right_hip_joint":  -0.30,
    "rear_right_knee_joint": -0.60,
    "left_arm_joint":         0.0,
    "right_arm_joint":        0.0,
}


def pose_to_positions(pose: dict[str, float]) -> list[float]:
    return [pose[j] for j in JOINT_ORDER]


def make_point(positions: list[float], t_sec: float) -> JointTrajectoryPoint:
    pt = JointTrajectoryPoint()
    pt.positions = positions
    secs = int(t_sec)
    nanos = int((t_sec - secs) * 1e9)
    pt.time_from_start = Duration(sec=secs, nanosec=nanos)
    return pt


class GizmoGaitNode(Node):
    def __init__(self) -> None:
        super().__init__("gizmo_gait_node")
        self.declare_parameter("action", "stand_pose")
        self.declare_parameter(
            "topic",
            "/gizmo_joint_trajectory_controller/joint_trajectory",
        )

        topic = self.get_parameter("topic").get_parameter_value().string_value
        self.pub = self.create_publisher(JointTrajectory, topic, 10)

        # Give the publisher a moment to discover the controller before
        # firing the first trajectory.
        self.timer = self.create_timer(1.5, self._kick_off)
        self._fired = False

    def _kick_off(self) -> None:
        if self._fired:
            return
        self._fired = True
        action = self.get_parameter("action").get_parameter_value().string_value
        self.get_logger().info(f"running action: {action}")

        if action == "stand_pose":
            self._publish_stand()
        elif action == "wave":
            self._publish_wave()
        else:
            self.get_logger().warn(f"unknown action '{action}', defaulting to stand_pose")
            self._publish_stand()

    def _publish_stand(self) -> None:
        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER
        msg.points = [make_point(pose_to_positions(STAND), 2.0)]
        self.pub.publish(msg)

    def _publish_wave(self) -> None:
        # Stand first, then swing the right arm up, side to side, back down.
        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER

        stand_pos = pose_to_positions(STAND)

        def with_right_arm(angle: float) -> list[float]:
            p = list(stand_pos)
            p[JOINT_ORDER.index("right_arm_joint")] = angle
            return p

        msg.points = [
            make_point(stand_pos,                       2.0),
            make_point(with_right_arm(-math.pi / 2),    3.5),
            make_point(with_right_arm(-math.pi / 2 + 0.5), 4.5),
            make_point(with_right_arm(-math.pi / 2 - 0.5), 5.5),
            make_point(with_right_arm(-math.pi / 2 + 0.5), 6.5),
            make_point(with_right_arm(-math.pi / 2 - 0.5), 7.5),
            make_point(stand_pos,                       9.0),
        ]
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GizmoGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
