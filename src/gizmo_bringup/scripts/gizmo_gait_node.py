#!/usr/bin/env python3
"""Pose / gait driver for Gizmo.

Publishes JointTrajectory goals to the gizmo_joint_trajectory_controller.
Pick the action with the `action` ROS parameter:

    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=stand_pose
    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=wave
    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=crawl_forward

The crawl is a static walk - one leg lifts at a time while the other
three keep the body supported. Step length, step frequency, swing height,
duty factor and total duration are exposed as ROS parameters so the gait
can be tuned without code changes.
"""
from __future__ import annotations

import math

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

LEG_PREFIXES = ("front_left", "front_right", "rear_left", "rear_right")

# Symmetric stand: hips straight down, knees folded back so each lower
# leg points slightly behind vertical. All four feet sit at the same
# height, which lets the spawn height (see launch file) be picked so
# the feet just touch the ground at rest.
STAND_HIP = 0.0
STAND_KNEE = -0.6

STAND = {
    "front_left_hip_joint":   STAND_HIP,
    "front_left_knee_joint":  STAND_KNEE,
    "front_right_hip_joint":  STAND_HIP,
    "front_right_knee_joint": STAND_KNEE,
    "rear_left_hip_joint":    STAND_HIP,
    "rear_left_knee_joint":   STAND_KNEE,
    "rear_right_hip_joint":   STAND_HIP,
    "rear_right_knee_joint":  STAND_KNEE,
    "left_arm_joint":         0.0,
    "right_arm_joint":        0.0,
}

# Phase offsets for a static crawl. With duty = 0.75 each leg swings for
# a quarter of the cycle and the four offsets are 25 % apart, so exactly
# one leg is in swing at any time and three feet stay on the ground.
LEG_PHASE = {
    "front_left":  0.00,
    "rear_right":  0.25,
    "front_right": 0.50,
    "rear_left":   0.75,
}


def pose_to_positions(pose: dict[str, float]) -> list[float]:
    return [pose[j] for j in JOINT_ORDER]


def make_point(positions: list[float], t_sec: float) -> JointTrajectoryPoint:
    pt = JointTrajectoryPoint()
    pt.positions = positions
    secs = int(t_sec)
    nanos = int(round((t_sec - secs) * 1e9))
    pt.time_from_start = Duration(sec=secs, nanosec=nanos)
    return pt


def crawl_pose_at(
    t: float,
    *,
    frequency: float,
    step_length: float,
    swing_height: float,
    duty: float,
) -> dict[str, float]:
    """Joint positions for the crawl gait at simulation time `t` (s)."""
    pose = dict(STAND)
    for prefix, phi in LEG_PHASE.items():
        phase = (t * frequency + phi) % 1.0
        if phase < duty:
            # Stance: hip rotates from +step/2 (forward) to -step/2 (back),
            # pushing the body forward. Knee stays at the stand angle.
            s = phase / duty
            hip_offset = step_length / 2.0 - step_length * s
            knee_offset = 0.0
        else:
            # Swing: hip returns to the forward position while the knee
            # bends to lift the foot. sin-shape gives a smooth lift/drop.
            s = (phase - duty) / (1.0 - duty)
            hip_offset = -step_length / 2.0 + step_length * s
            knee_offset = -swing_height * math.sin(math.pi * s)
        pose[f"{prefix}_hip_joint"] = STAND_HIP + hip_offset
        pose[f"{prefix}_knee_joint"] = STAND_KNEE + knee_offset
    return pose


class GizmoGaitNode(Node):
    def __init__(self) -> None:
        super().__init__("gizmo_gait_node")
        self.declare_parameter("action", "stand_pose")
        self.declare_parameter(
            "topic",
            "/gizmo_joint_trajectory_controller/joint_trajectory",
        )

        # Crawl tuning knobs - kept conservative so the default call
        # passes the Iteration 2 done-criterion (>= 5 s rhythmic motion
        # without immediately tipping over).
        self.declare_parameter("crawl_frequency", 0.7)        # cycles / s
        self.declare_parameter("crawl_step_length", 0.35)     # rad (hip swing)
        self.declare_parameter("crawl_swing_height", 0.35)    # rad (knee lift)
        self.declare_parameter("crawl_duty_factor", 0.75)     # stance fraction
        self.declare_parameter("crawl_duration", 8.0)         # s of motion
        self.declare_parameter("crawl_dt", 0.05)              # s sample step
        self.declare_parameter("crawl_settle_time", 1.5)      # s into stand
        self.declare_parameter("crawl_recover_time", 1.5)     # s back to stand

        topic = self.get_parameter("topic").get_parameter_value().string_value
        self.pub = self.create_publisher(JointTrajectory, topic, 10)

        # Give the publisher a moment to discover the controller before
        # firing the first trajectory.
        self.timer = self.create_timer(1.5, self._kick_off)
        self._fired = False

    def _f(self, name: str) -> float:
        return float(self.get_parameter(name).value)

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
        elif action == "crawl_forward":
            self._publish_crawl()
        else:
            self.get_logger().warn(
                f"unknown action '{action}', defaulting to stand_pose"
            )
            self._publish_stand()

    def _publish_stand(self) -> None:
        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER
        msg.points = [make_point(pose_to_positions(STAND), 2.0)]
        self.pub.publish(msg)

    def _publish_wave(self) -> None:
        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER

        stand_pos = pose_to_positions(STAND)

        def with_right_arm(angle: float) -> list[float]:
            p = list(stand_pos)
            p[JOINT_ORDER.index("right_arm_joint")] = angle
            return p

        msg.points = [
            make_point(stand_pos,                          2.0),
            make_point(with_right_arm(-math.pi / 2),       3.5),
            make_point(with_right_arm(-math.pi / 2 + 0.5), 4.5),
            make_point(with_right_arm(-math.pi / 2 - 0.5), 5.5),
            make_point(with_right_arm(-math.pi / 2 + 0.5), 6.5),
            make_point(with_right_arm(-math.pi / 2 - 0.5), 7.5),
            make_point(stand_pos,                          9.0),
        ]
        self.pub.publish(msg)

    def _publish_crawl(self) -> None:
        freq = self._f("crawl_frequency")
        step_len = self._f("crawl_step_length")
        swing_h = self._f("crawl_swing_height")
        duty = self._f("crawl_duty_factor")
        duration = self._f("crawl_duration")
        dt = self._f("crawl_dt")
        settle = self._f("crawl_settle_time")
        recover = self._f("crawl_recover_time")

        if not 0.0 < duty < 1.0:
            self.get_logger().warn(
                f"crawl_duty_factor={duty} out of (0,1), clamping to 0.75"
            )
            duty = 0.75

        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER

        # 1. Settle into the stand pose.
        msg.points = [make_point(pose_to_positions(STAND), settle)]

        # 2. Bake the gait cycle as dense waypoints. The controller
        #    interpolates between them, giving a smooth motion.
        steps = max(1, int(round(duration / dt)))
        for i in range(1, steps + 1):
            t = i * dt
            pose = crawl_pose_at(
                t,
                frequency=freq,
                step_length=step_len,
                swing_height=swing_h,
                duty=duty,
            )
            msg.points.append(
                make_point(pose_to_positions(pose), settle + t)
            )

        # 3. End back in stand so the robot doesn't keep drifting from
        #    a half-finished cycle.
        msg.points.append(
            make_point(pose_to_positions(STAND), settle + duration + recover)
        )

        self.get_logger().info(
            "crawl: freq=%.2f Hz step=%.2f rad swing=%.2f rad "
            "duty=%.2f duration=%.1f s waypoints=%d"
            % (freq, step_len, swing_h, duty, duration, len(msg.points))
        )
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
