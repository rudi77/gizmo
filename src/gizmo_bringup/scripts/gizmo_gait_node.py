#!/usr/bin/env python3
"""Pose / gait driver for Gizmo.

Publishes JointTrajectory goals to the gizmo_joint_trajectory_controller.
Pick the action with the `action` ROS parameter:

    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=stand_pose
    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=wave
    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=wave_left
    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=wave_right
    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=arms_up
    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=arms_down
    ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=crawl_forward

To wave while crawling, combine actions:

    ros2 run gizmo_bringup gizmo_gait_node --ros-args \
        -p action:=crawl_forward -p crawl_arm_action:=wave_right

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
    "head_pan_joint",
]

LEG_PREFIXES = ("front_left", "front_right", "rear_left", "rear_right")

# Symmetric stand: hips straight down, knees folded back so each lower
# leg points slightly behind vertical. All four feet sit at the same
# height, which lets the spawn height (see launch file) be picked so
# the feet just touch the ground at rest.
STAND_HIP = 0.0
STAND_KNEE = -0.6

# Arms hang straight down at angle 0; -pi/2 lifts them forward/up so the
# robot can wave or signal.
ARM_DOWN = 0.0
ARM_UP = -math.pi / 2

# Wave centre and amplitude so wave actions stay clear of the joint limit
# (-2.5 .. 2.5 rad) and don't snap when transitioning between poses.
WAVE_CENTRE = ARM_UP
WAVE_AMPLITUDE = 0.5

STAND = {
    "front_left_hip_joint":   STAND_HIP,
    "front_left_knee_joint":  STAND_KNEE,
    "front_right_hip_joint":  STAND_HIP,
    "front_right_knee_joint": STAND_KNEE,
    "rear_left_hip_joint":    STAND_HIP,
    "rear_left_knee_joint":   STAND_KNEE,
    "rear_right_hip_joint":   STAND_HIP,
    "rear_right_knee_joint":  STAND_KNEE,
    "left_arm_joint":         ARM_DOWN,
    "right_arm_joint":        ARM_DOWN,
    "head_pan_joint":         0.0,
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

# Recognised values for `crawl_arm_action`. Anything else is treated as
# "no parallel arm motion" (arms stay in stand pose during crawl).
CRAWL_ARM_ACTIONS = ("none", "wave_left", "wave_right", "arms_up", "arms_down")


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


def apply_arm_motion(
    pose: dict[str, float],
    action: str,
    t: float,
    *,
    wave_frequency: float,
) -> None:
    """Overwrite the arm joints in `pose` for a parallel arm motion."""
    if action == "arms_up":
        pose["left_arm_joint"] = ARM_UP
        pose["right_arm_joint"] = ARM_UP
    elif action == "arms_down":
        pose["left_arm_joint"] = ARM_DOWN
        pose["right_arm_joint"] = ARM_DOWN
    elif action == "wave_left":
        pose["left_arm_joint"] = WAVE_CENTRE + WAVE_AMPLITUDE * math.sin(
            2.0 * math.pi * wave_frequency * t
        )
    elif action == "wave_right":
        pose["right_arm_joint"] = WAVE_CENTRE + WAVE_AMPLITUDE * math.sin(
            2.0 * math.pi * wave_frequency * t
        )
    # "none" or anything else: leave arms at stand pose.


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

        # Iteration 3: optional arm motion that runs in parallel to the
        # crawl. "none" leaves the arms in the stand pose.
        self.declare_parameter("crawl_arm_action", "none")
        self.declare_parameter("crawl_arm_wave_frequency", 1.5)  # Hz

        # Iteration 3: stand-alone arm-action timing.
        self.declare_parameter("arm_settle_time", 1.5)  # s into stand
        self.declare_parameter("arm_hold_time", 4.0)    # s for static poses
        self.declare_parameter("arm_wave_cycles", 4)    # full sin cycles
        self.declare_parameter("arm_wave_frequency", 1.5)  # Hz

        topic = self.get_parameter("topic").get_parameter_value().string_value
        self.pub = self.create_publisher(JointTrajectory, topic, 10)

        # Give the publisher a moment to discover the controller before
        # firing the first trajectory.
        self.timer = self.create_timer(1.5, self._kick_off)
        self._fired = False

    def _f(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _i(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _s(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _kick_off(self) -> None:
        if self._fired:
            return
        self._fired = True
        action = self._s("action")
        self.get_logger().info(f"running action: {action}")

        if action == "stand_pose":
            self._publish_stand()
        elif action == "wave" or action == "wave_right":
            self._publish_wave(side="right")
        elif action == "wave_left":
            self._publish_wave(side="left")
        elif action == "arms_up":
            self._publish_arm_pose(left=ARM_UP, right=ARM_UP)
        elif action == "arms_down":
            self._publish_arm_pose(left=ARM_DOWN, right=ARM_DOWN)
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

    def _publish_arm_pose(self, *, left: float, right: float) -> None:
        """Move both arms to fixed angles via the stand pose.

        Always going through the stand pose keeps the path smooth even if
        the previous action left the arms somewhere unusual, because the
        controller only has to interpolate two short arcs instead of one
        big jump straight to the target.
        """
        settle = self._f("arm_settle_time")
        hold = self._f("arm_hold_time")

        target = dict(STAND)
        target["left_arm_joint"] = left
        target["right_arm_joint"] = right

        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER
        msg.points = [
            make_point(pose_to_positions(STAND),  settle),
            make_point(pose_to_positions(target), settle + hold * 0.4),
            make_point(pose_to_positions(target), settle + hold),
        ]
        self.pub.publish(msg)

    def _publish_wave(self, *, side: str) -> None:
        """Side-aware wave that always returns through the stand pose.

        The arm first ramps to WAVE_CENTRE (-pi/2), oscillates for
        `arm_wave_cycles` periods at `arm_wave_frequency`, and then ramps
        back to ARM_DOWN. Sampling the sin wave at `crawl_dt` keeps the
        controller from seeing big jumps near the limits.
        """
        if side not in ("left", "right"):
            self.get_logger().warn(
                f"unknown wave side '{side}', defaulting to right"
            )
            side = "right"
        joint = "left_arm_joint" if side == "left" else "right_arm_joint"

        settle = self._f("arm_settle_time")
        cycles = max(1, self._i("arm_wave_cycles"))
        freq = self._f("arm_wave_frequency")
        if freq <= 0.0:
            self.get_logger().warn(
                f"arm_wave_frequency={freq} must be > 0, clamping to 1.5 Hz"
            )
            freq = 1.5
        wave_duration = cycles / freq
        ramp_in = 0.7  # s to lift the arm to WAVE_CENTRE
        ramp_out = 0.7  # s to drop the arm back to ARM_DOWN
        sample_dt = self._f("crawl_dt")

        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER

        stand_pos = pose_to_positions(STAND)
        msg.points = [make_point(stand_pos, settle)]

        # 1. Lift arm to WAVE_CENTRE without disturbing the rest.
        lifted = dict(STAND)
        lifted[joint] = WAVE_CENTRE
        t_lift_end = settle + ramp_in
        msg.points.append(make_point(pose_to_positions(lifted), t_lift_end))

        # 2. Sin oscillation around WAVE_CENTRE.
        steps = max(1, int(round(wave_duration / sample_dt)))
        for i in range(1, steps + 1):
            t = i * sample_dt
            angle = WAVE_CENTRE + WAVE_AMPLITUDE * math.sin(
                2.0 * math.pi * freq * t
            )
            pose = dict(STAND)
            pose[joint] = angle
            msg.points.append(
                make_point(pose_to_positions(pose), t_lift_end + t)
            )

        # 3. Ramp back to stand.
        msg.points.append(
            make_point(stand_pos, t_lift_end + wave_duration + ramp_out)
        )

        self.get_logger().info(
            "wave_%s: cycles=%d freq=%.2f Hz waypoints=%d"
            % (side, cycles, freq, len(msg.points))
        )
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
        arm_action = self._s("crawl_arm_action").strip().lower()
        arm_wave_freq = self._f("crawl_arm_wave_frequency")

        if not 0.0 < duty < 1.0:
            self.get_logger().warn(
                f"crawl_duty_factor={duty} out of (0,1), clamping to 0.75"
            )
            duty = 0.75

        if arm_action and arm_action not in CRAWL_ARM_ACTIONS:
            self.get_logger().warn(
                f"unknown crawl_arm_action '{arm_action}', "
                f"falling back to 'none'"
            )
            arm_action = "none"

        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER

        # 1. Settle into the stand pose.
        msg.points = [make_point(pose_to_positions(STAND), settle)]

        # 2. Bake the gait cycle as dense waypoints. The controller
        #    interpolates between them, giving a smooth motion. Arm
        #    motion (if requested) is layered on top of the same pose
        #    so it cannot fight the leg trajectory.
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
            apply_arm_motion(
                pose,
                arm_action,
                t,
                wave_frequency=arm_wave_freq,
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
            "duty=%.2f duration=%.1f s arm=%s waypoints=%d"
            % (freq, step_len, swing_h, duty, duration,
               arm_action or "none", len(msg.points))
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
