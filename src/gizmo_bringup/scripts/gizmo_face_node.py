#!/usr/bin/env python3
"""Map symbolic face expressions to a colour on Gizmo's display panel.

Subscribes to:
    /gizmo/face        std_msgs/String   - expression name (e.g. "happy")

Publishes to:
    /gizmo/face_color  std_msgs/ColorRGBA  - retint colour for the display

The launch file bridges /gizmo/face_color through ros_gz_bridge to a
gz.msgs.Color topic; the gz-sim-material-color-system plugin attached to
display_link in the URDF picks it up and changes the visual's colour in
Gazebo. On real hardware later the same /gizmo/face topic drives a real
display, so the Brain layer never has to know which is connected.

    ros2 topic pub /gizmo/face std_msgs/String "data: 'happy'"
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String


# Keep the palette small and high-contrast so the change is obvious in
# Gazebo. Add new expressions by extending this dict - the Brain node
# can then learn them from the same source of truth.
EXPRESSIONS: dict[str, tuple[float, float, float, float]] = {
    "neutral":   (0.70, 0.70, 0.75, 1.0),
    "happy":     (0.20, 0.90, 0.30, 1.0),
    "sad":       (0.20, 0.40, 0.90, 1.0),
    "surprised": (1.00, 0.90, 0.20, 1.0),
    "angry":     (0.95, 0.20, 0.15, 1.0),
}


class GizmoFaceNode(Node):
    def __init__(self) -> None:
        super().__init__("gizmo_face_node")

        self.declare_parameter("input_topic", "/gizmo/face")
        self.declare_parameter("output_topic", "/gizmo/face_color")
        self.declare_parameter("default_expression", "neutral")

        in_topic = self._s("input_topic")
        out_topic = self._s("output_topic")
        default = self._s("default_expression").strip().lower()

        self.pub = self.create_publisher(ColorRGBA, out_topic, 10)
        self.sub = self.create_subscription(
            String, in_topic, self._on_face, 10
        )

        self.get_logger().info(
            f"face node ready: '{in_topic}' -> '{out_topic}', "
            f"known expressions: {sorted(EXPRESSIONS)}"
        )

        # Latch the default expression so a fresh launch shows a face
        # even before anyone publishes on /gizmo/face.
        self._last = ""
        if default in EXPRESSIONS:
            self._publish(default)
        else:
            self.get_logger().warn(
                f"default_expression '{default}' unknown, using 'neutral'"
            )
            self._publish("neutral")

    def _s(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _on_face(self, msg: String) -> None:
        name = (msg.data or "").strip().lower()
        if not name:
            self.get_logger().warn("received empty /gizmo/face, ignoring")
            return
        if name not in EXPRESSIONS:
            self.get_logger().warn(
                f"unknown face '{name}', valid: {sorted(EXPRESSIONS)}"
            )
            return
        self._publish(name)

    def _publish(self, name: str) -> None:
        r, g, b, a = EXPRESSIONS[name]
        msg = ColorRGBA(r=r, g=g, b=b, a=a)
        self.pub.publish(msg)
        if name != self._last:
            self.get_logger().info(
                f"face='{name}' rgba=({r:.2f},{g:.2f},{b:.2f},{a:.2f})"
            )
            self._last = name


def main() -> None:
    rclpy.init()
    node = GizmoFaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
