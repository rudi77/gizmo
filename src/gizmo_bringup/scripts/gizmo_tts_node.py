#!/usr/bin/env python3
"""Text-to-speech sink for Gizmo (Iteration 6).

Subscribes to:
    /gizmo/say   std_msgs/String   - utterance from the brain layer

The TTS layer is the third lower-half router (alongside movement and
face). It receives a plain text string and produces audio. As with the
other layers it does not know about the brain or the JSON action schema.

Engines:
    espeak  - default; calls `espeak-ng` (or `espeak`) via subprocess,
              non-blocking. Available on Ubuntu via
              `sudo apt install espeak-ng`. If neither binary is on PATH
              the node falls back to dummy mode rather than crashing.
    dummy   - logs the utterance only. Useful in headless CI / sim
              environments where there is no audio sink.

ROS parameters:
    engine        "espeak" | "dummy"  (default "espeak")
    input_topic   subscribe target (default "/gizmo/say")
    language      espeak voice tag (default "de")
    rate          words per minute (default 175)
    volume        0.0 - 2.0 (default 1.0; espeak amplitude is 0..200)

Latency note (Iteration 6 done-criterion):
    espeak-ng starts producing audio within ~50 ms of the subprocess
    spawning, so a single utterance adds well under 100 ms to the
    end-to-end voice loop. Total budget < 3 s in sim is dominated by
    the STT recogniser, not by this node.
"""
from __future__ import annotations

import shutil
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GizmoTtsNode(Node):
    def __init__(self) -> None:
        super().__init__("gizmo_tts_node")

        self.declare_parameter("engine", "espeak")
        self.declare_parameter("input_topic", "/gizmo/say")
        self.declare_parameter("language", "de")
        self.declare_parameter("rate", 175)
        self.declare_parameter("volume", 1.0)

        engine = self._s("engine").strip().lower()
        in_topic = self._s("input_topic")
        self.language = self._s("language")
        self.rate = self._i("rate")
        # espeak amplitude is 0..200; map a 0..2 volume float onto it
        # so callers can use the more familiar "1.0 = normal" scale.
        self.amplitude = max(0, min(200, int(round(self._f("volume") * 100))))

        self._binary = self._resolve_binary(engine)
        self.sub = self.create_subscription(
            String, in_topic, self._on_say, 10
        )

        if self._binary is None:
            self.get_logger().info(
                f"tts engine='dummy' on '{in_topic}': "
                f"utterances will be logged only"
            )
        else:
            self.get_logger().info(
                f"tts engine='{self._binary}' on '{in_topic}' "
                f"lang='{self.language}' rate={self.rate} "
                f"amplitude={self.amplitude}"
            )

    def _s(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _i(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _f(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _resolve_binary(self, engine: str) -> str | None:
        if engine == "dummy":
            return None
        if engine == "espeak":
            for candidate in ("espeak-ng", "espeak"):
                path = shutil.which(candidate)
                if path:
                    return path
            self.get_logger().warn(
                "espeak-ng/espeak not found on PATH, falling back to dummy "
                "(install with `sudo apt install espeak-ng`)"
            )
            return None
        self.get_logger().warn(
            f"unknown tts engine '{engine}', falling back to dummy"
        )
        return None

    def _on_say(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            self.get_logger().warn("received empty /gizmo/say, ignoring")
            return

        self.get_logger().info(f"say: '{text}'")
        if self._binary is None:
            return

        # Popen so the ROS spin loop is not blocked while espeak plays.
        # We deliberately drop the handle: the process is short-lived and
        # the OS reaps it; tracking it would force us to poll for exit.
        try:
            subprocess.Popen(
                [
                    self._binary,
                    "-v", self.language,
                    "-s", str(self.rate),
                    "-a", str(self.amplitude),
                    "--", text,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as exc:
            self.get_logger().warn(f"tts subprocess failed: {exc}")


def main() -> None:
    rclpy.init()
    node = GizmoTtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
