#!/usr/bin/env python3
"""Speech-to-text bridge for Gizmo (Iteration 6).

Publishes to:
    /gizmo/user_text   std_msgs/String   - recognised utterance for the brain

The STT layer feeds the brain. It deliberately does **not** know about
movement, face or TTS; the brain handles routing. Engines:

    dummy   - default; no audio I/O. The node idles so the launch file
              works on machines without a microphone (Gazebo + WSL2 by
              default has none). Test with:
                  ros2 topic pub --once /gizmo/user_text \\
                      std_msgs/String "data: 'lauf nach vorne'"
    mic     - uses the SpeechRecognition library. The recogniser runs in
              a background thread (listen_in_background) so the ROS spin
              loop stays responsive. If the library or the microphone is
              missing the node falls back to dummy mode and logs why,
              instead of crashing the whole launch.

ROS parameters:
    engine             "dummy" | "mic"  (default "dummy")
    output_topic       publish target (default "/gizmo/user_text")
    language           BCP-47 tag for the recognizer (default "de-DE")
    mic_index          PortAudio device index, -1 = default mic
    energy_threshold   silence cutoff for the recogniser
    pause_threshold    seconds of silence that end an utterance
    phrase_time_limit  hard cap on a single phrase (s)
    recognizer         "google" | "sphinx" - which SR backend to call
                       (only relevant when engine == "mic")

End-to-end latency budget (mic engine, target < 3 s per ROADMAP iter 6):
    mic capture + VAD     ~  pause_threshold (0.8 s default)
    cloud recogniser      ~  0.4-1.0 s
    brain routing         ~  0.01 s
    TTS playback start    ~  0.05 s (espeak-ng)
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GizmoSttNode(Node):
    def __init__(self) -> None:
        super().__init__("gizmo_stt_node")

        self.declare_parameter("engine", "dummy")
        self.declare_parameter("output_topic", "/gizmo/user_text")
        self.declare_parameter("language", "de-DE")
        self.declare_parameter("mic_index", -1)
        self.declare_parameter("energy_threshold", 300.0)
        self.declare_parameter("pause_threshold", 0.8)
        self.declare_parameter("phrase_time_limit", 6.0)
        self.declare_parameter("recognizer", "google")

        engine = self._s("engine").strip().lower()
        out_topic = self._s("output_topic")
        self.pub = self.create_publisher(String, out_topic, 10)

        self._stop_listening = None
        if engine == "dummy":
            self.get_logger().info(
                f"stt engine='dummy' -> '{out_topic}'. "
                f"Publish text manually via `ros2 topic pub`."
            )
        elif engine == "mic":
            ok = self._start_mic_engine()
            if not ok:
                self.get_logger().warn(
                    "stt engine='mic' unavailable, falling back to dummy"
                )
        else:
            self.get_logger().warn(
                f"unknown stt engine '{engine}', falling back to dummy"
            )

    def _s(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _f(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _i(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _start_mic_engine(self) -> bool:
        """Start a background thread that publishes recognised speech.

        Returns True if the recogniser is up and running. On any setup
        failure (missing library, no audio device, no API key for the
        chosen backend) the method logs the cause and returns False so
        the caller can fall back to dummy mode without crashing.
        """
        try:
            import speech_recognition as sr  # type: ignore
        except ImportError:
            self.get_logger().error(
                "speech_recognition not installed; "
                "`pip install SpeechRecognition pyaudio` to enable mic mode"
            )
            return False

        try:
            recognizer = sr.Recognizer()
            recognizer.energy_threshold = self._f("energy_threshold")
            recognizer.pause_threshold = self._f("pause_threshold")
            recognizer.dynamic_energy_threshold = True

            mic_index = self._i("mic_index")
            mic_kwargs = {} if mic_index < 0 else {"device_index": mic_index}
            mic = sr.Microphone(**mic_kwargs)
            with mic as source:
                recognizer.adjust_for_ambient_noise(source, duration=0.5)
        except Exception as exc:
            self.get_logger().error(f"microphone init failed: {exc}")
            return False

        language = self._s("language")
        backend = self._s("recognizer").strip().lower()
        phrase_time_limit = self._f("phrase_time_limit")

        def callback(_recognizer, audio):
            try:
                if backend == "sphinx":
                    text = _recognizer.recognize_sphinx(
                        audio, language=language
                    )
                else:
                    text = _recognizer.recognize_google(
                        audio, language=language
                    )
            except sr.UnknownValueError:
                self.get_logger().debug("stt: speech unintelligible")
                return
            except Exception as exc:
                self.get_logger().warn(f"stt recognition failed: {exc}")
                return

            text = (text or "").strip()
            if not text:
                return
            self.get_logger().info(f"stt heard: '{text}'")
            self.pub.publish(String(data=text))

        # listen_in_background spawns its own thread. Storing the stop
        # handle lets shutdown stop the recogniser cleanly.
        self._stop_listening = recognizer.listen_in_background(
            mic, callback, phrase_time_limit=phrase_time_limit
        )
        # Keep a reference to the thread name in the log so users can
        # tell the engine actually started up.
        self._mic_thread_name = threading.current_thread().name
        self.get_logger().info(
            f"stt engine='mic' backend='{backend}' lang='{language}' "
            f"-> '{self._s('output_topic')}'"
        )
        return True

    def destroy_node(self) -> bool:
        if self._stop_listening is not None:
            try:
                self._stop_listening(wait_for_stop=False)
            except Exception:
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GizmoSttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
