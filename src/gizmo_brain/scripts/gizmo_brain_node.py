#!/usr/bin/env python3
"""Brain / LLM Action Router for Gizmo.

Subscribes to:
    /gizmo/user_text   std_msgs/String   - natural-language user command

Publishes to:
    /gizmo/movement    std_msgs/String   - movement action name
    /gizmo/face        std_msgs/String   - face expression name
    /gizmo/say         std_msgs/String   - utterance for the (future) TTS layer

The Brain converts a free-form text command into a structured JSON
action and forwards it to the matching lower-layer router. Lower
layers do not know that an LLM exists - they just react to their own
topic. That keeps the architecture swappable: rule-based, local LLM
or cloud LLM all emit the same schema and route through the same
dispatch.

Action JSON schema (also documented in docs/PRD.md):

    {
        "action": "move" | "face" | "say" | "noop",
        "movement":   "<name>",  # required when action == "move"
        "expression": "<name>",  # required when action == "face"
        "text":       "<text>"   # required when action == "say"
    }

Backends (selectable via the `backend` parameter):
    rule_based  - deterministic keyword matcher, no external deps
    openai      - stub: would call OpenAI Chat Completions
    anthropic   - stub: would call Anthropic Messages

The cloud stubs fall back to the rule-based backend so a launch on a
machine without API keys still satisfies the Iteration 5 done-criteria.

Usage:
    ros2 topic pub --once /gizmo/user_text std_msgs/String "data: 'lauf nach vorne'"
    ros2 topic pub --once /gizmo/user_text std_msgs/String "data: 'freu dich'"
"""
from __future__ import annotations

import json
import os
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Recognised vocabularies. Kept here as the single source of truth so
# the rule-based backend and any future LLM prompt can share them.
KNOWN_MOVEMENTS = (
    "stand_pose",
    "wave",
    "wave_left",
    "wave_right",
    "arms_up",
    "arms_down",
    "crawl_forward",
)

KNOWN_EXPRESSIONS = (
    "neutral",
    "happy",
    "sad",
    "surprised",
    "angry",
)

# Order-sensitive: the matcher walks the list top-to-bottom and returns
# the first hit, so the more specific phrases must come first.
RULE_BASED_PATTERNS: list[tuple[tuple[str, ...], dict]] = [
    # Movement - crawl
    (("lauf nach vorne", "geh nach vorne", "krabbel nach vorne",
      "krabbel vorwärts", "krabbel", "vorwärts",
      "walk forward", "crawl forward", "move forward", "crawl"),
     {"action": "move", "movement": "crawl_forward"}),

    # Movement - arms
    (("winke links", "wink links", "wave left", "winke mit links"),
     {"action": "move", "movement": "wave_left"}),
    (("winke rechts", "wink rechts", "wave right", "winke mit rechts"),
     {"action": "move", "movement": "wave_right"}),
    (("winke", "wink", "wave", "hallo", "hi", "hello"),
     {"action": "move", "movement": "wave_right"}),
    (("arme hoch", "hände hoch", "hoch die hände",
      "arms up", "hands up"),
     {"action": "move", "movement": "arms_up"}),
    (("arme runter", "hände runter", "arme nach unten",
      "arms down", "hands down"),
     {"action": "move", "movement": "arms_down"}),
    (("steh still", "stehen bleiben", "still stehen",
      "stand pose", "stand still", "stand", "stehen"),
     {"action": "move", "movement": "stand_pose"}),

    # Face
    (("freu dich", "sei glücklich", "sei froh", "lach",
      "be happy", "happy", "smile"),
     {"action": "face", "expression": "happy"}),
    (("sei traurig", "be sad", "trauer", "sad"),
     {"action": "face", "expression": "sad"}),
    (("sei überrascht", "überrascht", "be surprised", "surprised", "wow"),
     {"action": "face", "expression": "surprised"}),
    (("sei wütend", "wütend", "sei sauer", "sauer",
      "be angry", "angry"),
     {"action": "face", "expression": "angry"}),
    (("neutral", "ruhig", "gesicht aus", "relax"),
     {"action": "face", "expression": "neutral"}),
]

# Prefixes that turn the rest of the message into a TTS utterance.
SAY_PREFIXES = ("sag mir ", "sage mir ", "sag ", "sage ", "sprich ",
                "say ", "tell me ", "tell ")


def _tokenize(text: str) -> list[str]:
    # \w handles ASCII + Unicode word chars (incl. ä/ö/ü/ß) on str
    # patterns in Python 3, which keeps the German keywords working.
    return re.findall(r"\w+", text.lower())


def _phrase_in_tokens(haystack: list[str], needle: list[str]) -> bool:
    n, m = len(haystack), len(needle)
    if m == 0 or m > n:
        return False
    for i in range(n - m + 1):
        if haystack[i:i + m] == needle:
            return True
    return False


def rule_based_route(text: str) -> dict:
    """Map a free-form German/English command to an action dict.

    Returns a dict matching the action JSON schema. If nothing matches,
    returns ``{"action": "noop"}`` so the caller can decide whether to
    log, ask the user to rephrase, or fall through to a cloud backend.
    Matching is whole-token: short keywords like "wink" never match
    inside an unrelated word.
    """
    norm = (text or "").strip().lower()
    if not norm:
        return {"action": "noop"}

    for prefix in SAY_PREFIXES:
        if norm.startswith(prefix):
            payload = text.strip()[len(prefix):].strip()
            if payload:
                return {"action": "say", "text": payload}

    tokens = _tokenize(norm)
    for keywords, action in RULE_BASED_PATTERNS:
        for kw in keywords:
            if _phrase_in_tokens(tokens, _tokenize(kw)):
                return dict(action)

    return {"action": "noop"}


def validate_action(action: dict) -> tuple[bool, str]:
    """Return (ok, reason) for the action JSON. Used after every backend
    call so the lower layers never see malformed input."""
    if not isinstance(action, dict):
        return False, "action must be an object"
    kind = action.get("action")
    if kind == "move":
        m = action.get("movement")
        if m not in KNOWN_MOVEMENTS:
            return False, f"unknown movement '{m}'"
        return True, ""
    if kind == "face":
        e = action.get("expression")
        if e not in KNOWN_EXPRESSIONS:
            return False, f"unknown expression '{e}'"
        return True, ""
    if kind == "say":
        t = action.get("text")
        if not isinstance(t, str) or not t.strip():
            return False, "say action needs non-empty 'text'"
        return True, ""
    if kind == "noop":
        return True, ""
    return False, f"unknown action kind '{kind}'"


class CloudBackend:
    """Stub for cloud LLM backends.

    The real implementation would build a prompt that pins the model to
    the JSON schema above (using the KNOWN_* tuples as the allowed
    vocabulary) and parse the structured reply. Here we keep the surface
    so the launch file can pass `backend:=openai` without crashing on
    machines that have no API keys configured: when the env var is
    missing we transparently fall back to the rule-based matcher.
    """

    def __init__(self, name: str, env_var: str) -> None:
        self.name = name
        self.env_var = env_var
        self.api_key = os.environ.get(env_var, "")

    def available(self) -> bool:
        return bool(self.api_key)

    def route(self, text: str) -> dict:
        # Real call would go here. We deliberately do not ship a network
        # dependency in the sim launch.
        return rule_based_route(text)


class GizmoBrainNode(Node):
    def __init__(self) -> None:
        super().__init__("gizmo_brain_node")

        self.declare_parameter("backend", "rule_based")
        self.declare_parameter("input_topic", "/gizmo/user_text")
        self.declare_parameter("movement_topic", "/gizmo/movement")
        self.declare_parameter("face_topic", "/gizmo/face")
        self.declare_parameter("say_topic", "/gizmo/say")

        self.backend_name = self._s("backend").strip().lower()
        in_topic = self._s("input_topic")
        move_topic = self._s("movement_topic")
        face_topic = self._s("face_topic")
        say_topic = self._s("say_topic")

        if self.backend_name == "openai":
            self._cloud = CloudBackend("openai", "OPENAI_API_KEY")
        elif self.backend_name == "anthropic":
            self._cloud = CloudBackend("anthropic", "ANTHROPIC_API_KEY")
        elif self.backend_name == "rule_based":
            self._cloud = None
        else:
            self.get_logger().warn(
                f"unknown backend '{self.backend_name}', "
                f"falling back to rule_based"
            )
            self.backend_name = "rule_based"
            self._cloud = None

        self.move_pub = self.create_publisher(String, move_topic, 10)
        self.face_pub = self.create_publisher(String, face_topic, 10)
        self.say_pub = self.create_publisher(String, say_topic, 10)
        self.sub = self.create_subscription(
            String, in_topic, self._on_text, 10
        )

        self.get_logger().info(
            f"brain ready: backend='{self.backend_name}' "
            f"in='{in_topic}' move='{move_topic}' "
            f"face='{face_topic}' say='{say_topic}'"
        )

    def _s(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _on_text(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            self.get_logger().warn("empty /gizmo/user_text, ignoring")
            return

        action = self._route(text)
        ok, reason = validate_action(action)
        if not ok:
            self.get_logger().warn(
                f"backend produced invalid action {action}: {reason}; "
                f"retrying with rule_based"
            )
            action = rule_based_route(text)
            ok, reason = validate_action(action)
            if not ok:
                self.get_logger().error(
                    f"rule-based fallback also invalid {action}: {reason}"
                )
                return

        self.get_logger().info(
            f"text='{text}' -> {json.dumps(action, ensure_ascii=False)}"
        )
        self._dispatch(action)

    def _route(self, text: str) -> dict:
        if self._cloud is not None and self._cloud.available():
            try:
                return self._cloud.route(text)
            except Exception as exc:
                self.get_logger().warn(
                    f"{self._cloud.name} backend failed ({exc}); "
                    f"falling back to rule_based"
                )
        return rule_based_route(text)

    def _dispatch(self, action: dict) -> None:
        kind = action.get("action")
        if kind == "move":
            self.move_pub.publish(String(data=action["movement"]))
        elif kind == "face":
            self.face_pub.publish(String(data=action["expression"]))
        elif kind == "say":
            self.say_pub.publish(String(data=action["text"]))
        elif kind == "noop":
            self.get_logger().info("noop: no matching action, doing nothing")


def main() -> None:
    rclpy.init()
    node = GizmoBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
