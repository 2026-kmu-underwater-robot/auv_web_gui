from copy import deepcopy
from html.parser import HTMLParser
from pathlib import Path

import rclpy

from kmu26_auv_web_gui.ros_interface import LocalizationRosNode
from kmu26_auv_web_gui.server import _pinger_live_preflight


ROOT = Path(__file__).resolve().parents[1]
VOID_ELEMENTS = {
    "area",
    "base",
    "br",
    "col",
    "embed",
    "hr",
    "img",
    "input",
    "link",
    "meta",
    "param",
    "source",
    "track",
    "wbr",
}


class _LayoutParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.stack: list[tuple[str, str]] = []
        self.section_parents: dict[str, str] = {}
        self.ids: list[str] = []
        self.app_scripts: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        values = dict(attrs)
        element_id = values.get("id") or ""
        if element_id:
            self.ids.append(element_id)
        if tag == "section" and element_id:
            parent_section = next(
                (item_id for item_tag, item_id in reversed(self.stack) if item_tag == "section"),
                "",
            )
            self.section_parents[element_id] = parent_section
        if tag == "script" and "app.js" in (values.get("src") or ""):
            self.app_scripts.append(values["src"] or "")
        if tag not in VOID_ELEMENTS:
            self.stack.append((tag, element_id))

    def handle_startendtag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        self.handle_starttag(tag, attrs)
        if tag not in VOID_ELEMENTS:
            self.handle_endtag(tag)

    def handle_endtag(self, tag: str) -> None:
        for index in range(len(self.stack) - 1, -1, -1):
            if self.stack[index][0] == tag:
                del self.stack[index:]
                return


def test_vision_and_pinger_tabs_are_siblings_and_script_is_loaded_once() -> None:
    parser = _LayoutParser()
    parser.feed((ROOT / "web" / "index.html").read_text(encoding="utf-8"))

    assert parser.section_parents["vision-tab"] == ""
    assert parser.section_parents["pinger-tab"] == ""
    assert len(parser.app_scripts) == 1
    assert len(parser.ids) == len(set(parser.ids))


def test_pinger_parameter_controls_and_css_contract_are_complete() -> None:
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
    javascript = (ROOT / "web" / "app.js").read_text(encoding="utf-8")

    assert html.count("data-pinger-param") == 13
    assert 'id="pinger-parameter-reset"' in html
    assert 'id="pinger-parameter-summary"' in html
    assert css.count("{") == css.count("}")
    assert ".vision-log-output" in css
    assert ".pinger-parameter-groups" in css
    assert "function validatePingerParameters" in javascript
    assert "bindPingerParameterControls();" in javascript


def test_ros_bridge_keeps_pinger_subscription_separate_and_owns_no_final_rc() -> None:
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init(args=[])
    node = LocalizationRosNode()
    try:
        subscription_topics = {item.topic_name for item in node.subscriptions}
        publisher_topics = {item.topic_name for item in node.publishers}
        assert "/mission/rc_command" in subscription_topics
        assert "/pinger_homing/status" in subscription_topics
        assert "/mavros/rc/override" not in publisher_topics
    finally:
        node.destroy_node()
        if owned_context and rclpy.ok():
            rclpy.shutdown()


def test_physical_contract_preflight_accepts_valid_data_and_rejects_positive_depth() -> None:
    status = {
        "topics": {
            "odom": {"alive": True},
            "depth": {"alive": True},
            "mavros_state": {"alive": True},
        },
        "mavros_state": {"connected": True, "armed": True, "mode": "ALT_HOLD"},
        "pose": {"z": -1.2},
        "depth": {"z": -1.1},
        "frames": {
            "odom": {"frame_id": "odom", "child_frame_id": "base_link"},
            "depth": {"frame_id": "odom"},
        },
        "graph": {
            "rc_output_publishers": 0,
            "rc_output_publisher_nodes": [],
            "audio_publishers": 1,
            "topic_types": {
                "odom": ["nav_msgs/msg/Odometry"],
                "depth": ["geometry_msgs/msg/PoseWithCovarianceStamped"],
                "mavros_state": ["mavros_msgs/msg/State"],
                "audio": ["audio_common_msgs/msg/AudioData"],
            },
            "services": {"arming": True, "set_mode": True},
        },
    }
    body = {
        "use_hydrophone_estimator": True,
        "use_audio_capture": False,
        "max_runtime_s": 180,
        "arrival_radius_m": 1.5,
    }

    valid = _pinger_live_preflight(status, body)
    assert valid["ok"]
    assert len(valid["checks"]) == 18

    positive_depth = deepcopy(status)
    positive_depth["pose"]["z"] = 1.2
    positive_depth["depth"]["z"] = 1.1
    invalid = _pinger_live_preflight(positive_depth, body)
    depth_check = next(item for item in invalid["checks"] if item["name"] == "depth_sign")
    assert not invalid["ok"]
    assert not depth_check["ok"]
