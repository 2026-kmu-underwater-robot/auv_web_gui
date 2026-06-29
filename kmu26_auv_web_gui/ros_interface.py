import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import rclpy
from dvl_msgs.msg import CommandResponse
from dvl_msgs.msg import ConfigCommand
from dvl_msgs.msg import ConfigStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from robot_localization.srv import SetPose
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy


PATH_MIN_DISTANCE_M = 0.01
WEB_CONTROL_TIMEOUT_S = 0.35
WEB_CONTROL_PERIOD_S = 0.05


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> dict[str, float]:
    half_yaw = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


def _finite_or_none(value: float) -> float | None:
    return value if math.isfinite(value) else None


@dataclass
class TopicHealth:
    name: str
    stale_after: float = 1.0
    last_seen: float | None = None
    stamps: deque[float] = field(default_factory=lambda: deque(maxlen=120))

    def tick(self) -> None:
        now = time.monotonic()
        self.last_seen = now
        self.stamps.append(now)

    def snapshot(self) -> dict:
        now = time.monotonic()
        age = None if self.last_seen is None else now - self.last_seen
        return {
            "name": self.name,
            "alive": age is not None and age <= self.stale_after,
            "age": age,
            "hz": self._hz(now),
        }

    def _hz(self, now: float) -> float:
        recent = [stamp for stamp in self.stamps if now - stamp <= 2.0]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])


class LocalizationRosNode(Node):
    def __init__(self):
        super().__init__("kmu26_auv_web_gui_bridge")
        self._lock = threading.Lock()
        self._health = {
            "odom": TopicHealth("/odometry/filtered"),
            "dvl": TopicHealth("/dvl/twist"),
            "depth": TopicHealth("/depth/pose"),
            "imu": TopicHealth("/mavros/imu/data"),
            "mavros_state": TopicHealth("/mavros/state", stale_after=2.0),
            "joy": TopicHealth("/joy", stale_after=0.5),
            "battery": TopicHealth("/battery", stale_after=3.0),
        }
        self._pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self._velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._depth = {"z": 0.0}
        self._battery = {
            "voltage": None,
            "current": None,
            "temperature": None,
            "percentage": None,
            "present": False,
        }
        self._mavros_state = {
            "connected": False,
            "armed": False,
            "guided": False,
            "manual_input": False,
            "mode": "",
            "system_status": None,
            "updated_at": "",
        }
        self._joy = {"axes": [], "buttons": []}
        self._dvl_config: dict = {}
        self._dvl_events: deque[dict] = deque(maxlen=40)
        self._path: list[dict[str, float]] = []
        self._web_control = {
            "enabled": False,
            "active": False,
            "axes": {
                "forward": 0.0,
                "lateral": 0.0,
                "vertical": 0.0,
                "yaw": 0.0,
            },
            "last_command": 0.0,
            "last_publish": "",
            "neutral_burst": 0,
        }

        sensor_qos = qos_profile_sensor_data

        self._dvl_config_pub = self.create_publisher(
            ConfigCommand,
            "/dvl/config/command",
            sensor_qos,
        )
        self._web_joy_pub = self.create_publisher(Joy, "/joy", 10)
        self._arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self._set_pose_client = self.create_client(SetPose, "/set_pose")
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped, "/dvl/twist", self._on_dvl, sensor_qos)
        self.create_subscription(CommandResponse, "/dvl/command/response", self._on_dvl_response, sensor_qos)
        self.create_subscription(ConfigStatus, "/dvl/config/status", self._on_dvl_config, sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/depth/pose", self._on_depth, sensor_qos)
        self.create_subscription(BatteryState, "/battery", self._on_battery, sensor_qos)
        self.create_subscription(State, "/mavros/state", self._on_mavros_state, 20)
        self.create_subscription(Imu, "/mavros/imu/data", self._on_imu, sensor_qos)
        self.create_subscription(Joy, "/joy", self._on_joy, 20)
        self.create_timer(WEB_CONTROL_PERIOD_S, self._publish_web_control)

    def publish_dvl_command(
        self,
        command: str,
        parameter_name: str = "",
        parameter_value: str = "",
    ) -> None:
        msg = ConfigCommand()
        msg.command = command
        msg.parameter_name = parameter_name
        msg.parameter_value = parameter_value
        self._dvl_config_pub.publish(msg)
        self._append_dvl_event(
            "sent",
            command,
            parameter_name,
            parameter_value,
            True,
            "",
        )

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "topics": {name: item.snapshot() for name, item in self._health.items()},
                "pose": dict(self._pose),
                "velocity": dict(self._velocity),
                "depth": dict(self._depth),
                "battery": dict(self._battery),
                "joy": {
                    "axes": list(self._joy["axes"]),
                    "buttons": list(self._joy["buttons"]),
                },
                "mavros_state": dict(self._mavros_state),
                "dvl_config": dict(self._dvl_config),
                "dvl_events": list(self._dvl_events),
                "path": list(self._path),
                "path_count": len(self._path),
                "web_control": self._web_control_snapshot(),
            }

    def clear_path(self) -> None:
        with self._lock:
            self._path.clear()

    def wait_for_odom_after(self, timestamp: float, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                odom_seen = self._health["odom"].last_seen
            if odom_seen is not None and odom_seen >= timestamp:
                return True
            time.sleep(0.05)
        return False

    def set_localization_origin(self) -> dict:
        now = time.monotonic()
        with self._lock:
            odom_seen = self._health["odom"].last_seen
            if odom_seen is None or now - odom_seen > 2.0:
                raise RuntimeError("odometry is not alive; cannot set localization origin")
            pose = dict(self._pose)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = float(pose["z"])
        orientation = _quaternion_from_yaw(float(pose["yaw"]))
        msg.pose.pose.orientation.x = orientation["x"]
        msg.pose.pose.orientation.y = orientation["y"]
        msg.pose.pose.orientation.z = orientation["z"]
        msg.pose.pose.orientation.w = orientation["w"]
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = 0.01
        msg.pose.covariance[21] = 0.01
        msg.pose.covariance[28] = 0.01
        msg.pose.covariance[35] = 0.01
        if not self._set_pose_client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError("robot_localization set_pose service is not available")

        request = SetPose.Request()
        request.pose = msg
        future = self._set_pose_client.call_async(request)
        deadline = time.monotonic() + 1.0
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("robot_localization set_pose service timed out")
            time.sleep(0.02)
        if future.exception() is not None:
            raise RuntimeError(f"robot_localization set_pose failed: {future.exception()}")

        with self._lock:
            self._pose = {
                "x": 0.0,
                "y": 0.0,
                "z": pose["z"],
                "yaw": pose["yaw"],
            }
            self._path.clear()
        self.get_logger().info(
            "Set localization origin: "
            f"previous x={pose['x']:.3f} "
            f"y={pose['y']:.3f} "
            f"z={pose['z']:.3f} "
            f"yaw={pose['yaw']:.3f}"
        )
        return {
            "previous_pose": pose,
            "new_pose": {
                "x": 0.0,
                "y": 0.0,
                "z": pose["z"],
                "yaw": pose["yaw"],
            },
        }

    def set_web_control_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._web_control["enabled"] = enabled
            self._web_control["active"] = False
            self._web_control["last_command"] = time.monotonic()
            if not enabled:
                self._web_control["neutral_burst"] = 8

    def update_web_control_command(self, axes: dict, active: bool) -> None:
        clean_axes = {
            "forward": _clamp_axis(axes.get("forward", 0.0)),
            "lateral": _clamp_axis(axes.get("lateral", 0.0)),
            "vertical": _clamp_axis(axes.get("vertical", 0.0)),
            "yaw": _clamp_axis(axes.get("yaw", 0.0)),
        }
        with self._lock:
            self._web_control["active"] = bool(active)
            self._web_control["axes"] = clean_axes
            self._web_control["last_command"] = time.monotonic()

    def set_armed(self, armed: bool) -> bool:
        if not self._arm_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Arming service not available.")
            return False
        request = CommandBool.Request()
        request.value = bool(armed)
        self._arm_client.call_async(request)
        return True

    def set_mode(self, mode: str) -> bool:
        if not self._set_mode_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Set mode service not available.")
            return False
        request = SetMode.Request()
        request.custom_mode = mode
        self._set_mode_client.call_async(request)
        return True

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        with self._lock:
            self._health["odom"].tick()
            self._pose = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "yaw": yaw,
            }
            self._append_path_point(pose.position.x, pose.position.y)

    def _append_path_point(self, x: float, y: float) -> None:
        if not math.isfinite(x) or not math.isfinite(y):
            return
        if self._path:
            last = self._path[-1]
            if math.hypot(x - last["x"], y - last["y"]) < PATH_MIN_DISTANCE_M:
                return
        self._path.append({"x": x, "y": y})

    def _web_control_snapshot(self) -> dict:
        now = time.monotonic()
        last_command = self._web_control["last_command"]
        age = None if last_command == 0.0 else now - last_command
        return {
            "enabled": bool(self._web_control["enabled"]),
            "active": bool(self._web_control["active"]),
            "fresh": age is not None and age <= WEB_CONTROL_TIMEOUT_S,
            "age": age,
            "axes": dict(self._web_control["axes"]),
            "last_publish": self._web_control["last_publish"],
        }

    def _publish_web_control(self) -> None:
        with self._lock:
            enabled = bool(self._web_control["enabled"])
            neutral_burst = int(self._web_control["neutral_burst"])
            if not enabled and neutral_burst <= 0:
                return

            now = time.monotonic()
            fresh = now - self._web_control["last_command"] <= WEB_CONTROL_TIMEOUT_S
            active = enabled and bool(self._web_control["active"]) and fresh
            axes = dict(self._web_control["axes"]) if active else {
                "forward": 0.0,
                "lateral": 0.0,
                "vertical": 0.0,
                "yaw": 0.0,
            }
            if not enabled:
                self._web_control["neutral_burst"] = max(0, neutral_burst - 1)
            self._web_control["last_publish"] = time.strftime("%H:%M:%S")

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [
            axes["lateral"],
            axes["forward"],
            axes["yaw"],
            axes["vertical"],
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        msg.buttons = [0] * 12
        self._web_joy_pub.publish(msg)

    def _on_dvl(self, msg: TwistWithCovarianceStamped) -> None:
        linear = msg.twist.twist.linear
        with self._lock:
            self._health["dvl"].tick()
            self._velocity = {"x": linear.x, "y": linear.y, "z": linear.z}

    def _on_dvl_response(self, msg: CommandResponse) -> None:
        self._append_dvl_event(
            "response",
            msg.response_to,
            "",
            str(msg.result),
            msg.success,
            msg.error_message,
        )

    def _on_dvl_config(self, msg: ConfigStatus) -> None:
        with self._lock:
            self._dvl_config = {
                "updated_at": time.strftime("%H:%M:%S"),
                "response_to": msg.response_to,
                "success": msg.success,
                "error_message": msg.error_message,
                "speed_of_sound": msg.speed_of_sound,
                "acoustic_enabled": msg.acoustic_enabled,
                "dark_mode_enabled": msg.dark_mode_enabled,
                "mounting_rotation_offset": msg.mounting_rotation_offset,
                "range_mode": msg.range_mode,
                "format": msg.format,
                "type": msg.type,
            }
        self._append_dvl_event(
            "config",
            msg.response_to,
            "",
            msg.range_mode,
            msg.success,
            msg.error_message,
        )

    def _on_depth(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._health["depth"].tick()
            self._depth = {"z": msg.pose.pose.position.z}

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._health["battery"].tick()
            self._battery = {
                "voltage": _finite_or_none(msg.voltage),
                "current": _finite_or_none(msg.current),
                "temperature": _finite_or_none(msg.temperature),
                "percentage": _finite_or_none(msg.percentage),
                "present": bool(msg.present),
            }

    def _on_mavros_state(self, msg: State) -> None:
        with self._lock:
            self._health["mavros_state"].tick()
            self._mavros_state = {
                "connected": bool(msg.connected),
                "armed": bool(msg.armed),
                "guided": bool(msg.guided),
                "manual_input": bool(msg.manual_input),
                "mode": msg.mode,
                "system_status": int(msg.system_status),
                "updated_at": time.strftime("%H:%M:%S"),
            }

    def _on_imu(self, msg: Imu) -> None:
        del msg
        with self._lock:
            self._health["imu"].tick()

    def _on_joy(self, msg: Joy) -> None:
        with self._lock:
            self._health["joy"].tick()
            self._joy = {
                "axes": [round(value, 3) for value in msg.axes],
                "buttons": list(msg.buttons),
            }

    def _append_dvl_event(
        self,
        event_type: str,
        command: str,
        parameter_name: str,
        parameter_value: str,
        success: bool,
        error_message: str,
    ) -> None:
        with self._lock:
            self._dvl_events.append(
                {
                    "time": time.strftime("%H:%M:%S"),
                    "type": event_type,
                    "command": command,
                    "parameter_name": parameter_name,
                    "parameter_value": parameter_value,
                    "success": success,
                    "error_message": error_message,
                }
            )


class RosInterface:
    def __init__(self):
        self.node: LocalizationRosNode | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None

    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = LocalizationRosNode()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self) -> None:
        if self._executor is None:
            return
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if rclpy.ok():
            rclpy.shutdown()

    def status(self) -> dict:
        if self.node is None:
            return {}
        return self.node.snapshot()

    def publish_dvl_command(
        self,
        command: str,
        parameter_name: str = "",
        parameter_value: str = "",
    ) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.publish_dvl_command(command, parameter_name, parameter_value)

    def reset_dvl_dead_reckoning(self) -> None:
        self.publish_dvl_command("reset_dead_reckoning")

    def clear_path(self) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.clear_path()

    def set_localization_origin(self) -> dict:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.set_localization_origin()

    def wait_for_odom_after(self, timestamp: float, timeout: float = 5.0) -> bool:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.wait_for_odom_after(timestamp, timeout)

    def set_web_control_enabled(self, enabled: bool) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.set_web_control_enabled(enabled)

    def update_web_control_command(self, axes: dict, active: bool) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.update_web_control_command(axes, active)

    def set_armed(self, armed: bool) -> bool:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.set_armed(armed)

    def set_mode(self, mode: str) -> bool:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.set_mode(mode)


def _clamp_axis(value: object) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(numeric):
        return 0.0
    return max(-1.0, min(1.0, numeric))
