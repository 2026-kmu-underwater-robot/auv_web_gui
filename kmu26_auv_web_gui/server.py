import argparse
import asyncio
import os
import time
from pathlib import Path

import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI
from fastapi import HTTPException
from fastapi import Request
from fastapi import Response
from fastapi import WebSocket
from fastapi import WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from kmu26_auv_web_gui.bag_analyzer import analyze_bag
from kmu26_auv_web_gui.bag_analyzer import write_analysis_artifacts
from kmu26_auv_web_gui.ekf_config import read_process_noise_covariance
from kmu26_auv_web_gui.ekf_config import write_process_noise_covariance
from kmu26_auv_web_gui.process_manager import ProcessManager
from kmu26_auv_web_gui.ros_interface import ATTITUDE_MAX_TILT_DEG
from kmu26_auv_web_gui.ros_interface import READY_MODES
from kmu26_auv_web_gui.ros_interface import STILLNESS_MAX_SPEED_MPS
from kmu26_auv_web_gui.ros_interface import RosInterface


DEFAULT_BAG_TOPICS = [
    "/joy",
    "/battery",
    "/dvl/command/response",
    "/dvl/config/command",
    "/dvl/config/status",
    "/dvl/data",
    "/dvl/odometry",
    "/dvl/position",
    "/dvl/twist",
    "/depth/pose",
    "/mavros/imu/data",
    "/mavros/state",
    "/odometry/filtered",
    "/localization/path",
    "/tf",
    "/tf_static",
]

OPTIONAL_BAG_TOPICS = [
    "/joy/set_feedback",
]

ALLOWED_DVL_COMMANDS = {
    "calibrate_gyro",
    "get_config",
    "reset_dead_reckoning",
    "set_config",
}

ALLOWED_DVL_PARAMETERS = {
    "",
    "acoustic_enabled",
    "dark_mode_enabled",
    "mounting_rotation_offset",
    "mountig_rotation_offset",
    "range_mode",
    "speed_of_sound",
}

ALLOWED_CONTROL_MODES = {
    "MANUAL",
    "STABILIZE",
    "ALT_HOLD",
    "POSHOLD",
    "GUIDED",
}


def create_app(
    robot_package: str,
    robot_launch: str,
    start_dronecan_allocator: bool = True,
    dronecan_can_interface: str = "can0",
    dronecan_allocator_node_id: int = 126,
    dronecan_allocator_db: str = "",
    dronecan_python: str = "",
) -> FastAPI:
    app = FastAPI(title="AUV Localization Test GUI")
    process_manager = ProcessManager(
        robot_package=robot_package,
        robot_launch=robot_launch,
        start_dronecan_allocator=start_dronecan_allocator,
        dronecan_can_interface=dronecan_can_interface,
        dronecan_allocator_node_id=dronecan_allocator_node_id,
        dronecan_allocator_db=dronecan_allocator_db,
        dronecan_python=dronecan_python,
    )
    ros_interface = RosInterface()
    bag_selection: dict[str, object] = {
        "record_all": False,
        "topics": list(DEFAULT_BAG_TOPICS),
    }
    web_dir_override = os.environ.get("KMU26_WEB_GUI_WEB_DIR")
    if web_dir_override:
        web_dir = Path(web_dir_override)
    else:
        package_share = Path(get_package_share_directory("kmu26_auv_web_gui"))
        web_dir = package_share / "web"

    app.mount("/static", StaticFiles(directory=web_dir, follow_symlink=True), name="static")

    @app.on_event("startup")
    def on_startup() -> None:
        process_manager.start_dronecan_allocator()
        ros_interface.start()

    @app.on_event("shutdown")
    def on_shutdown() -> None:
        process_manager.stop_all()
        ros_interface.stop()

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(web_dir / "index.html")

    @app.get("/favicon.ico", include_in_schema=False)
    def favicon() -> Response:
        return Response(status_code=204)

    @app.get("/api/status")
    def status() -> dict:
        return _status(process_manager, ros_interface)

    @app.post("/api/stack/start")
    async def start_stack(request: Request) -> dict:
        body = await _json_or_empty(request)
        process_manager.start_stack(body.get("launch_args", {}))
        return _status(process_manager, ros_interface)

    @app.post("/api/stack/stop")
    def stop_stack() -> dict:
        process_manager.stop_stack()
        return _status(process_manager, ros_interface)

    @app.post("/api/dvl/command")
    async def run_dvl_command(request: Request) -> dict:
        body = await _json_or_empty(request)
        command = str(body.get("command", ""))
        parameter_name = str(body.get("parameter_name", ""))
        parameter_value = str(body.get("parameter_value", ""))
        if command not in ALLOWED_DVL_COMMANDS:
            raise HTTPException(status_code=400, detail=f"unsupported DVL command: {command}")
        if parameter_name not in ALLOWED_DVL_PARAMETERS:
            raise HTTPException(status_code=400, detail=f"unsupported DVL parameter: {parameter_name}")
        if command != "set_config" and (parameter_name or parameter_value):
            raise HTTPException(status_code=400, detail=f"{command} does not accept a parameter")
        if command == "set_config" and not parameter_name:
            raise HTTPException(status_code=400, detail="set_config requires a parameter_name")

        ros_interface.publish_dvl_command(command, parameter_name, parameter_value)
        return _status(process_manager, ros_interface)

    @app.post("/api/dvl/reset_dr")
    def reset_dvl() -> dict:
        ros_interface.reset_dvl_dead_reckoning()
        return _status(process_manager, ros_interface)

    @app.post("/api/path/clear")
    def clear_path() -> dict:
        ros_interface.clear_path()
        return _status(process_manager, ros_interface)

    @app.post("/api/localization/set_origin")
    def set_localization_origin() -> dict:
        previous_pose = ros_interface.status().get("pose", {})
        set_pose = None
        try:
            restart = process_manager.restart_localization_filter()
            fresh_since = time.monotonic()
            odom_refreshed = ros_interface.wait_for_odom_after(fresh_since, timeout=5.0)
            if odom_refreshed:
                set_pose = ros_interface.set_localization_origin()
            ros_interface.clear_path()
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        payload = _status(process_manager, ros_interface)
        payload["origin"] = {
            "method": "restart_robot_localization_then_set_pose",
            "previous_pose": previous_pose,
            "restart": restart,
            "set_pose": set_pose,
            "odom_refreshed": odom_refreshed,
        }
        return payload

    @app.post("/api/test/start")
    async def start_localization_test(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            test = await asyncio.to_thread(
                _run_localization_test,
                process_manager,
                ros_interface,
                body,
            )
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        payload = _status(process_manager, ros_interface)
        payload["test"] = test
        return payload

    @app.post("/api/test/stop")
    def stop_localization_test() -> dict:
        try:
            process_manager.stop_bag()
            process_manager.stop_stack()
            ros_interface.clear_path()
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        payload = _status(process_manager, ros_interface)
        payload["test"] = {
            "message": "Idle",
            "steps": [
                {"name": "bag", "status": "ok", "detail": "stopped"},
                {"name": "stack", "status": "ok", "detail": "stopped"},
                {"name": "path", "status": "ok", "detail": "cleared"},
            ],
        }
        return payload

    @app.post("/api/control/enable")
    async def set_control_enabled(request: Request) -> dict:
        body = await _json_or_empty(request)
        ros_interface.set_web_control_enabled(bool(body.get("enabled", False)))
        return _status(process_manager, ros_interface)

    @app.post("/api/control/command")
    async def set_control_command(request: Request) -> dict:
        body = await _json_or_empty(request)
        axes = body.get("axes", {})
        if not isinstance(axes, dict):
            raise HTTPException(status_code=400, detail="axes must be an object")
        ros_interface.update_web_control_command(axes, bool(body.get("active", False)))
        return _status(process_manager, ros_interface)

    @app.post("/api/control/arm")
    async def set_control_arm(request: Request) -> dict:
        body = await _json_or_empty(request)
        accepted = ros_interface.set_armed(bool(body.get("armed", False)))
        payload = _status(process_manager, ros_interface)
        payload["accepted"] = accepted
        return payload

    @app.post("/api/control/mode")
    async def set_control_mode(request: Request) -> dict:
        body = await _json_or_empty(request)
        mode = str(body.get("mode", ""))
        if mode not in ALLOWED_CONTROL_MODES:
            raise HTTPException(status_code=400, detail=f"unsupported mode: {mode}")
        accepted = ros_interface.set_mode(mode)
        payload = _status(process_manager, ros_interface)
        payload["accepted"] = accepted
        return payload

    @app.get("/api/ekf/process_noise")
    def get_process_noise() -> dict:
        return read_process_noise_covariance()

    @app.post("/api/ekf/process_noise")
    async def set_process_noise(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            return write_process_noise_covariance(body.get("values", []))
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/bag/start")
    async def start_bag(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            topics, record_all = _bag_record_options(body)
            _save_bag_selection(bag_selection, topics, record_all)
            output_dir = process_manager.start_bag(topics, record_all=record_all)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        payload = _status(process_manager, ros_interface)
        payload["bag_output"] = output_dir
        return payload

    @app.post("/api/bag/stop")
    def stop_bag() -> dict:
        process_manager.stop_bag()
        return _status(process_manager, ros_interface)

    @app.get("/api/bag/topics")
    def bag_topics() -> dict:
        try:
            active_topics = process_manager.list_topics()
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        topics = sorted(
            set(DEFAULT_BAG_TOPICS)
            | set(OPTIONAL_BAG_TOPICS)
            | set(active_topics)
            | set(bag_selection["topics"])
        )
        return {
            "default_topics": DEFAULT_BAG_TOPICS,
            "optional_topics": OPTIONAL_BAG_TOPICS,
            "active_topics": active_topics,
            "selected_topics": list(bag_selection["topics"]),
            "record_all": bool(bag_selection["record_all"]),
            "topics": topics,
        }

    @app.post("/api/bag/selection")
    async def save_bag_selection(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            topics, record_all = _bag_record_options(body)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        _save_bag_selection(bag_selection, topics, record_all)
        return {
            "record_all": bool(bag_selection["record_all"]),
            "topics": list(bag_selection["topics"]),
        }

    @app.post("/api/bag/analyze")
    async def run_bag_analysis(request: Request) -> dict:
        body = await _json_or_empty(request)
        process_status = process_manager.status()
        if process_status.get("bag_running"):
            raise HTTPException(status_code=400, detail="stop bag recording before analysis")

        bag_path = str(body.get("path") or process_status.get("bag_output") or "")
        if not bag_path:
            raise HTTPException(status_code=400, detail="no bag output path is available")

        try:
            analysis = await asyncio.to_thread(_analyze_and_write_bag, bag_path)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

        payload = _status(process_manager, ros_interface)
        payload["analysis"] = analysis
        return payload

    @app.websocket("/ws/status")
    async def status_ws(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(_status(process_manager, ros_interface))
                await asyncio.sleep(0.2)
        except WebSocketDisconnect:
            return

    return app


async def _json_or_empty(request: Request) -> dict:
    try:
        return await request.json()
    except Exception:
        return {}


def _status(process_manager: ProcessManager, ros_interface: RosInterface) -> dict:
    return {
        "process": process_manager.status(),
        "ros": ros_interface.status(),
    }


def _run_localization_test(
    process_manager: ProcessManager,
    ros_interface: RosInterface,
    body: dict,
) -> dict:
    launch_args = body.get("launch_args", {})
    if not isinstance(launch_args, dict):
        raise RuntimeError("launch_args must be an object")
    launch_args = {str(key): str(value) for key, value in launch_args.items()}

    steps: list[dict[str, str]] = []

    if process_manager.stack_running:
        _append_test_step(steps, "stack", "skipped", "already running")
    else:
        process_manager.start_stack(launch_args)
        _append_test_step(steps, "stack", "ok", "started")
        time.sleep(1.0)

    publisher_count = _wait_for_topic_publisher(ros_interface, "/dvl/data", timeout=8.0)
    if publisher_count <= 0:
        raise RuntimeError("DVL data publisher was not discovered")
    _append_test_step(steps, "dvl_data", "ok", f"{publisher_count} publisher(s)")

    subscriber_count = _wait_for_dvl_command_subscriber(ros_interface, timeout=4.0)
    if subscriber_count <= 0:
        raise RuntimeError("DVL command subscriber was not discovered")
    _append_test_step(steps, "dvl_command", "ok", f"{subscriber_count} subscriber(s)")

    ros_interface.neutralize_web_control()
    _append_test_step(steps, "web_control", "ok", "neutral burst sent")
    time.sleep(0.3)

    _send_dvl_command_step(
        ros_interface,
        steps,
        "set_config",
        "acoustic_enabled",
        "true",
    )
    time.sleep(0.25)
    _send_dvl_command_step(
        ros_interface,
        steps,
        "set_config",
        "range_mode",
        "auto",
    )
    time.sleep(0.25)
    _send_dvl_command_step(ros_interface, steps, "get_config")
    time.sleep(0.25)

    _require_mode_ready(ros_interface, steps)
    _require_attitude_level(ros_interface, steps)
    _require_dvl_good(ros_interface, steps, "dvl_good_before_reset")
    _require_vehicle_still(ros_interface, steps)

    _send_dvl_command_step(ros_interface, steps, "reset_dead_reckoning")
    time.sleep(0.25)
    _require_dvl_good(ros_interface, steps, "dvl_good_after_reset")
    _require_topic_alive(ros_interface, steps, "depth", "depth_fresh", timeout=5.0)

    previous_pose = ros_interface.status().get("pose", {})
    restart = process_manager.restart_localization_filter()
    _append_test_step(steps, "ekf", "ok", "restarted")
    fresh_since = time.monotonic()
    odom_refreshed = ros_interface.wait_for_odom_after(fresh_since, timeout=6.0)
    set_pose = None
    if odom_refreshed:
        set_pose = ros_interface.set_localization_origin()
        _append_test_step(steps, "origin", "ok", "set_pose sent")
    else:
        _append_test_step(steps, "origin", "warn", "odometry did not refresh")

    ros_interface.clear_path()
    _append_test_step(steps, "path", "ok", "cleared")

    warning = next((step["detail"] for step in steps if step["status"] == "warn"), "")
    return {
        "message": f"Ready with warning: {warning}" if warning else "Ready",
        "origin": {
            "method": "test_sequence_restart_then_set_pose",
            "previous_pose": previous_pose,
            "restart": restart,
            "set_pose": set_pose,
            "odom_refreshed": odom_refreshed,
        },
        "steps": steps,
    }


def _bag_record_options(
    body: dict,
    include_default_topics: bool = False,
) -> tuple[list[str], bool]:
    record_all = bool(body.get("record_all", False))
    raw_topics = body["topics"] if "topics" in body else DEFAULT_BAG_TOPICS
    if not isinstance(raw_topics, list):
        raise RuntimeError("topics must be a list")
    topics = [str(topic).strip() for topic in raw_topics if str(topic).strip()]
    if include_default_topics:
        topics = list(dict.fromkeys([*DEFAULT_BAG_TOPICS, *topics]))
    return topics, record_all


def _save_bag_selection(
    bag_selection: dict[str, object],
    topics: list[str],
    record_all: bool,
) -> None:
    bag_selection["record_all"] = record_all
    bag_selection["topics"] = list(dict.fromkeys(topics))


def _append_test_step(
    steps: list[dict[str, str]],
    name: str,
    status: str,
    detail: str = "",
) -> None:
    steps.append({"name": name, "status": status, "detail": detail})


def _wait_for_dvl_command_subscriber(
    ros_interface: RosInterface,
    timeout: float,
) -> int:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        count = ros_interface.dvl_command_subscriber_count()
        if count > 0:
            return count
        time.sleep(0.1)
    return ros_interface.dvl_command_subscriber_count()


def _wait_for_topic_publisher(
    ros_interface: RosInterface,
    topic: str,
    timeout: float,
) -> int:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        count = ros_interface.topic_publisher_count(topic)
        if count > 0:
            return count
        time.sleep(0.1)
    return ros_interface.topic_publisher_count(topic)


def _send_dvl_command_step(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
    command: str,
    parameter_name: str = "",
    parameter_value: str = "",
) -> None:
    ros_interface.publish_dvl_command(command, parameter_name, parameter_value)
    detail = parameter_name
    if parameter_value:
        detail = f"{detail}={parameter_value}" if detail else parameter_value
    _append_test_step(steps, f"dvl_{command}", "ok", detail)


def _require_dvl_good(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
    name: str,
) -> None:
    ok, snapshot = ros_interface.wait_for_dvl_good(duration_s=2.0, timeout_s=10.0)
    detail = _dvl_quality_detail(snapshot)
    if not ok:
        _append_test_step(steps, name, "bad", detail)
        raise RuntimeError(f"{name} failed: {detail}")
    _append_test_step(steps, name, "ok", detail)


def _require_attitude_level(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
) -> None:
    ok, snapshot = ros_interface.wait_for_attitude_level(
        duration_s=1.0,
        timeout_s=5.0,
        max_tilt_deg=ATTITUDE_MAX_TILT_DEG,
    )
    tilt = snapshot.get("recent_max_tilt_deg", snapshot.get("tilt_deg"))
    detail = (
        f"tilt={float(tilt):.1f} deg"
        if isinstance(tilt, (int, float))
        else str(snapshot.get("reason", "no attitude"))
    )
    if not ok:
        _append_test_step(steps, "attitude", "bad", detail)
        raise RuntimeError(f"attitude precheck failed: {detail}")
    _append_test_step(steps, "attitude", "ok", detail)


def _require_mode_ready(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
) -> None:
    ok, snapshot = ros_interface.wait_for_mode_ready(READY_MODES, timeout_s=3.0)
    detail = f"mode={snapshot.get('mode') or '--'}"
    if not ok:
        reason = str(snapshot.get("reason", detail))
        _append_test_step(steps, "mode", "bad", reason)
        raise RuntimeError(f"mode precheck failed: {reason}")
    _append_test_step(steps, "mode", "ok", detail)


def _require_vehicle_still(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
) -> None:
    ok, snapshot = ros_interface.wait_for_vehicle_still(
        duration_s=1.0,
        timeout_s=5.0,
        max_speed_mps=STILLNESS_MAX_SPEED_MPS,
    )
    speed = snapshot.get("recent_max_speed_mps")
    detail = (
        f"speed<={float(speed):.3f} m/s"
        if isinstance(speed, (int, float))
        else str(snapshot.get("reason", "no speed"))
    )
    if not ok:
        _append_test_step(steps, "stillness", "bad", detail)
        raise RuntimeError(f"stillness precheck failed: {detail}")
    _append_test_step(steps, "stillness", "ok", detail)


def _require_topic_alive(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
    topic_key: str,
    step_name: str,
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    snapshot = {}
    while time.monotonic() < deadline:
        snapshot = ros_interface.status().get("topics", {}).get(topic_key, {})
        if snapshot.get("alive"):
            _append_test_step(steps, step_name, "ok", f"age={float(snapshot.get('age', 0.0)):.2f}s")
            return
        time.sleep(0.05)
    detail = str(snapshot.get("name", topic_key)) + " stale"
    _append_test_step(steps, step_name, "bad", detail)
    raise RuntimeError(f"{step_name} failed: {detail}")


def _dvl_quality_detail(snapshot: dict) -> str:
    reason = str(snapshot.get("reason", "DVL status unknown"))
    fom = snapshot.get("fom")
    altitude = snapshot.get("altitude")
    beams = snapshot.get("valid_beams")
    parts = [reason]
    if isinstance(fom, (int, float)):
        parts.append(f"fom={fom:.3f}")
    if isinstance(altitude, (int, float)):
        parts.append(f"alt={altitude:.2f}m")
    if isinstance(beams, (int, float)):
        parts.append(f"beams={int(beams)}")
    return ", ".join(parts)


def _analyze_and_write_bag(bag_path: str) -> dict:
    result = analyze_bag(bag_path)
    write_analysis_artifacts(result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--robot-package", default="hit25_auv_ros2")
    parser.add_argument("--robot-launch", default="localization_test.launch.py")
    parser.add_argument("--start-dronecan-allocator", default="true")
    parser.add_argument("--dronecan-can-interface", default="can0")
    parser.add_argument("--dronecan-allocator-node-id", default=126, type=int)
    parser.add_argument("--dronecan-allocator-db", default="")
    parser.add_argument("--dronecan-python", default="")
    args, _ = parser.parse_known_args()

    app = create_app(
        robot_package=args.robot_package,
        robot_launch=args.robot_launch,
        start_dronecan_allocator=_parse_bool(args.start_dronecan_allocator),
        dronecan_can_interface=args.dronecan_can_interface,
        dronecan_allocator_node_id=args.dronecan_allocator_node_id,
        dronecan_allocator_db=args.dronecan_allocator_db,
        dronecan_python=args.dronecan_python,
    )
    uvicorn.run(app, host=args.host, port=args.port)


def _parse_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


if __name__ == "__main__":
    main()
