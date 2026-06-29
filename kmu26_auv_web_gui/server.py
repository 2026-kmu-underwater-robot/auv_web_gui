import argparse
import asyncio
import os
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

from kmu26_auv_web_gui.ekf_config import read_process_noise_covariance
from kmu26_auv_web_gui.ekf_config import write_process_noise_covariance
from kmu26_auv_web_gui.process_manager import ProcessManager
from kmu26_auv_web_gui.ros_interface import RosInterface


DEFAULT_BAG_TOPICS = [
    "/joy",
    "/battery",
    "/dvl/data",
    "/dvl/twist",
    "/depth/pose",
    "/mavros/imu/data",
    "/mavros/state",
    "/odometry/filtered",
    "/localization/path",
    "/tf",
    "/tf_static",
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


def create_app(robot_package: str, robot_launch: str) -> FastAPI:
    app = FastAPI(title="AUV Localization Test GUI")
    process_manager = ProcessManager(robot_package=robot_package, robot_launch=robot_launch)
    ros_interface = RosInterface()
    web_dir_override = os.environ.get("KMU26_WEB_GUI_WEB_DIR")
    if web_dir_override:
        web_dir = Path(web_dir_override)
    else:
        package_share = Path(get_package_share_directory("kmu26_auv_web_gui"))
        web_dir = package_share / "web"

    app.mount("/static", StaticFiles(directory=web_dir), name="static")

    @app.on_event("startup")
    def on_startup() -> None:
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
        record_all = bool(body.get("record_all", False))
        topics = body.get("topics") or DEFAULT_BAG_TOPICS
        output_dir = process_manager.start_bag(topics, record_all=record_all)
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
        topics = sorted(set(DEFAULT_BAG_TOPICS) | set(active_topics))
        return {
            "default_topics": DEFAULT_BAG_TOPICS,
            "active_topics": active_topics,
            "topics": topics,
        }

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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--robot-package", default="hit25_auv_ros2")
    parser.add_argument("--robot-launch", default="localization_test.launch.py")
    args = parser.parse_args()

    app = create_app(robot_package=args.robot_package, robot_launch=args.robot_launch)
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
