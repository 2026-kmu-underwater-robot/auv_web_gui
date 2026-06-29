import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Iterable


def subprocess_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    env.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "0")
    return env


class ManagedProcess:
    def __init__(self, name: str, cmd: list[str], log_buffer: deque[str]):
        self.name = name
        self.cmd = cmd
        self.log_buffer = log_buffer
        self.process: subprocess.Popen[str] | None = None
        self.pgid: int | None = None
        self._reader_thread: threading.Thread | None = None

    def start(self) -> None:
        if self.is_running:
            raise RuntimeError(f"{self.name} is already running")

        self._log(f"$ {' '.join(self.cmd)}")
        self.process = subprocess.Popen(
            self.cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=subprocess_env(),
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.pgid = os.getpgid(self.process.pid)
        self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
        self._reader_thread.start()

    def stop(self) -> None:
        if self.process is None and self.pgid is None:
            return

        self._log(f"[{self.name}] stopping")
        pgid = self.pgid
        if pgid is None and self.process is not None:
            try:
                pgid = os.getpgid(self.process.pid)
            except ProcessLookupError:
                return

        try:
            _kill_process_group(pgid, signal.SIGTERM)
            if self.process is not None and self.process.poll() is None:
                self.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._log(f"[{self.name}] force killing")
            _kill_process_group(pgid, signal.SIGKILL)
            if self.process is not None:
                self.process.wait(timeout=2.0)
            return

        if not _wait_for_process_group_exit(pgid, timeout=2.0):
            self._log(f"[{self.name}] force killing remaining process group {pgid}")
            _kill_process_group(pgid, signal.SIGKILL)
            _wait_for_process_group_exit(pgid, timeout=2.0)

    @property
    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def _read_output(self) -> None:
        assert self.process is not None
        if self.process.stdout is None:
            return
        for line in self.process.stdout:
            self._log(f"[{self.name}] {line.rstrip()}")
        return_code = self.process.wait()
        self._log(f"[{self.name}] exited with code {return_code}")

    def _log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        entry = f"{timestamp} {message}"
        self.log_buffer.append(entry)
        print(entry, file=sys.stdout, flush=True)


class ProcessManager:
    def __init__(
        self,
        robot_package: str = "hit25_auv_ros2",
        robot_launch: str = "localization_test.launch.py",
    ):
        self.robot_package = robot_package
        self.robot_launch = robot_launch
        self.logs: deque[str] = deque(maxlen=500)
        self._stack: ManagedProcess | None = None
        self._bag: ManagedProcess | None = None
        self._bag_output: str = ""

    def start_stack(self, launch_args: dict[str, str] | None = None) -> None:
        if self.stack_running:
            raise RuntimeError("robot stack is already running")

        args = launch_args or {}
        cmd = ["ros2", "launch", self.robot_package, self.robot_launch]
        for key, value in args.items():
            if value != "":
                cmd.append(f"{key}:={value}")

        self._stack = ManagedProcess("localization_test", cmd, self.logs)
        self._stack.start()

    def stop_stack(self) -> None:
        if self._stack:
            self._stack.stop()
        self._stop_orphaned_stack_groups()

    def start_bag(
        self,
        topics: Iterable[str] | None = None,
        output_root: str = "~/auv_localization_bags",
        record_all: bool = False,
    ) -> str:
        if self._bag and self._bag.is_running:
            raise RuntimeError("bag recording is already running")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(output_root).expanduser() / f"localization_{timestamp}"
        output_dir.parent.mkdir(parents=True, exist_ok=True)
        topic_list = [topic for topic in (topics or []) if topic]
        if record_all:
            cmd = ["ros2", "bag", "record", "-o", str(output_dir), "-a"]
        else:
            if not topic_list:
                raise RuntimeError("at least one topic must be selected")
            cmd = ["ros2", "bag", "record", "-o", str(output_dir), *topic_list]
        self._bag = ManagedProcess("bag", cmd, self.logs)
        self._bag_output = str(output_dir)
        self._bag.start()
        return str(output_dir)

    def list_topics(self) -> list[str]:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=subprocess_env(),
            text=True,
            timeout=3.0,
        )
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or "failed to list ROS topics")
        return sorted(line.strip() for line in result.stdout.splitlines() if line.strip())

    def stop_bag(self) -> None:
        if self._bag:
            self._bag.stop()

    def status(self) -> dict:
        return {
            "stack_running": self.stack_running,
            "bag_running": bool(self._bag and self._bag.is_running),
            "bag_output": self._bag_output,
            "logs": list(self.logs)[-80:],
        }

    def stop_all(self) -> None:
        self.stop_bag()
        self.stop_stack()

    @property
    def stack_running(self) -> bool:
        return bool(self._stack and self._stack.is_running) or bool(
            _matching_launch_process_groups(self.robot_package, self.robot_launch)
        )

    def _stop_orphaned_stack_groups(self) -> None:
        groups = _matching_launch_process_groups(self.robot_package, self.robot_launch)
        for pgid in sorted(groups):
            self._log(f"[localization_test] stopping orphaned process group {pgid}")
            _kill_process_group(pgid, signal.SIGTERM)
        for pgid in sorted(groups):
            if _wait_for_process_group_exit(pgid, timeout=5.0):
                continue
            self._log(f"[localization_test] force killing orphaned process group {pgid}")
            _kill_process_group(pgid, signal.SIGKILL)
            _wait_for_process_group_exit(pgid, timeout=2.0)

    def _log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        entry = f"{timestamp} {message}"
        self.logs.append(entry)
        print(entry, file=sys.stdout, flush=True)


def _matching_launch_process_groups(robot_package: str, robot_launch: str) -> set[int]:
    groups: set[int] = set()
    current_pid = os.getpid()
    for pid, cmdline in _iter_process_cmdlines():
        if pid == current_pid or not _is_matching_ros2_launch(cmdline, robot_package, robot_launch):
            continue
        try:
            groups.add(os.getpgid(pid))
        except ProcessLookupError:
            continue
    return groups


def _is_matching_ros2_launch(
    cmdline: list[str],
    robot_package: str,
    robot_launch: str,
) -> bool:
    for index, arg in enumerate(cmdline[:-3]):
        if os.path.basename(arg) != "ros2":
            continue
        if cmdline[index + 1 : index + 4] == ["launch", robot_package, robot_launch]:
            return True
    return False


def _iter_process_cmdlines() -> Iterable[tuple[int, list[str]]]:
    proc = Path("/proc")
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if not raw:
            continue
        cmdline = [part.decode(errors="replace") for part in raw.split(b"\0") if part]
        if cmdline:
            yield int(entry.name), cmdline


def _kill_process_group(pgid: int, sig: signal.Signals) -> None:
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        return


def _wait_for_process_group_exit(pgid: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _process_group_has_live_members(pgid):
            return True
        time.sleep(0.1)
    return not _process_group_has_live_members(pgid)


def _process_group_has_live_members(pgid: int) -> bool:
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            pid = int(entry.name)
            if os.getpgid(pid) != pgid:
                continue
            stat = (entry / "stat").read_text(encoding="utf-8", errors="replace")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        parts = stat.rsplit(")", 1)
        if len(parts) != 2:
            continue
        fields = parts[1].strip().split()
        if fields and fields[0] != "Z":
            return True
    return False
