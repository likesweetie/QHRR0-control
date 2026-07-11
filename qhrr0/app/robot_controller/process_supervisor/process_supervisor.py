from __future__ import annotations

import logging
import os
import signal
import shlex
import subprocess
import time
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, BinaryIO

from ...helper.config_manage import validate_process_supervisor_config


logger = logging.getLogger(__name__)


PID_DIR = Path("/tmp/qhrr_robot_controller_processes")

LOG_ROOT_DIR_NAME = "log"
LOG_DIR_TIMESTAMP_FORMAT = "%Y%m%d_%H%M%S"

PID_FILE_SUFFIX = ".pid"
LOG_FILE_SUFFIX = ".log"

LOG_OPEN_MODE = "ab"
LOG_BUFFERING = 0

PIDFILE_ENCODING = "utf-8"

# PID 0 targets the current process group, and PID 1 is typically init/systemd.
# Managed child process IDs must therefore start from 2.
MIN_VALID_PID = 2
PID_EXIT_POLL_INTERVAL_S = 0.02

DEFAULT_STOP_TIMEOUT_S = 2.0

SHELL_EXECUTABLE = "bash"
SHELL_EXEC_FLAG = "-lc"

PROCESS_LOG_MESSAGE = "[process_supervisor] log file: %s\\n"


@dataclass(frozen=True)
class ProcessStatus:
    name: str
    alive: bool
    pid: int | None
    managed_pid: int | None
    returncode: int | None
    log_file: str


class ManagedProcess:
    def __init__(self, config: Mapping[str, Any], pid_dir: Path, log_dir: Path):
        self.config = config
        self.pidfile = pid_dir / f"{config['name']}{PID_FILE_SUFFIX}"
        self.log_file = log_dir / f"{config['name']}{LOG_FILE_SUFFIX}"
        self.process: subprocess.Popen | None = None
        self._log_handle: BinaryIO | None = None

    def start(self) -> None:
        if self.is_alive():
            return

        self._close_log_handle()

        if not self.config["command"]:
            raise ValueError(f"Process {self.config['name']} has empty command")

        self.pidfile.parent.mkdir(parents=True, exist_ok=True)

        if self.pidfile.exists():
            self.pidfile.unlink()

        stdout = None
        stderr = None

        if not self.config["new_terminal"]:
            self._log_handle = self.log_file.open(
                LOG_OPEN_MODE,
                buffering=LOG_BUFFERING,
            )
            stdout = self._log_handle
            stderr = subprocess.STDOUT

        try:
            self.process = subprocess.Popen(
                self._launch_command(),
                cwd=self.config["working_dir"],
                env=self._process_env(),
                stdout=stdout,
                stderr=stderr,
                start_new_session=True,
            )
        except Exception:
            self._close_log_handle()
            raise

    def stop(self, timeout_s: float = DEFAULT_STOP_TIMEOUT_S) -> None:
        managed_pid = self._read_pidfile()
        process = self.process

        try:
            if process is None and managed_pid is None:
                return

            if managed_pid is not None:
                self._terminate_pid(managed_pid, signal.SIGTERM)

            if process is not None and process.poll() is None:
                self._terminate_process_group(process, signal.SIGTERM)

            deadline = time.monotonic() + timeout_s

            if managed_pid is not None:
                self._wait_pid_exit(managed_pid, deadline)

            if process is not None:
                try:
                    remaining_s = max(0.0, deadline - time.monotonic())
                    process.wait(timeout=remaining_s)
                except subprocess.TimeoutExpired:
                    logger.warning(
                        "Killing process %s after stop timeout %.3fs",
                        self.config["name"],
                        timeout_s,
                    )

                    if managed_pid is not None:
                        self._terminate_pid(managed_pid, signal.SIGKILL)

                    self._terminate_process_group(process, signal.SIGKILL)
                    process.wait(timeout=timeout_s)

            elif managed_pid is not None and self._pid_is_running(managed_pid):
                logger.warning(
                    "Killing managed process %s after stop timeout %.3fs",
                    self.config["name"],
                    timeout_s,
                )
                self._terminate_pid(managed_pid, signal.SIGKILL)
                self._wait_pid_exit(managed_pid, time.monotonic() + timeout_s)
        finally:
            self._cleanup_pidfile()
            self._close_log_handle()

    def is_alive(self) -> bool:
        if self.process is not None and self.process.poll() is None:
            return True

        managed_pid = self._read_pidfile()
        return managed_pid is not None and self._pid_is_running(managed_pid)

    @property
    def health(self) -> ProcessStatus:
        returncode = self.process.poll() if self.process is not None else None
        process_alive = self.process is not None and returncode is None
        managed_pid = self._read_pidfile()

        return ProcessStatus(
            name=self.config["name"],
            alive=process_alive or (
                managed_pid is not None and self._pid_is_running(managed_pid)
            ),
            pid=self.process.pid if process_alive else None,
            managed_pid=managed_pid,
            returncode=returncode,
            log_file=str(self.log_file),
        )

    def _launch_command(self) -> list[str]:
        if not self.config["new_terminal"]:
            return list(self.config["command"])

        if not self.config["terminal_command"]:
            raise ValueError(f"Process {self.config['name']} requires terminal_command")

        workdir = Path(self.config["working_dir"]).resolve()

        pidfile_part = (
            f"printf '%s\\n' \"$$\" > {shlex.quote(str(self.pidfile))} && "
        )

        log_part = (
            f"exec > >(tee -a {shlex.quote(str(self.log_file))}) 2>&1 && "
            f"printf '{PROCESS_LOG_MESSAGE}' "
            f"{shlex.quote(str(self.log_file))} && "
        )

        shell_command = (
            f"cd {shlex.quote(str(workdir))} && "
            f"{pidfile_part}"
            f"{log_part}"
            f"exec {shlex.join(self.config['command'])}"
        )

        return list(self.config["terminal_command"]) + [
            SHELL_EXECUTABLE,
            SHELL_EXEC_FLAG,
            shell_command,
        ]

    def _process_env(self) -> dict[str, str]:
        env = dict(os.environ)
        env.update(self.config["env_vars"])
        return env

    def _read_pidfile(self) -> int | None:
        if not self.pidfile.exists():
            return None

        try:
            raw_pid = self.pidfile.read_text(encoding=PIDFILE_ENCODING).strip()
        except OSError as exc:
            logger.warning("Failed to read pidfile %s: %s", self.pidfile, exc)
            return None

        try:
            value = int(raw_pid)
        except ValueError:
            logger.warning("Invalid pidfile %s: %r", self.pidfile, raw_pid)
            return None

        if value < MIN_VALID_PID:
            logger.warning(
                "Invalid pidfile %s: pid must be >= %d: %d",
                self.pidfile,
                MIN_VALID_PID,
                value,
            )
            return None

        return value

    def _cleanup_pidfile(self) -> None:
        try:
            self.pidfile.unlink()
        except FileNotFoundError:
            return

    def _close_log_handle(self) -> None:
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None

    @staticmethod
    def _terminate_process_group(process: subprocess.Popen, signum: int) -> None:
        try:
            os.killpg(os.getpgid(process.pid), signum)
        except ProcessLookupError:
            return
        except PermissionError as exc:
            logger.warning(
                "Failed to signal process group for pid %s: %s",
                process.pid,
                exc,
            )
            if signum == signal.SIGTERM:
                process.terminate()
            else:
                process.kill()

    @staticmethod
    def _terminate_pid(pid: int, signum: int) -> None:
        try:
            os.kill(pid, signum)
        except ProcessLookupError:
            return
        except PermissionError as exc:
            logger.warning("Failed to signal managed pid %s: %s", pid, exc)

    @staticmethod
    def _wait_pid_exit(pid: int, deadline: float) -> None:
        while time.monotonic() < deadline:
            if not ManagedProcess._pid_is_running(pid):
                return
            time.sleep(PID_EXIT_POLL_INTERVAL_S)

    @staticmethod
    def _pid_is_running(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True


class ProcessSupervisor:
    def __init__(self, process_configs: Sequence[Mapping[str, Any]]):
        validate_process_supervisor_config(process_configs)
        self.pid_dir = PID_DIR
        self.log_dir = (
            Path.cwd()
            / LOG_ROOT_DIR_NAME
            / datetime.now().strftime(LOG_DIR_TIMESTAMP_FORMAT)
        )
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self._processes: dict[str, ManagedProcess] = {
            config["name"]: ManagedProcess(
                config=config,
                pid_dir=self.pid_dir,
                log_dir=self.log_dir,
            )
            for config in process_configs
        }

    @property
    def process_configs(self) -> dict[str, Mapping[str, Any]]:
        return {
            name: process.config
            for name, process in self._processes.items()
        }

    def _process(self, name: str) -> ManagedProcess:
        if not name:
            raise ValueError("Process name must not be empty")

        try:
            return self._processes[name]
        except KeyError:
            raise KeyError(f"Unknown process: {name}") from None

    def start_by_name(self, name: str) -> None:
        self._process(name).start()

    def stop_by_name(self, name: str, timeout_s: float = DEFAULT_STOP_TIMEOUT_S) -> None:
        self._process(name).stop(timeout_s=timeout_s)

    def is_alive(self, name: str) -> bool:
        return self._process(name).is_alive()

    def start_all(self) -> None:
        for process in sorted(
            self._processes.values(),
            key=lambda item: int(item.config["start_order"]),
        ):
            process.start()

    def stop_all(self, timeout_s: float = DEFAULT_STOP_TIMEOUT_S) -> None:
        for process in sorted(
            self._processes.values(),
            key=lambda item: int(item.config["stop_order"]),
        ):
            process.stop(timeout_s=timeout_s)

    def health(self) -> dict[str, ProcessStatus]:
        return {
            name: process.health
            for name, process in self._processes.items()
        }

    def status(self) -> dict[str, dict[str, object]]:
        return {
            name: {
                "config": dict(process.config),
                **asdict(process.health),
            }
            for name, process in self._processes.items()
        }
