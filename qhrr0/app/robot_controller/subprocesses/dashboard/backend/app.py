from __future__ import annotations

import asyncio
import json
import logging
from pathlib import Path
from typing import Any

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .command_api import CommandError, CommandService
from .operator_commands import OperatorCommandWriter
from .robot_state_shm import DashboardRobotStateReader
from .socketcan_io import CAN_FRAME_SIZE, open_can_socket, parse_can_frame
from .state import MonitorState
from qhrr0.app.hal.can_bus.process_client import CANProcessClient
from qhrr0.app.robot_controller.process_supervisor import ProcessSupervisor
from qhrr0.factory.app_factory.dashboard_runtime import load_dashboard_runtime_config
from qhrr0.factory.app_factory.schema import DashboardRuntimeConfig, ProcessLaunchSpec


ROOT = Path(__file__).resolve().parents[1]
FRONTEND_DIR = ROOT / "frontend"
logger = logging.getLogger(__name__)
PROTECTED_PROCESS_NAMES = frozenset({"can_daemon", "dashboard"})


class RawSendRequest(BaseModel):
    can_id: int | str
    data: str


class MotorZeroRequest(BaseModel):
    offset_count: int | None = None
    offset_deg: float | None = None

    def resolved_offset_count(self) -> int:
        if self.offset_count is not None and self.offset_deg is not None:
            raise ValueError("Use either offset_count or offset_deg, not both")
        if self.offset_deg is not None:
            return round_half_away_from_zero(float(self.offset_deg) * 100.0)
        if self.offset_count is not None:
            return int(self.offset_count)
        return 0


class OperatorZeroTargetRequest(MotorZeroRequest):
    can_id: int | str


class OperatorZeroSetRequest(BaseModel):
    targets: list[OperatorZeroTargetRequest] = Field(default_factory=list)


class ConfirmRequest(BaseModel):
    confirmed: bool = False


def require_section(config: dict[str, Any], section: str) -> dict[str, Any]:
    value = config.get(section)
    if not isinstance(value, dict):
        raise ValueError(f"Dashboard config section '{section}' is required")
    return value


def nested(config: dict[str, Any], section: str, key: str) -> Any:
    section_value = require_section(config, section)
    if key not in section_value:
        raise ValueError(f"Dashboard config key '{section}.{key}' is required")
    return section_value[key]


def parse_int_maybe_hex(value: Any) -> int:
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


def require_hz(value: float, field: str, *, lo: float = 0.1, hi: float = 1000.0) -> float:
    value = float(value)
    if value < lo or value > hi:
        raise ValueError(f"{field} must be in [{lo}, {hi}] Hz")
    return value


def load_actuator_configs(config: dict[str, Any]) -> tuple[dict, ...]:
    raw = config.get("actuators")
    if not isinstance(raw, list):
        raise ValueError("Dashboard config key 'actuators' must be a list")
    if not raw:
        raise ValueError("Dashboard config key 'actuators' must not be empty")

    actuators = []
    seen_can_ids: set[int] = set()
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f"Dashboard actuator #{index} must be a mapping")
        if "can_id" not in item:
            raise ValueError(f"Dashboard actuator #{index} is missing 'can_id'")
        if "name" not in item:
            raise ValueError(f"Dashboard actuator #{index} is missing 'name'")
        can_id = parse_int_maybe_hex(item["can_id"])
        if can_id <= 0:
            raise ValueError(f"Dashboard actuator #{index} has invalid CAN ID 0x{can_id:X}")
        if can_id in seen_can_ids:
            raise ValueError(f"Duplicate Dashboard actuator CAN ID 0x{can_id:X}")
        seen_can_ids.add(can_id)
        actuators.append(
            {
                "name": str(item["name"]),
                "can_id": can_id,
            }
        )
    return tuple(actuators)


def make_state(config: dict[str, Any]) -> MonitorState:
    return MonitorState(
        iface=str(nested(config, "can", "iface")),
        bitrate=int(nested(config, "can", "bitrate")),
        bus_window_s=float(nested(config, "can", "bus_window_s")),
        heartbeat_window_s=float(nested(config, "can", "heartbeat_window_s")),
        node_timeout_s=float(nested(config, "can", "node_timeout_s")),
        stuff_factor=float(nested(config, "can", "stuff_factor")),
        feedback_position_max_rad=float(nested(config, "spg", "feedback_position_max_rad")),
        iq_full_scale_count=float(nested(config, "spg", "iq_full_scale_count")),
        iq_full_scale_current_a=float(nested(config, "spg", "iq_full_scale_current_a")),
        mit_p_max_rad=float(nested(config, "spg", "p_max_rad")),
        mit_v_max_rad_s=float(nested(config, "spg", "v_max_rad_s")),
        mit_kp_max=float(nested(config, "spg", "kp_max")),
        mit_kd_max=float(nested(config, "spg", "kd_max")),
        mit_tau_max_nm=float(nested(config, "spg", "tau_max_nm")),
        imu_request_id=parse_int_maybe_hex(nested(config, "imu", "request_id")),
        imu_quat_id=parse_int_maybe_hex(nested(config, "imu", "quat_id")),
        imu_gyro_id=parse_int_maybe_hex(nested(config, "imu", "gyro_id")),
        imu_quat_scale=float(nested(config, "imu", "quat_scale")),
        imu_gyro_scale=float(nested(config, "imu", "gyro_scale")),
        imu_normalize_quat=bool(nested(config, "imu", "normalize_quat")),
        actuator_configs=load_actuator_configs(config),
        tx_enabled=bool(nested(config, "safety", "tx_enabled_by_default")),
        allow_actuator_commands=bool(nested(config, "safety", "allow_actuator_commands")),
        mit_poll_hz=require_hz(nested(config, "spg", "default_mit_poll_hz"), "spg.default_mit_poll_hz"),
    )


def parse_can_id(value: int | str) -> int:
    if isinstance(value, int):
        return value
    return int(value.strip(), 0)


def parse_hex_payload(value: str) -> bytes:
    compact = value.replace(" ", "").replace("_", "").replace("-", "")
    if len(compact) % 2 != 0:
        raise ValueError("Payload hex string must contain whole bytes")
    data = bytes.fromhex(compact)
    if len(data) > 8:
        raise ValueError("Classical CAN payload must be <= 8 bytes")
    return data


def round_half_away_from_zero(value: float) -> int:
    if value >= 0.0:
        return int(value + 0.5)
    return int(value - 0.5)


def build_process_supervisor(processes: tuple[ProcessLaunchSpec, ...]) -> ProcessSupervisor:
    supervisor = ProcessSupervisor()
    for process in processes:
        supervisor.add_process(
            name=process.name,
            command=process.command,
            start_order=process.start_order,
            stop_order=process.stop_order,
            new_terminal=process.new_terminal,
            terminal_command=process.terminal_command,
            working_dir=process.working_dir,
            env_vars=dict(process.env_vars),
        )
    return supervisor


class DashboardBackend:
    def __init__(self, runtime: DashboardRuntimeConfig) -> None:
        self.runtime = runtime
        self.config = runtime.effective_config()
        self.state = make_state(self.config)
        self.process_supervisor = build_process_supervisor(runtime.processes)
        self.robot_state_reader = (
            DashboardRobotStateReader(
                control_shm_name=str(nested(self.config, "robot_controller_state", "control_shm_name")),
                dashboard_shm_name=str(nested(self.config, "robot_controller_state", "dashboard_shm_name")),
                stale_timeout_s=float(nested(self.config, "robot_controller_state", "stale_timeout_s")),
                state=self.state,
            )
            if bool(nested(self.config, "robot_controller_state", "enabled"))
            else None
        )
        self.operator_commands = OperatorCommandWriter(
            name=str(nested(self.config, "robot_controller_state", "operator_shm_name")),
            size_bytes=int(nested(self.config, "robot_controller_state", "operator_shm_size_bytes")),
            source="dashboard",
        )
        self.commands = CommandService(
            self.state,
            CANProcessClient(
                socket_path=str(nested(self.config, "can_daemon", "ipc_socket_path")),
                connect_timeout_s=float(nested(self.config, "can_daemon", "connect_timeout_s")),
                rx_enabled=False,
            ),
            controller_safety_state_provider=(
                self.current_controller_safety_state if self.robot_state_reader is not None else None
            ),
            controller_safety_reason_provider=(
                self.current_controller_safety_reason if self.robot_state_reader is not None else None
            ),
        )

    def current_controller_snapshot(self) -> dict | None:
        snapshot = self.dashboard_snapshot()
        controller = snapshot.get("robot_controller")
        if not isinstance(controller, dict):
            return None
        if controller.get("status") != "online":
            return None
        return controller

    def current_controller_safety_state(self) -> str | None:
        controller = self.current_controller_snapshot()
        if controller is None:
            return None
        value = controller.get("safety_state")
        return None if value is None else str(value)

    def current_controller_safety_reason(self) -> str | None:
        controller = self.current_controller_snapshot()
        if controller is None:
            return None
        value = controller.get("safety_reason")
        return None if value is None else str(value)

    def publish_operator_zero_set(self, targets: list[tuple[int, int]]) -> int:
        self.commands.require_controller_state(
            action_name="Operator zero set",
            allowed_states={"NORMAL"},
        )
        for can_id, offset_count in targets:
            if self.state.actuator_config_for_can_id(can_id) is None:
                raise ValueError(f"Unknown actuator CAN ID 0x{can_id:03X}")
            if not (-32768 <= int(offset_count) <= 32767):
                raise ValueError(f"MIT zero offset_count out of int16 range: {offset_count}")
        return self.operator_commands.publish_zero_set(targets)

    def dashboard_snapshot(self) -> dict:
        if self.robot_state_reader is None:
            snapshot = self.state.snapshot()
        else:
            snapshot = self.robot_state_reader.dashboard_snapshot()
        snapshot["processes"] = self.process_status_rows()
        return snapshot

    def process_status_rows(self) -> list[dict[str, Any]]:
        return [
            {
                "name": name,
                **dict(info),
                "manageable": name not in PROTECTED_PROCESS_NAMES,
                "management_reason": (
                    "managed by robot controller supervisor"
                    if name in PROTECTED_PROCESS_NAMES
                    else ""
                ),
            }
            for name, info in sorted(self.process_supervisor.status().items())
        ]

    def ensure_process_name(self, name: str) -> str:
        if name not in self.process_supervisor.process_configs:
            raise HTTPException(status_code=404, detail=f"Unknown process: {name}")
        return name

    @staticmethod
    def ensure_process_manageable(name: str) -> None:
        if name in PROTECTED_PROCESS_NAMES:
            raise HTTPException(
                status_code=403,
                detail=f"Process '{name}' is managed by the robot controller supervisor",
            )

    async def socketcan_loop(self) -> None:
        reconnect_delay_s = 1.0
        max_frames_per_tick = 4096
        sock = None
        while True:
            if sock is None:
                try:
                    sock = open_can_socket(self.state.iface)
                    self.state.socket_status = "connected"
                    self.state.socket_error = None
                except OSError as exc:
                    self.state.socket_status = "disconnected"
                    self.state.socket_error = str(exc)
                    await asyncio.sleep(reconnect_delay_s)
                    continue

            try:
                frames_read = 0
                while frames_read < max_frames_per_tick:
                    try:
                        frame_bytes = sock.recv(CAN_FRAME_SIZE)
                    except BlockingIOError:
                        break
                    self.state.mark_rx(parse_can_frame(frame_bytes))
                    frames_read += 1
            except OSError as exc:
                self.state.socket_status = "disconnected"
                self.state.socket_error = str(exc)
                try:
                    sock.close()
                except OSError as close_exc:
                    logger.warning("CAN socket close failed after RX error: %s", close_exc)
                sock = None

            await asyncio.sleep(0 if frames_read else 0.001)

    async def mit_poll_loop(self) -> None:
        next_send_t = asyncio.get_running_loop().time()
        while True:
            if self.state.mit_poll_can_ids:
                hz = float(self.state.mit_poll_hz)
                period_s = 1.0 / hz
                now = asyncio.get_running_loop().time()
                if now < next_send_t:
                    await asyncio.sleep(next_send_t - now)

                try:
                    for can_id in sorted(self.state.mit_poll_can_ids):
                        self.commands.motor_mit_hold(can_id)
                except CommandError as exc:
                    self.state.socket_error = str(exc)

                now = asyncio.get_running_loop().time()
                next_send_t += period_s
                if next_send_t < now:
                    next_send_t = now + period_s
            else:
                next_send_t = asyncio.get_running_loop().time()
                await asyncio.sleep(0.05)

    async def close(self) -> None:
        if self.robot_state_reader is not None:
            self.robot_state_reader.close()
        self.operator_commands.close()


def create_app(runtime: DashboardRuntimeConfig) -> FastAPI:
    backend = DashboardBackend(runtime)
    app = FastAPI(title="QHRR Robot State")
    app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")

    @app.on_event("startup")
    async def startup() -> None:
        app.state.tasks = [
            asyncio.create_task(backend.socketcan_loop()),
            asyncio.create_task(backend.mit_poll_loop()),
        ]

    @app.on_event("shutdown")
    async def shutdown() -> None:
        for task in getattr(app.state, "tasks", []):
            task.cancel()
        await asyncio.gather(*getattr(app.state, "tasks", []), return_exceptions=True)
        await backend.close()

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(FRONTEND_DIR / "index.html")

    @app.get("/processes")
    async def processes_page() -> FileResponse:
        return FileResponse(FRONTEND_DIR / "processes.html")

    @app.get("/shm")
    async def shm_page() -> FileResponse:
        return FileResponse(FRONTEND_DIR / "shm.html")

    @app.get("/api/config")
    async def api_config() -> dict:
        return backend.config

    @app.get("/api/state")
    async def api_state() -> dict:
        return backend.dashboard_snapshot()

    @app.get("/api/processes")
    async def api_processes() -> dict:
        return {"ok": True, "processes": backend.process_status_rows()}

    @app.post("/api/processes/{name}/start")
    async def api_process_start(name: str) -> dict:
        name = backend.ensure_process_name(name)
        backend.ensure_process_manageable(name)
        logger.info("Dashboard requested process start: %s", name)
        try:
            await asyncio.to_thread(backend.process_supervisor.start_by_name, name)
        except (KeyError, RuntimeError, ValueError, OSError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"ok": True, "processes": backend.process_status_rows()}

    @app.post("/api/processes/{name}/stop")
    async def api_process_stop(name: str) -> dict:
        name = backend.ensure_process_name(name)
        backend.ensure_process_manageable(name)
        logger.info("Dashboard requested process stop: %s", name)
        try:
            await asyncio.to_thread(
                backend.process_supervisor.stop_by_name,
                name,
                float(runtime.shutdown_timeout_s),
            )
        except (KeyError, RuntimeError, ValueError, OSError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"ok": True, "processes": backend.process_status_rows()}

    @app.post("/api/tx/lock")
    async def tx_lock() -> dict:
        raise HTTPException(status_code=410, detail="Direct CAN TX control was removed")

    @app.post("/api/tx/unlock")
    async def tx_unlock() -> dict:
        raise HTTPException(status_code=410, detail="Direct CAN TX control was removed")

    @app.post("/api/can/send")
    async def can_send(_req: RawSendRequest) -> dict:
        raise HTTPException(status_code=410, detail="Raw CAN send was removed")

    @app.post("/api/operator/fault-clear")
    async def operator_fault_clear() -> dict:
        try:
            command_id = backend.operator_commands.publish(clear_fault=True)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {"ok": True, "command_id": command_id}

    @app.post("/api/operator/arm")
    async def operator_arm() -> dict:
        try:
            command_id = backend.operator_commands.publish(arm=True)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {"ok": True, "command_id": command_id}

    @app.post("/api/operator/run")
    async def operator_run() -> dict:
        try:
            command_id = backend.operator_commands.publish(run=True)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {"ok": True, "command_id": command_id}

    @app.post("/api/operator/damping")
    async def operator_damping() -> dict:
        try:
            command_id = backend.operator_commands.publish(damping=True)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {"ok": True, "command_id": command_id}

    @app.post("/api/operator/zero-set")
    async def operator_zero_set(req: OperatorZeroSetRequest | None = None) -> dict:
        try:
            targets: list[tuple[int, int]] = []
            request_targets = [] if req is None else req.targets
            for target in request_targets:
                can_id = parse_can_id(target.can_id)
                targets.append((can_id, target.resolved_offset_count()))
            command_id = backend.publish_operator_zero_set(targets)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except CommandError as exc:
            raise HTTPException(status_code=403, detail=str(exc)) from exc
        except (FileNotFoundError, RuntimeError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {"ok": True, "command_id": command_id}

    @app.post("/api/operator/estop")
    async def operator_estop() -> dict:
        try:
            command_id = backend.operator_commands.publish(estop=True)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {"ok": True, "command_id": command_id}

    @app.post("/api/actuator/{can_id}/enter")
    async def motor_enter(_can_id: str) -> dict:
        raise HTTPException(status_code=410, detail="Direct actuator enable was removed")

    @app.post("/api/actuator/{can_id}/exit")
    async def motor_exit(_can_id: str) -> dict:
        raise HTTPException(status_code=410, detail="Direct actuator disable was removed")

    @app.post("/api/actuator/{can_id}/zero")
    async def motor_zero(can_id: str, req: MotorZeroRequest) -> dict:
        try:
            offset_count = req.resolved_offset_count()
            command_id = backend.publish_operator_zero_set([(parse_can_id(can_id), offset_count)])
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except CommandError as exc:
            raise HTTPException(status_code=403, detail=str(exc)) from exc
        except (FileNotFoundError, RuntimeError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {"ok": True, "command_id": command_id}

    @app.post("/api/actuator/{can_id}/mit-poll/start")
    async def motor_mit_poll_start(_can_id: str, _req: ConfirmRequest) -> dict:
        raise HTTPException(status_code=410, detail="Direct actuator MIT polling was removed")

    @app.post("/api/actuator/{can_id}/mit-poll/stop")
    async def motor_mit_poll_stop(_can_id: str) -> dict:
        raise HTTPException(status_code=410, detail="Direct actuator MIT polling was removed")

    @app.websocket("/ws/state")
    async def ws_state(websocket: WebSocket) -> None:
        await websocket.accept()
        state_hz = require_hz(
            nested(backend.config, "dashboard", "state_update_rate"),
            "dashboard.state_update_rate",
            lo=1.0,
            hi=60.0,
        )
        delay = 1.0 / state_hz
        try:
            while True:
                await websocket.send_text(json.dumps(backend.dashboard_snapshot()))
                await asyncio.sleep(delay)
        except WebSocketDisconnect:
            return

    return app


def main(runtime_config_path: str | Path) -> None:
    runtime = load_dashboard_runtime_config(runtime_config_path)
    app = create_app(runtime)
    config = runtime.effective_config()
    host = str(nested(config, "dashboard", "host"))
    port = int(nested(config, "dashboard", "port"))
    uvicorn.run(
        app,
        host=host,
        port=port,
        reload=False,
    )


if __name__ == "__main__":
    raise SystemExit("Use qhrr0.app.robot_controller.subprocesses.dashboard.main")
