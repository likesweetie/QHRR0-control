from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any

import yaml
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .can_decode import E2BOX_CMD_GET_ALL, E2BOX_REQ_ID
from .command_api import CommandError, CommandService
from .socketcan_io import CAN_FRAME_SIZE, open_can_socket, parse_can_frame
from .state import MonitorState


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config.yaml"
FRONTEND_DIR = ROOT / "frontend"


class RawSendRequest(BaseModel):
    can_id: int | str
    data: str


class ImuHzRequest(BaseModel):
    hz: float


class MotorZeroRequest(BaseModel):
    offset_count: int = 0


class ConfirmRequest(BaseModel):
    confirmed: bool = False


def load_config() -> dict[str, Any]:
    with CONFIG_PATH.open("r", encoding="utf-8") as fp:
        return yaml.safe_load(fp) or {}


def nested(config: dict[str, Any], section: str, key: str, default: Any) -> Any:
    return config.get(section, {}).get(key, default)


def parse_int_maybe_hex(value: Any) -> int:
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


def load_actuator_configs(config: dict[str, Any]) -> tuple[dict, ...]:
    raw = config.get("actuators")
    if raw is None:
        raw = config.get("can", {}).get("actuators", [])
    if not isinstance(raw, list):
        return ()

    actuators = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            continue
        motor_id = parse_int_maybe_hex(item.get("motor_id", index))
        can_id = parse_int_maybe_hex(item.get("can_id", 0x140 + motor_id))
        actuators.append(
            {
                "name": str(item.get("name") or f"SPG motor {motor_id}"),
                "motor_id": motor_id,
                "can_id": can_id,
            }
        )
    return tuple(actuators)


def make_state(config: dict[str, Any]) -> MonitorState:
    return MonitorState(
        iface=str(nested(config, "can", "iface", "vcan0")),
        bitrate=int(nested(config, "can", "bitrate", 1_000_000)),
        motor_id_base=int(nested(config, "can", "motor_id_base", 0)),
        motor_count=int(nested(config, "can", "motor_count", 12)),
        bus_window_s=float(nested(config, "can", "bus_window_s", 1.0)),
        heartbeat_window_s=float(nested(config, "can", "heartbeat_window_s", 1.0)),
        node_timeout_s=float(nested(config, "can", "node_timeout_s", 0.25)),
        stuff_factor=float(nested(config, "can", "stuff_factor", 1.15)),
        feedback_position_max_rad=float(nested(config, "spg", "feedback_position_max_rad", 12.56)),
        iq_full_scale_count=float(nested(config, "spg", "iq_full_scale_count", 2048.0)),
        iq_full_scale_current_a=float(nested(config, "spg", "iq_full_scale_current_a", 33.0)),
        actuator_configs=load_actuator_configs(config),
        tx_enabled=bool(nested(config, "safety", "tx_enabled_by_default", False)),
        allow_motor_commands=bool(nested(config, "safety", "allow_motor_commands", False)),
        imu_poll_hz=float(nested(config, "imu", "default_poll_hz", 100.0)),
        mit_poll_hz=float(nested(config, "spg", "default_mit_poll_hz", 50.0)),
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


config = load_config()
state = make_state(config)
commands = CommandService(state)
app = FastAPI(title="QHRR Robot State")
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")


async def socketcan_loop() -> None:
    reconnect_delay_s = 1.0
    max_frames_per_tick = 4096
    sock = None
    while True:
        if sock is None:
            try:
                sock = open_can_socket(state.iface)
                commands.set_socket(sock)
                state.socket_status = "connected"
                state.socket_error = None
            except OSError as exc:
                commands.set_socket(None)
                state.socket_status = "disconnected"
                state.socket_error = str(exc)
                await asyncio.sleep(reconnect_delay_s)
                continue

        try:
            frames_read = 0
            while frames_read < max_frames_per_tick:
                try:
                    frame_bytes = sock.recv(CAN_FRAME_SIZE)
                except BlockingIOError:
                    break
                state.mark_rx(parse_can_frame(frame_bytes))
                frames_read += 1
        except OSError as exc:
            state.socket_status = "disconnected"
            state.socket_error = str(exc)
            commands.set_socket(None)
            try:
                sock.close()
            except OSError:
                pass
            sock = None

        await asyncio.sleep(0 if frames_read else 0.001)


async def imu_poll_loop() -> None:
    next_send_t = asyncio.get_running_loop().time()
    while True:
        if state.imu_polling:
            hz = max(0.1, min(float(state.imu_poll_hz), 1000.0))
            period_s = 1.0 / hz
            now = asyncio.get_running_loop().time()
            if now < next_send_t:
                await asyncio.sleep(next_send_t - now)

            try:
                commands.send_raw(E2BOX_REQ_ID, bytes([E2BOX_CMD_GET_ALL]))
            except CommandError as exc:
                state.socket_error = str(exc)

            now = asyncio.get_running_loop().time()
            next_send_t += period_s
            if next_send_t < now:
                next_send_t = now + period_s
        else:
            next_send_t = asyncio.get_running_loop().time()
            await asyncio.sleep(0.05)


async def mit_poll_loop() -> None:
    next_send_t = asyncio.get_running_loop().time()
    while True:
        if state.mit_poll_motor_ids:
            hz = max(0.1, min(float(state.mit_poll_hz), 1000.0))
            period_s = 1.0 / hz
            now = asyncio.get_running_loop().time()
            if now < next_send_t:
                await asyncio.sleep(next_send_t - now)

            try:
                for motor_id in sorted(state.mit_poll_motor_ids):
                    commands.motor_mit_hold(motor_id)
            except CommandError as exc:
                state.socket_error = str(exc)

            now = asyncio.get_running_loop().time()
            next_send_t += period_s
            if next_send_t < now:
                next_send_t = now + period_s
        else:
            next_send_t = asyncio.get_running_loop().time()
            await asyncio.sleep(0.05)


@app.on_event("startup")
async def startup() -> None:
    app.state.tasks = [
        asyncio.create_task(socketcan_loop()),
        asyncio.create_task(imu_poll_loop()),
        asyncio.create_task(mit_poll_loop()),
    ]


@app.on_event("shutdown")
async def shutdown() -> None:
    for task in getattr(app.state, "tasks", []):
        task.cancel()
    await asyncio.gather(*getattr(app.state, "tasks", []), return_exceptions=True)


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(FRONTEND_DIR / "index.html")


@app.get("/api/config")
async def api_config() -> dict:
    return config


@app.get("/api/state")
async def api_state() -> dict:
    return state.snapshot()


@app.post("/api/tx/lock")
async def tx_lock() -> dict:
    commands.lock_tx()
    return {"ok": True, "tx_enabled": state.tx_enabled}


@app.post("/api/tx/unlock")
async def tx_unlock() -> dict:
    commands.unlock_tx()
    return {"ok": True, "tx_enabled": state.tx_enabled}


@app.post("/api/imu/poll/start")
async def imu_poll_start() -> dict:
    state.imu_polling = True
    return {"ok": True, "imu_polling": state.imu_polling}


@app.post("/api/imu/poll/stop")
async def imu_poll_stop() -> dict:
    state.imu_polling = False
    return {"ok": True, "imu_polling": state.imu_polling}


@app.post("/api/imu/poll/hz")
async def imu_poll_hz(req: ImuHzRequest) -> dict:
    state.imu_poll_hz = max(0.1, min(float(req.hz), 1000.0))
    return {"ok": True, "imu_poll_hz": state.imu_poll_hz}


@app.post("/api/can/send")
async def can_send(req: RawSendRequest) -> dict:
    try:
        tx = commands.send_raw(parse_can_id(req.can_id), parse_hex_payload(req.data))
    except (ValueError, CommandError, OSError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True, "tx": tx}


@app.post("/api/motor/{motor_id}/enter")
async def motor_enter(motor_id: int) -> dict:
    try:
        tx = commands.motor_enter(motor_id)
    except (CommandError, OSError) as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    return {"ok": True, "tx": tx}


@app.post("/api/motor/{motor_id}/exit")
async def motor_exit(motor_id: int) -> dict:
    try:
        tx = commands.motor_exit(motor_id)
    except (CommandError, OSError) as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    return {"ok": True, "tx": tx}


@app.post("/api/motor/{motor_id}/zero")
async def motor_zero(motor_id: int, req: MotorZeroRequest) -> dict:
    try:
        tx = commands.motor_zero(motor_id, req.offset_count)
    except (CommandError, OSError) as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    return {"ok": True, "tx": tx}


@app.post("/api/motor/{motor_id}/mit-poll/start")
async def motor_mit_poll_start(motor_id: int, req: ConfirmRequest) -> dict:
    if not req.confirmed:
        raise HTTPException(status_code=400, detail="MIT polling start was not confirmed")
    if motor_id not in state.motors:
        raise HTTPException(status_code=404, detail=f"Unknown motor id {motor_id}")
    state.mit_poll_motor_ids.add(motor_id)
    return {"ok": True, "motor_id": motor_id, "mit_polling": True}


@app.post("/api/motor/{motor_id}/mit-poll/stop")
async def motor_mit_poll_stop(motor_id: int) -> dict:
    state.mit_poll_motor_ids.discard(motor_id)
    return {"ok": True, "motor_id": motor_id, "mit_polling": False}


@app.websocket("/ws/state")
async def ws_state(websocket: WebSocket) -> None:
    await websocket.accept()
    state_hz = float(nested(config, "dashboard", "state_hz", 20.0))
    delay = 1.0 / max(1.0, min(state_hz, 60.0))
    try:
        while True:
            await websocket.send_text(json.dumps(state.snapshot()))
            await asyncio.sleep(delay)
    except WebSocketDisconnect:
        return


def main() -> None:
    host = str(nested(config, "dashboard", "host", "127.0.0.1"))
    port = int(nested(config, "dashboard", "port", 8000))
    uvicorn.run("Dashboard.backend.app:app", host=host, port=port, reload=False)


if __name__ == "__main__":
    main()
