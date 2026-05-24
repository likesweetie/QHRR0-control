# Logging

근거 파일: `robot_controller/utils/process_supervisor.py`, `robot_controller/process/*/main.py`, `robot_controller/runtime_io.py`, `legacy/mujoco-QHRR/app/shm_logger/main.cpp`.

## Log Directory 구조

`ProcessSupervisor` 생성 시 현재 working directory 기준으로 다음 폴더를 만든다.

```text
log/YYYYMMDD_HHMMSS/
```

각 supervised process는 다음 파일에 stdout/stderr가 저장된다.

| File | 의미 |
| --- | --- |
| `log/<timestamp>/can_daemon.log` | CAN daemon stdout/stderr, Python logging |
| `log/<timestamp>/aux_reader.log` | joystick reader stdout/stderr |
| `log/<timestamp>/task_controller.log` | policy load, SHM wait, loop overrun, traceback |
| `log/<timestamp>/dashboard.log` | FastAPI/Uvicorn/dashboard backend logs |

`new_terminal: true` process는 terminal command 안에서 `tee -a <log_file>`로 출력이 저장된다.

## Supervisor Log / PID

| 항목 | 경로/형식 |
| --- | --- |
| log root | `Path.cwd()/log/<timestamp>` |
| pid root | `/tmp/qhrr_robot_controller_processes` |
| pidfile | `/tmp/qhrr_robot_controller_processes/<process>.pid` |
| terminal first line | `[process_supervisor] log file: <path>` |

## Runtime Log Message 예시

| Component | Message | 의미 |
| --- | --- | --- |
| `RuntimeIO` | `Sending one damping command because ...` | reason별 damping fallback 전송 |
| `RuntimeIO` | `Registered CAN RX callbacks: ...` | CAN callback 등록된 ID 목록 |
| `RuntimeIO` | `CAN error during actuator shutdown: ...` | shutdown 중 CAN error |
| `CANProcessClient` | `CAN daemon RX socket failed` | RX Unix socket closed/error |
| `CANSubprocessDaemon` | `Replacing existing CAN daemon IPC socket: ...` | stale/existing socket unlink |
| `ProcessSupervisor` | `Killing process ... after stop timeout ...` | SIGTERM timeout 후 SIGKILL |
| `task_controller` | `[task_controller] loop overrun: ...` | policy loop period 초과 |

## CSV Field Table

현재 `ProcessSupervisor` process list에는 `shm_logger`가 없다. 다만 legacy C++ logger source에서 CSV 형식이 확인된다.

| CSV field | 의미 |
| --- | --- |
| `wall_time_ns` | system clock nanoseconds |
| `sim_time` | MuJoCo shared memory sim time |
| `state_seq` | simulation state sequence |
| `applied_command_seq` | applied command sequence |
| `nq`, `nv`, `nu` | qpos/qvel/control dimension |
| `q_0...` | qpos values |
| `qd_0...` | qvel values |
| `ctrl_applied_0...` | applied control values |

CSV logger period는 legacy source에서 20 ms, 즉 50 Hz로 확인된다.

UNKNOWN: 현재 Python process manager에서 `shm_logger`를 실행하는 config는 확인되지 않았다.

## Timestamp 기준

| 위치 | 기준 |
| --- | --- |
| Robot State SHM header `timestamp_ns` | `time.time_ns()` |
| Robot State JSON `timestamp_unix` | `time.time()` |
| Robot State JSON `timestamp_monotonic` | `time.monotonic()` |
| Driver feedback `last_feedback_t`, IMU `last_*_t` | `time.monotonic()` |
| CSV `wall_time_ns` | `std::chrono::system_clock` |
| CSV `sim_time` | MuJoCo SHM field |

## Decimation / Rate

| Log/data | Rate |
| --- | --- |
| `control_state` SHM | 500 Hz |
| `dashboard_state` SHM | 10 Hz |
| dashboard websocket | 24 Hz |
| legacy CSV logger | 50 Hz |
| process stdout logs | event-driven |

## Debugging Guide

```bash
ls -td log/* | head
tail -f log/<timestamp>/can_daemon.log
tail -f log/<timestamp>/task_controller.log
tail -f log/<timestamp>/dashboard.log
ls /tmp/qhrr_robot_controller_processes
cat /tmp/qhrr_robot_controller_processes/can_daemon.pid
```

CAN과 함께 볼 때:

```bash
candump -td vcan0
tail -f log/<timestamp>/can_daemon.log
```

## 검증 필요 항목

| 항목 | 질문 |
| --- | --- |
| log retention | TODO(owner): `log/` 자동 정리/보존 기간 정책 |
| structured logging | TODO(owner): Python logging format 통일 여부 |
| dashboard access log | TODO(owner): Uvicorn access log 필요 여부 |
| current CSV logger | TODO(owner): legacy `shm_logger`를 process로 유지할지 제거할지 결정 |
