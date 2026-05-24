# Codex Agent Prompt: RobotController 패키지 skeleton 생성

다음 작업을 수행해 주세요.

## 목표

`QHRR_control` 프로젝트 루트에 새로운 Python 패키지 `robot_controller/`를 추가합니다.

이 패키지는 HAL 하위 모듈이 아닙니다. `RobotController`는 CAN daemon, SHM, child controller subprocess, bringup/shutdown sequence를 조율하는 상위 runtime controller입니다.

## 생성할 파일 구조

```text
robot_controller/
├── __init__.py
├── main.py
├── robot_controller.py
├── config.py
├── shm_manager.py
├── process_supervisor.py
├── shm_command_router.py
├── state.py
└── configs/
    └── robot_controller.yaml
```

## 설계 원칙

- 중심 클래스 이름은 `RobotController`입니다.
- `RobotController`는 `hal/` 밑에 두지 않습니다.
- SHM 생성/초기화/정리는 `RobotController`가 소유합니다.
- Child process들은 SHM에 attach만 해야 하며, SHM을 생성하거나 unlink하지 않습니다.
- `ProcessSupervisor`는 `RobotController`의 멤버 모듈입니다.
- CAN daemon subprocess lifecycle은 `ProcessSupervisor`가 기계적으로 관리합니다.
- CAN daemon bringup 순서와 failure 의미 판단은 `RobotController`가 담당합니다.
- `SafetyManager`는 만들지 마세요.
- `CanDaemonClient`도 만들지 마세요.
- CAN daemon과의 실제 통신은 초기에는 `RobotController` private method 안에 TODO로 남기세요.
- Config는 YAML을 사용합니다.
- 1차 구현은 skeleton이 목적입니다. 실제 CAN daemon IPC와 완전한 SHM binary layout은 TODO로 남겨도 됩니다.

## 구현할 기능

### 1. config.py

`yaml.safe_load` 기반 config loader를 구현하세요.

Dataclass:

- `ProcessConfig`
- `MitCommandShmConfig`
- `RobotStateShmConfig`
- `ShmConfig`
- `MotorConfig`
- `MitLimitsConfig`
- `CanConfig`
- `RobotControllerCoreConfig`
- `RobotControllerConfig`

Function:

```python
def load_robot_controller_config(path: str | Path) -> RobotControllerConfig:
    ...
```

pydantic은 사용하지 마세요.

### 2. state.py

다음 dataclass와 enum을 구현하세요.

- `RobotControllerState`
- `MitTarget`
- `MitCommandBatch`
- `MotorState`
- `RobotStateSnapshot`

`RobotControllerState`는 최소한 다음을 포함합니다.

```text
CREATED
INIT_SHM
START_CAN_DAEMON
BRINGUP_MOTORS
START_CHILD_PROCESSES
RUNNING
SHUTTING_DOWN
STOPPED
ERROR
```

### 3. process_supervisor.py

`subprocess.Popen` 기반으로 구현하세요.

Class:

```python
class ProcessSupervisor:
    def __init__(self, process_configs: list[ProcessConfig]):
        ...

    def start_by_name(self, name: str) -> None:
        ...

    def start_all_except(self, excluded_name: str) -> None:
        ...

    def stop_by_name(self, name: str, timeout_s: float = 2.0) -> None:
        ...

    def stop_all(self, timeout_s: float = 2.0) -> None:
        ...

    def is_alive(self, name: str) -> bool:
        ...

    def status(self) -> dict[str, object]:
        ...
```

stdout/stderr는 일단 부모 프로세스에 그대로 연결해도 됩니다.

### 4. shm_manager.py

Class:

```python
class ShmManager:
    def __init__(self, config: ShmConfig):
        ...

    def cleanup_stale(self) -> None:
        ...

    def create_all(self) -> None:
        ...

    def close_all(self) -> None:
        ...

    def unlink_all(self) -> None:
        ...
```

초기에는 `multiprocessing.shared_memory`를 사용하세요.

MIT command SHM은 최소 크기 placeholder라도 생성하세요.  
실제 binary layout은 TODO로 남겨도 됩니다.

### 5. shm_command_router.py

Class:

```python
class ShmMitCommandRouter:
    def __init__(self, config: MitCommandShmConfig):
        ...

    def read_latest_batch(self) -> MitCommandBatch | None:
        ...

    def is_fresh(self, batch: MitCommandBatch, timeout_s: float) -> bool:
        ...
```

초기 `read_latest_batch()`는 TODO 또는 dummy None 반환이어도 됩니다.  
단, 구조와 주석은 명확하게 작성하세요.

### 6. robot_controller.py

Class:

```python
class RobotController:
    def __init__(self, config: RobotControllerConfig):
        ...

    def start(self) -> None:
        ...

    def run(self) -> None:
        ...

    def tick(self) -> None:
        ...

    def shutdown(self) -> None:
        ...
```

내부 멤버:

```python
self.shm_manager
self.process_supervisor
self.command_router
```

Main sequence:

```text
start():
  INIT_SHM
  cleanup stale SHM if configured
  create SHM
  START_CAN_DAEMON
  start can_daemon process
  BRINGUP_MOTORS
  call _bringup_motors()
  START_CHILD_PROCESSES
  start all processes except can_daemon
  RUNNING

run():
  loop at configured control_hz
  call tick()

tick():
  read latest MIT batch from SHM router
  if missing or stale, call _send_zero_or_damping_command()
  else sanitize and call _send_mit_batch_to_can_daemon()

shutdown():
  stop runtime
  shutdown motors
  stop processes
  close/unlink SHM
```

Private methods:

```python
def _bringup_motors(self) -> None:
    # TODO

def _shutdown_motors(self) -> None:
    # TODO

def _send_zero_or_damping_command(self) -> None:
    # TODO

def _sanitize_mit_batch(self, batch: MitCommandBatch) -> MitCommandBatch:
    # clamp or reject NaN/inf and command ranges

def _send_mit_batch_to_can_daemon(self, batch: MitCommandBatch) -> None:
    # TODO
```

### 7. main.py

CLI entrypoint를 구현하세요.

```bash
python3 -m robot_controller.main --config robot_controller/configs/robot_controller.yaml
```

`--config`를 생략하면 기본 경로 `robot_controller/configs/robot_controller.yaml`을 사용하세요.

SIGINT/SIGTERM에서 graceful shutdown 하도록 구현하세요.

### 8. configs/robot_controller.yaml

다음 예시 config를 추가하세요.

```yaml
robot_controller:
  name: qhrr_robot_controller
  control_hz: 1000
  startup_timeout_s: 5.0
  shutdown_timeout_s: 2.0

shm:
  cleanup_stale_on_start: true
  unlink_on_shutdown: true

  mit_command:
    name: qhrr_mit_command
    motor_count: 12

  robot_state:
    name: qhrr_robot_state
    enabled: false

can:
  daemon_process: can_daemon
  command_source: shm
  command_timeout_s: 0.05

  motors:
    ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    enter_on_start: true
    exit_on_shutdown: true
    set_zero_on_start: false

  mit_limits:
    position_rad: 12.5
    velocity_rad_s: 45.0
    kp: 500.0
    kd: 5.0
    torque_ff_nm: 33.0

processes:
  - name: can_daemon
    command: ["python3", "-m", "hal.daemon_launcher"]
    required: true
    start_order: 0
    stop_order: 100
    restart: false

  - name: task_controller
    command: ["./mujoco-QHRR/build/task_controller"]
    required: true
    start_order: 10
    stop_order: 10
    restart: false
```

## Acceptance Criteria

- `python3 -m robot_controller.main`로 실행 가능해야 합니다.
- YAML config를 정상 로드해야 합니다.
- SHM placeholder를 생성하고 종료 시 close/unlink 해야 합니다.
- `can_daemon` process를 `ProcessSupervisor`로 시작하려 시도해야 합니다.
- SIGINT에서 graceful shutdown 해야 합니다.
- `SafetyManager`와 `CanDaemonClient` 클래스는 만들지 않아야 합니다.
- 실제 CAN daemon IPC와 실제 MIT SHM binary read/write는 TODO로 남겨도 됩니다.
