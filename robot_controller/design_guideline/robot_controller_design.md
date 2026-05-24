# RobotController 설계 인수인계 문서

## 0. 목적

이 문서는 `QHRR_control` 프로젝트에 새로 추가할 **RobotController** 계층의 설계 명세이다.

`RobotController`는 HAL 하위 모듈이 아니다. HAL은 장치 추상화 계층이고, `RobotController`는 CAN daemon, SHM, controller subprocess, bringup/shutdown sequence를 조율하는 **상위 런타임 관리자**이다.

목표는 다음과 같다.

```text
RobotController
  ├── SHM 생성/초기화/정리
  ├── CAN daemon subprocess lifecycle 관리
  ├── child controller process lifecycle 관리
  ├── 초기 robot bringup sequence 수행
  ├── SHM에 기록된 MIT target command 수집
  ├── 간단한 command freshness / NaN / range validation
  ├── CAN daemon을 통해 motor command 송신
  └── shutdown 시 안전한 종료 순서 수행
```

---

## 1. 현재까지 확정한 설계 판단

### 1.1 이름

중심 클래스 이름은 다음으로 한다.

```python
class RobotController:
    ...
```

이전 후보였던 `RobotRuntime`, `RobotOrchestrator`, `RobotSupervisor` 대신 `RobotController`를 사용한다.

### 1.2 위치

`RobotController`는 `hal/` 밑에 두지 않는다.

```text
hal/
  - CANBus
  - CANDaemon
  - CANDispatcher
  - device driver/protocol/state
  - hardware abstraction

robot_controller/
  - runtime orchestration
  - SHM ownership
  - process lifecycle
  - robot bringup/shutdown
  - command routing
```

추천 구조:

```text
QHRR_control/
├── hal/
├── tools/
├── mujoco-QHRR/
└── robot_controller/
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

### 1.3 초기 버전에서 제외할 모듈

초기 버전에서 다음 클래스는 만들지 않는다.

```text
SafetyManager
CanDaemonClient
```

이유:

- `SafetyManager`는 초기에 과분하다. 최소 validation은 `RobotController` 또는 `ShmMitCommandRouter` 내부에 둔다.
- `CanDaemonClient`도 초기에 과분하다. CAN daemon과의 통신은 `RobotController` main loop 내부에 직접 구현하고, 나중에 IPC가 깊어지면 분리한다.

초기 구조:

```text
RobotController
├── ShmManager
├── ProcessSupervisor
│   ├── can_daemon process
│   └── child controller processes
├── ShmMitCommandRouter
└── main loop 내부 CAN daemon communication
```

---

## 2. RobotController 책임

### 2.1 책임

```text
- robot_controller.yaml 로드
- SHM segment 생성
- SHM layout 초기화
- stale SHM cleanup
- CAN daemon subprocess start/stop
- CAN daemon ready 상태 확인
- child controller subprocess start/stop
- robot bringup sequence 수행
- motor enter/exit mode 수행
- SHM에 기록된 MIT target command 읽기
- active command source 선택
- command freshness 검사
- NaN/inf command 차단
- command range clamp 또는 reject
- CAN daemon으로 MIT target batch 전달
- runtime loop 주기 관리
- shutdown sequence 수행
```

### 2.2 비책임

```text
- CAN frame byte packing 세부 구현
- SPG/E2Box protocol 세부 encode/decode
- SocketCAN raw read/write 세부 구현
- policy inference
- MuJoCo simulation step 처리
- dashboard UI 처리
- actuator driver 세부 구현
```

---

## 3. SHM 소유권

SHM 생성은 `RobotController`가 관리한다.

확정한 원칙:

```text
RobotController owns SHM.
Child processes attach to SHM.
Child processes must not create or unlink SHM.
```

이유:

- SHM이 깨지면 모든 controller process가 영향을 받는다.
- SHM은 개별 node의 리소스가 아니라 시스템 공용 runtime resource이다.
- RobotController가 process lifecycle을 관리하므로, SHM lifecycle도 같은 계층에서 관리해야 한다.

시작 순서:

```text
RobotController.start()
  ├── cleanup_stale_shm()
  ├── create_shm_segments()
  ├── initialize_shm_layout()
  ├── start_can_daemon()
  ├── bringup_motors()
  ├── start_child_processes()
  └── enter_runtime_loop()
```

종료 순서:

```text
RobotController.shutdown()
  ├── stop_command_routing()
  ├── send_zero_or_damping_command()
  ├── stop_child_processes()
  ├── motor_exit_mode()
  ├── stop_can_daemon()
  ├── close_shm_handles()
  └── unlink_shm_segments()
```

---

## 4. ProcessSupervisor 책임

`ProcessSupervisor`는 `RobotController`의 멤버 모듈이다.

```python
class RobotController:
    def __init__(...):
        self.process_supervisor = ProcessSupervisor(...)
```

책임:

```text
- subprocess.Popen
- stop / terminate / kill
- is_alive check
- exit code 확인
- restart policy 적용
- stdout/stderr logging
```

비책임:

```text
- 어떤 순서로 프로세스를 켤지
- 어떤 failure가 fatal인지
- robot bringup state transition
- motor enable/disable 의미
```

이 결정은 `RobotController`가 한다.

---

## 5. CAN daemon lifecycle

CAN daemon은 process 목록에 포함한다.

```text
RobotController
  └── ProcessSupervisor
        └── can_daemon process
```

하지만 의미적으로는 특별 취급한다.

```text
ProcessSupervisor:
  - can_daemon subprocess start/stop/is_alive

RobotController:
  - 언제 can_daemon을 켤지 결정
  - can_daemon ready 여부 확인
  - can_daemon failure를 fatal로 볼지 결정
  - motor bringup/shutdown 순서를 결정
  - CAN daemon으로 MIT target batch 전달
```

`CanDaemonClient`는 만들지 않는다. 초기에는 `RobotController` main loop 내부에 CAN daemon 통신 로직을 직접 둔다.

나중에 아래 조건이 생기면 분리한다.

```text
- CAN daemon IPC API가 커짐
- request/response transaction이 많아짐
- feedback/state query가 복잡해짐
- retry, timeout, reconnect 정책이 필요해짐
```

---

## 6. 상태 머신

초기 상태 enum:

```python
class RobotControllerState(Enum):
    CREATED = auto()
    INIT_SHM = auto()
    START_CAN_DAEMON = auto()
    BRINGUP_MOTORS = auto()
    START_CHILD_PROCESSES = auto()
    RUNNING = auto()
    SHUTTING_DOWN = auto()
    STOPPED = auto()
    ERROR = auto()
```

기본 흐름:

```text
CREATED
  ↓
INIT_SHM
  ↓
START_CAN_DAEMON
  ↓
BRINGUP_MOTORS
  ↓
START_CHILD_PROCESSES
  ↓
RUNNING
  ↓
SHUTTING_DOWN
  ↓
STOPPED
```

에러 흐름:

```text
any state
  → ERROR
  → SHUTTING_DOWN
  → STOPPED
```

---

## 7. YAML config

Config는 YAML을 사용한다.

추천 경로:

```text
robot_controller/configs/robot_controller.yaml
```

실행:

```bash
python3 -m robot_controller.main --config robot_controller/configs/robot_controller.yaml
```

또는 기본 경로:

```bash
python3 -m robot_controller.main
```

---

## 8. YAML schema 초안

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

---

## 9. Config dataclass 명세

`robot_controller/config.py`에 구현한다.

필요 dataclass:

```python
@dataclass
class ProcessConfig:
    name: str
    command: list[str]
    required: bool = True
    start_order: int = 0
    stop_order: int = 0
    restart: bool = False
```

```python
@dataclass
class MitCommandShmConfig:
    name: str = "qhrr_mit_command"
    motor_count: int = 12
```

```python
@dataclass
class RobotStateShmConfig:
    name: str = "qhrr_robot_state"
    enabled: bool = False
```

```python
@dataclass
class ShmConfig:
    cleanup_stale_on_start: bool = True
    unlink_on_shutdown: bool = True
    mit_command: MitCommandShmConfig = field(default_factory=MitCommandShmConfig)
    robot_state: RobotStateShmConfig = field(default_factory=RobotStateShmConfig)
```

```python
@dataclass
class MotorConfig:
    ids: list[int] = field(default_factory=lambda: list(range(1, 13)))
    enter_on_start: bool = True
    exit_on_shutdown: bool = True
    set_zero_on_start: bool = False
```

```python
@dataclass
class MitLimitsConfig:
    position_rad: float = 12.5
    velocity_rad_s: float = 45.0
    kp: float = 500.0
    kd: float = 5.0
    torque_ff_nm: float = 33.0
```

```python
@dataclass
class CanConfig:
    daemon_process: str = "can_daemon"
    command_source: str = "shm"
    command_timeout_s: float = 0.05
    motors: MotorConfig = field(default_factory=MotorConfig)
    mit_limits: MitLimitsConfig = field(default_factory=MitLimitsConfig)
```

```python
@dataclass
class RobotControllerCoreConfig:
    name: str = "qhrr_robot_controller"
    control_hz: float = 1000.0
    startup_timeout_s: float = 5.0
    shutdown_timeout_s: float = 2.0
```

```python
@dataclass
class RobotControllerConfig:
    robot_controller: RobotControllerCoreConfig = field(default_factory=RobotControllerCoreConfig)
    shm: ShmConfig = field(default_factory=ShmConfig)
    can: CanConfig = field(default_factory=CanConfig)
    processes: list[ProcessConfig] = field(default_factory=list)
```

Function:

```python
def load_robot_controller_config(path: str | Path) -> RobotControllerConfig:
    ...
```

Use `yaml.safe_load`.

Do not require `pydantic` for the first version.

---

## 10. 데이터 타입

`robot_controller/state.py`에 둔다.

```python
@dataclass
class MitTarget:
    motor_id: int
    position_rad: float
    velocity_rad_s: float
    kp: float
    kd: float
    torque_ff_nm: float
```

```python
@dataclass
class MitCommandBatch:
    source: str
    timestamp: float
    targets: list[MitTarget]
```

```python
@dataclass
class MotorState:
    motor_id: int
    enabled: bool = False
    position_rad: float | None = None
    velocity_rad_s: float | None = None
    torque_nm: float | None = None
    temperature_c: float | None = None
    fault_code: int | None = None
```

```python
@dataclass
class RobotStateSnapshot:
    timestamp: float
    motor_states: dict[int, MotorState] = field(default_factory=dict)
    imu_state: object | None = None
    can_alive: bool = False
```

---

## 11. SHM layout 초안

초기 SHM은 MIT command batch만 만든다.

권장 방식:

```text
RobotController가 create
Controller process들이 attach
Controller process들이 latest command를 write
RobotController가 read
```

최소 layout:

```text
header:
  magic: uint32
  version: uint32
  motor_count: uint32
  sequence: uint64
  timestamp_ns: uint64
  source_id: uint32

per motor target:
  motor_id: int32
  position_rad: float64
  velocity_rad_s: float64
  kp: float64
  kd: float64
  torque_ff_nm: float64
```

주의:

- writer와 reader 간 tearing을 막기 위해 sequence counter를 둔다.
- reader는 sequence를 앞뒤로 읽어 값이 같을 때만 유효하다고 판단한다.
- timestamp 기반 stale check를 반드시 둔다.

---

## 12. ShmManager 명세

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

책임:

```text
- stale SHM cleanup
- SHM 생성
- SHM 초기값 세팅
- close
- unlink
```

---

## 13. ShmMitCommandRouter 명세

```python
class ShmMitCommandRouter:
    def __init__(self, config: MitCommandShmConfig):
        ...

    def read_latest_batch(self) -> MitCommandBatch | None:
        ...

    def is_fresh(self, batch: MitCommandBatch, timeout_s: float) -> bool:
        ...
```

초기에는 active source가 하나라고 가정해도 된다.

---

## 14. ProcessSupervisor 명세

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

Start ordering:

```text
start_all_except():
  sort by start_order ascending
```

Stop ordering policy must be explicit. Recommended:

```text
larger stop_order stops later
```

---

## 15. RobotController skeleton

```python
class RobotController:
    def __init__(self, config: RobotControllerConfig):
        self.config = config
        self.state = RobotControllerState.CREATED
        self.shm_manager = ShmManager(config.shm)
        self.process_supervisor = ProcessSupervisor(config.processes)
        self.command_router = ShmMitCommandRouter(config.shm.mit_command)
        self._running = False

    def start(self) -> None:
        self.state = RobotControllerState.INIT_SHM
        if self.config.shm.cleanup_stale_on_start:
            self.shm_manager.cleanup_stale()
        self.shm_manager.create_all()

        self.state = RobotControllerState.START_CAN_DAEMON
        self.process_supervisor.start_by_name(self.config.can.daemon_process)

        self.state = RobotControllerState.BRINGUP_MOTORS
        self._bringup_motors()

        self.state = RobotControllerState.START_CHILD_PROCESSES
        self.process_supervisor.start_all_except(self.config.can.daemon_process)

        self.state = RobotControllerState.RUNNING
        self._running = True

    def run(self) -> None:
        period_s = 1.0 / self.config.robot_controller.control_hz
        while self._running:
            t0 = time.monotonic()
            self.tick()
            sleep_s = period_s - (time.monotonic() - t0)
            if sleep_s > 0:
                time.sleep(sleep_s)

    def tick(self) -> None:
        batch = self.command_router.read_latest_batch()

        if batch is None:
            self._send_zero_or_damping_command()
            return

        if not self.command_router.is_fresh(batch, self.config.can.command_timeout_s):
            self._send_zero_or_damping_command()
            return

        batch = self._sanitize_mit_batch(batch)
        self._send_mit_batch_to_can_daemon(batch)

    def shutdown(self) -> None:
        self.state = RobotControllerState.SHUTTING_DOWN
        self._running = False
        self._shutdown_motors()
        self.process_supervisor.stop_all(self.config.robot_controller.shutdown_timeout_s)
        self.shm_manager.close_all()
        if self.config.shm.unlink_on_shutdown:
            self.shm_manager.unlink_all()
        self.state = RobotControllerState.STOPPED
```

---

## 16. CAN daemon 통신 임시 구현

`CanDaemonClient`는 만들지 않는다.

초기에는 `RobotController` 내부 private method로 둔다.

```python
def _send_mit_batch_to_can_daemon(self, batch: MitCommandBatch) -> None:
    # TODO:
    # 현재 CAN daemon IPC 구조에 맞춰 구현한다.
    # 1차 skeleton에서는 NotImplementedError 또는 logging만 해도 된다.
    ...
```

```python
def _bringup_motors(self) -> None:
    # TODO:
    # CAN daemon에 clear fault / enter mode command를 보낸다.
    ...
```

```python
def _shutdown_motors(self) -> None:
    # TODO:
    # zero/damping command 후 exit mode.
    ...
```

---

## 17. Acceptance Criteria

Codex agent는 다음을 만족해야 한다.

### 17.1 파일 생성

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

### 17.2 실행

```bash
python3 -m robot_controller.main --config robot_controller/configs/robot_controller.yaml
```

또는 기본 경로로:

```bash
python3 -m robot_controller.main
```

### 17.3 구현 범위

1차 skeleton에서 반드시 구현:

```text
- YAML config 로드
- RobotController state machine
- ShmManager skeleton
- ProcessSupervisor 실제 subprocess start/stop
- ShmMitCommandRouter skeleton
- main loop
- graceful shutdown handling
```

아직 구현하지 않아도 되는 것:

```text
- 실제 CAN daemon IPC 송신
- 실제 MIT command SHM binary layout 완성
- multiple command source arbitration
- SafetyManager
- CanDaemonClient
```

단, TODO 주석으로 분명히 남긴다.

---

## 18. Codex agent에게 요청할 작업 요약

작업명:

```text
Create robot_controller package skeleton
```

핵심 요구:

```text
- Do not put RobotController under hal/
- Use YAML config
- RobotController owns SHM lifecycle
- ProcessSupervisor is a member module
- CAN daemon is managed as a supervised process
- Do not create SafetyManager
- Do not create CanDaemonClient
- Put simple command validation and CAN daemon communication TODOs in RobotController
- Keep code readable and minimal
```
