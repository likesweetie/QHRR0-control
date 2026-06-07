# YAML Config Refactor Implementation Plan

## 0. 현재 상태 요약

현재 `config/app_config/`에는 다음 YAML이 존재합니다.

```text
config/app_config/config_paths.yaml
config/app_config/robot_controller.yaml
config/app_config/robot_platform.yaml
config/app_config/can_device_config.yaml
config/app_config/processes.yaml
config/app_config/dashboard.yaml
config/app_config/mujoco.yaml
```

현재 YAML 구조는 새 설계 방향을 상당히 반영하고 있습니다.

```text
- platform.yaml은 없음
- robot_platform.yaml 존재
- can_device_config.yaml 존재
- config_paths.yaml 존재
- dashboard.yaml에서 transmit_ids 삭제됨
- dashboard.state_update_rate 적용됨
- processes.yaml에서 env_vars 적용됨
- mujoco.yaml은 imu_sensors만 남음
- robot_controller.yaml에서 require_manual_arm, cleanup_stale_on_start, unlink_on_shutdown, can.motors 제거됨
```

하지만 Python 코드는 아직 대부분 이전 schema를 기대합니다. 따라서 현재 상태에서 `load_robot_controller_config("config/app_config/robot_controller.yaml")`를 실행하면 바로 실패합니다.

```text
Missing required config key: <root>.platform_config
```

즉 지금 필요한 작업은 YAML을 다시 고치는 것이 아니라, **새 YAML schema에 맞게 loader, dataclass, subprocess, dashboard, tests를 이관하는 작업**입니다.

---

## 1. Step 1 — Config path registry를 실제 loader에 연결

### 현재 문제

아직 `config_paths.yaml`를 읽는 코드가 없습니다. 기존 로더들은 각 YAML 안의 sibling path를 직접 기대합니다.

대표적인 기존 기대값:

```text
robot_controller.yaml:
  platform_config
  processes_config

dashboard.yaml:
  platform_config
  robot_controller_config

mujoco.yaml:
  platform_config
```

하지만 새 YAML에는 이런 key가 없습니다.

### 수정 대상

새 파일 추가를 권장합니다.

```text
robot_controller/config/paths.py
```

### 구현 지침

```python
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from robot_controller.config.loader import ConfigError, load_yaml_mapping


DEFAULT_CONFIG_PATHS = Path("config/app_config/config_paths.yaml")


@dataclass(frozen=True)
class ConfigPathRegistry:
    configs: Mapping[str, Path]
    policy: Mapping[str, Path]

    def config(self, key: str) -> Path:
        try:
            return self.configs[key]
        except KeyError as exc:
            raise ConfigError(f"unknown config path key: configs.{key}") from exc

    def policy_path(self, key: str) -> Path:
        try:
            return self.policy[key]
        except KeyError as exc:
            raise ConfigError(f"unknown policy path key: policy.{key}") from exc


def load_config_paths(path: str | Path = DEFAULT_CONFIG_PATHS) -> ConfigPathRegistry:
    path = Path(path)
    raw = load_yaml_mapping(path)
    base = path.resolve().parents[2]  # project root if path == config/app_config/config_paths.yaml

    configs = {
        str(key): (base / str(value)).resolve()
        for key, value in raw.get("configs", {}).items()
    }
    policy = {
        str(key): (base / str(value)).resolve()
        for key, value in raw.get("policy", {}).items()
    }
    return ConfigPathRegistry(configs=configs, policy=policy)
```

### 주의점

`config_paths.yaml`는 프로젝트 고정 entry point로 두려는 의도이므로, 각 YAML에 다시 `robot_platform_config`, `can_device_config`, `processes_config` 같은 path key를 넣지 마십시오.

---

## 2. Step 2 — `robot_platform.yaml` loader를 새 schema로 교체

### 현재 문제

`robot_controller/platform/config.py`는 아직 이전 `platform.yaml` schema를 기대합니다.

현재 기대하는 key:

```text
robots
can.interface
can.bitrate
can.daemon_socket
shm.*
imu.*
spg_mit.*
actuators[].enabled
```

하지만 새 `robot_platform.yaml`에는 위 key들이 없습니다.

### 수정 대상

```text
robot_controller/platform/config.py
robot_controller/platform/__init__.py
robot_controller/core/platform_config.py
robot_controller/core/config.py
```

### 권장 dataclass

```python
@dataclass(frozen=True)
class RobotPlatformRobotConfig:
    name: str


@dataclass(frozen=True)
class RobotPlatformAssetConfig:
    mujoco_model_path: str


@dataclass(frozen=True)
class RobotPlatformCanConfig:
    allowed_interfaces: tuple[str, ...]


@dataclass(frozen=True)
class RobotPlatformActuatorConfig:
    name: str
    driver: str
    can_id: int
    mujoco_joint: str
    mujoco_actuator: str
    sign: float
    offset_rad: float


@dataclass(frozen=True)
class RobotPlatformConfig:
    path: Path
    robot: RobotPlatformRobotConfig
    assets: RobotPlatformAssetConfig
    can: RobotPlatformCanConfig
    actuators: tuple[RobotPlatformActuatorConfig, ...]
```

### 제거할 개념

```text
PlatformRobotAssetConfig.policy_config_dir
PlatformRobotAssetConfig.pd_config_path
PlatformCanConfig.interface
PlatformCanConfig.bitrate
PlatformCanConfig.daemon_socket
PlatformShmConfig
PlatformImuConfig
PlatformSpgMitConfig
PlatformActuatorConfig.enabled
enabled_actuators property
robots mapping
```

### 검증 규칙

```text
- robot.name은 non-empty
- assets.mujoco_model_path는 non-empty
- can.allowed_interfaces는 non-empty
- actuator name 중복 금지
- actuator can_id 중복 금지
- actuator driver는 non-empty
- sign은 0이면 경고 또는 에러 권장
```

### 호환성 처리

기존 class 이름 `PlatformConfig`를 유지하면 수정 범위는 줄지만 의미가 흐립니다. 가능하면 새 이름을 권장합니다.

```text
PlatformConfig -> RobotPlatformConfig
load_platform_config -> load_robot_platform_config
PlatformConfigError -> RobotPlatformConfigError
```

단, 단계적 이관이 필요하면 `robot_controller/platform/__init__.py`에서 compatibility alias를 잠시 둘 수 있습니다.

---

## 3. Step 3 — `can_device_config.yaml` loader 추가

### 현재 YAML

```yaml
imu:
  type: "e2box"
  request_id: "0x221"
  quat_id: "0x2A1"
  gyro_id: "0x321"
  cmd_get_quat: "0x01"
  cmd_get_gyro: "0x02"
  cmd_get_all: "0x03"
  quat_scale: 10000.0
  gyro_scale: 100.0
  normalize_quat: true

drivers:
  spg_mit:
    p_max_rad: 12.5
    v_max_rad_s: 45.0
    kp_max: 500.0
    kd_max: 5.0
    tau_max_nm: 33.0
    feedback_position_max_rad: 12.56
    iq_full_scale_count: 2048.0
    iq_full_scale_current_a: 33.0
    set_zero_hold_s: 0.020
```

### 현재 문제

현재 IMU protocol과 SPG MIT range/current scaling은 `robot_controller/platform/config.py`가 읽습니다. 새 설계에서는 이 정보가 `can_device_config.yaml`로 이동했으므로 별도 loader가 필요합니다.

### 수정 대상

새 파일 추가를 권장합니다.

```text
robot_controller/config/can_device.py
```

또는 패키지 분리를 강하게 하려면:

```text
robot_controller/can_device/config.py
```

### 권장 dataclass

```python
@dataclass(frozen=True)
class CanDeviceImuConfig:
    type: str
    request_id: int
    quat_id: int
    gyro_id: int
    cmd_get_quat: int
    cmd_get_gyro: int
    cmd_get_all: int
    quat_scale: float
    gyro_scale: float
    normalize_quat: bool


@dataclass(frozen=True)
class SpgMitDriverConfig:
    p_max_rad: float
    v_max_rad_s: float
    kp_max: float
    kd_max: float
    tau_max_nm: float
    feedback_position_max_rad: float
    iq_full_scale_count: float
    iq_full_scale_current_a: float
    set_zero_hold_s: float


@dataclass(frozen=True)
class CanDeviceConfig:
    imu: CanDeviceImuConfig
    drivers: dict[str, SpgMitDriverConfig]
```

### 검증 규칙

```text
- imu.type == "e2box"만 현재 지원
- CAN ID/command ID는 parse_int로 hex 문자열 허용
- quat_scale, gyro_scale > 0
- driver 이름 중복은 YAML mapping 특성상 자연히 불가능하지만, key non-empty 검증
- spg_mit range들은 모두 양수
- set_zero_hold_s >= 0
- iq_full_scale_count > 0
```

### 연결 검증

`robot_platform.actuators[*].driver`가 `can_device.drivers`에 존재하는지 검사해야 합니다.

```text
for actuator in robot_platform.actuators:
    actuator.driver in can_device.drivers
```

---

## 4. Step 4 — `RobotControllerConfig` composition 재설계

### 현재 문제

`robot_controller/config/app.py`는 아직 다음을 기대합니다.

```text
robot_controller.yaml의 platform_config
robot_controller.yaml의 processes_config
PlatformConfig 하나가 CAN/SHM/IMU/SPG/actuator 정보를 모두 소유
```

현재 새 YAML에서는 경로가 `config_paths.yaml`에 있고, 정보도 다음처럼 분리되어 있습니다.

```text
robot_platform.yaml       -> robot object / actuator wiring
can_device_config.yaml    -> CAN device protocol / driver profile
robot_controller.yaml     -> runtime / SHM / selected CAN transport / controller policy
processes.yaml            -> subprocess launch
```

### 수정 대상

```text
robot_controller/config/app.py
robot_controller/config/__init__.py
robot_controller/main.py
robot_controller/subprocesses/*/main.py
```

### 권장 `RobotControllerConfig`

```python
@dataclass
class RobotControllerConfig:
    robot_platform: RobotPlatformConfig
    can_device: CanDeviceConfig
    runtime: RuntimeModeConfig
    hardware: HardwareSafetyConfig
    safety: SafetyPolicyConfig
    state_machine: StateMachineConfig
    robot_controller: RobotControllerCoreConfig
    shm: ShmConfig
    can: CanConfig
    processes: list[ProcessConfig]
```

`platform`이라는 필드는 제거하거나 compatibility 단계에서만 alias로 남기는 것을 권장합니다.

### 권장 loader signature

```python
def load_robot_controller_config(
    path: str | Path | None = None,
    *,
    config_paths: ConfigPathRegistry | None = None,
) -> RobotControllerConfig:
    ...
```

동작 원칙:

```text
- path가 None이면 config_paths.config("robot_controller") 사용
- robot_platform은 config_paths.config("robot_platform")에서 로드
- can_device는 config_paths.config("can_device")에서 로드
- processes는 config_paths.config("processes")에서 로드
- robot_controller.yaml 내부에서는 sibling path를 읽지 않음
```

---

## 5. Step 5 — `HardwareSafetyConfig`에서 삭제된 선택지를 코드 불변조건으로 이동

### 현재 YAML

```yaml
hardware:
  allow_real_can: false
```

### 현재 문제

`robot_controller/config/app.py`는 아직 다음 필드를 요구합니다.

```text
hardware.require_manual_arm
hardware.require_estop
hardware.allow_enable_on_start
hardware.allowed_can_interfaces
```

하지만 새 설계에서는:

```text
require_manual_arm        -> 삭제, controller/runtime 불변조건 또는 CLI gate
require_estop             -> 삭제, CLI/runtime 불변조건
allow_enable_on_start     -> 삭제, controller startup에서 절대 미지원
allowed_can_interfaces    -> robot_platform.yaml의 can.allowed_interfaces로 이동
```

### 수정 대상

```text
robot_controller/config/app.py
robot_controller/config/validation.py
robot_controller/main.py
```

### 권장 dataclass

```python
@dataclass
class HardwareSafetyConfig:
    allow_real_can: bool
```

### runtime validation 변경

기존 삭제:

```text
config.hardware.require_manual_arm
config.hardware.require_estop
config.hardware.allow_enable_on_start
config.hardware.allowed_can_interfaces
config.can.motors.enter_on_start
```

신규 정책:

```text
- hardware mode는 --hardware 요구
- hardware mode는 --i-understand-this-can-enable-motors 요구
- hardware mode에서 vcan* 거부
- hardware mode에서 can* 요구
- hardware mode에서 config.hardware.allow_real_can == true 요구
- selected can.interface는 robot_platform.can.allowed_interfaces에 포함되어야 함
- startup enable/zero-set은 config가 아니라 controller 코드에서 아예 제공하지 않음
```

`--estop-ok`를 계속 유지할지 결정해야 합니다. YAML에서는 삭제되었으므로, 유지한다면 config가 아니라 CLI hard gate로 처리해야 합니다.

```python
if config.runtime.mode == "hardware" and not options.estop_ok:
    raise ConfigError("hardware mode requires --estop-ok")
```

또는 `--estop-ok` 자체를 제거합니다. 실제 하드웨어 안전 정책상 저는 CLI hard gate로 유지하는 쪽을 권장합니다.

---

## 6. Step 6 — SHM config를 robot_controller.yaml 소유로 이관

### 현재 YAML

```yaml
shm:
  mit_command:
    name: "qhrr_mit_command"

  aux_command:
    name: "qhrr_aux_command"
    size_bytes: 4096
    publish_hz: 100.0

  operator_command:
    name: "qhrr_operator_command"
    size_bytes: 4096

  control_state:
    name: "qhrr_control_state"
    size_bytes: 16384
    publish_hz: 500.0

  dashboard_state:
    name: "qhrr_dashboard_state"
    size_bytes: 65536
    publish_hz: 10.0
```

### 현재 문제

`robot_controller/shm/config.py`는 아직 다음을 기대합니다.

```text
shm.cleanup_stale_on_start
shm.unlink_on_shutdown
platform.shm.*
platform.enabled_actuators
```

새 설계에서는:

```text
cleanup_stale_on_start  -> 삭제, controller 기본 동작
unlink_on_shutdown      -> 삭제, controller 기본 동작
SHM names               -> robot_controller.yaml 소유
target_count            -> len(robot_platform.actuators) 또는 len(can.motors.can_ids)에서 파생
```

### 수정 대상

```text
robot_controller/shm/config.py
robot_controller/controller.py
robot_controller/config/app.py
```

### 권장 dataclass 변경

```python
@dataclass
class ShmConfig:
    mit_command: MitCommandShmConfig
    aux_command: RobotStateShmConfig
    operator_command: OperatorCommandShmConfig
    control_state: RobotStateShmConfig
    dashboard_state: RobotStateShmConfig
```

`cleanup_stale_on_start`, `unlink_on_shutdown` 제거.

### parser 변경

```python
def parse_shm_config(raw: dict[str, Any], target_count: int) -> ShmConfig:
    mit_command_raw = require_mapping(raw, "mit_command", "shm")
    ...
    mit_command=MitCommandShmConfig(
        name=str(require_key(mit_command_raw, "name", "shm.mit_command")),
        target_count=target_count,
    )
```

### controller 변경

기존:

```python
if self.config.shm.cleanup_stale_on_start:
    self.shm_manager.cleanup_stale()
...
if self.config.shm.unlink_on_shutdown:
    self.shm_manager.unlink_all()
```

신규:

```python
# 기본 동작으로 항상 수행하거나, 정책상 한쪽으로 고정합니다.
self.shm_manager.cleanup_stale()
...
self.shm_manager.unlink_all()
```

사용자 의도상 둘 다 config 선택지를 제거하고 기본 기능으로 가져가는 것이므로, 지금은 항상 수행하는 쪽으로 정리하면 됩니다.

---

## 7. Step 7 — CAN config를 robot_controller + robot_platform + can_device 조합으로 재구성

### 현재 YAML

`robot_controller.yaml`:

```yaml
can:
  interface: "vcan0"
  bitrate: 1000000
  daemon_socket: "/tmp/qhrr_can_daemon.sock"
  command_timeout_s: 0.05
  bringup_delay_s: 0.1
  daemon: ...
  imu: ...
```

`robot_platform.yaml`:

```yaml
can:
  allowed_interfaces:
    - "vcan0"
    - "can0"
actuators:
  - driver: "spg_mit"
    can_id: "0x141"
```

`can_device_config.yaml`:

```yaml
imu: ...
drivers:
  spg_mit: ...
```

### 현재 문제

`robot_controller/config/can.py`는 아직 다음을 기대합니다.

```text
platform.can.interface
platform.can.daemon_socket
platform.enabled_actuators
platform.spg_mit
can.motors.enter_on_start
can.motors.exit_on_shutdown
can.motors.set_zero_on_start
```

새 YAML에는 `can.motors`가 없습니다.

### 수정 대상

```text
robot_controller/config/can.py
robot_controller/config/app.py
robot_controller/config/validation.py
robot_controller/controller.py
```

### 권장 dataclass 변경

```python
@dataclass
class MotorConfig:
    can_ids: list[int]


@dataclass
class CANDaemonConfig:
    rx_timeout_s: float
    tx_timeout_s: float
    join_timeout_s: float
    max_tx_queue_size: int
    send_block: bool
    send_timeout_s: float | None
    ipc_socket_path: str
    connect_timeout_s: float


@dataclass
class CanConfig:
    interface: str
    bitrate: int
    command_timeout_s: float
    bringup_delay_s: float
    daemon: CANDaemonConfig
    motors: MotorConfig
    imu: ImuConfig
    mit_protocol_range: MitProtocolRangeConfig
```

### parser 변경

```python
def parse_can_config(
    raw: dict[str, Any],
    robot_platform: RobotPlatformConfig,
    can_device: CanDeviceConfig,
) -> CanConfig:
    interface = str(require_key(raw, "interface", "can"))
    if interface not in robot_platform.can.allowed_interfaces:
        raise ConfigError("can.interface is not listed in robot_platform.can.allowed_interfaces")

    motor_can_ids = [actuator.can_id for actuator in robot_platform.actuators]
    spg = can_device.drivers["spg_mit"]
    ...
```

### 기본 불변조건

다음 개념은 config에서 삭제되었으므로 다시 dataclass에 넣지 마십시오.

```text
enter_on_start
exit_on_shutdown
set_zero_on_start
```

shutdown disable은 controller 기본 동작으로 유지합니다.

---

## 8. Step 8 — Robot spec 생성 경로 수정

### 현재 문제

현재 controller는 다음 경로에 의존합니다.

```python
self.robot_spec = robot_spec_from_platform(config.platform)
```

그리고 qhrr0_hw 쪽은 다음을 기대합니다.

```text
platform.actuators[*].enabled
platform.imu.*
```

하지만 새 구조에서:

```text
actuator.enabled 삭제
IMU protocol은 can_device_config.yaml로 이동
```

### 수정 대상

```text
qhrr0_hw/robot_spec.py
qhrr0_hw/actuators/actuator_specs.py
qhrr0_hw/imu/imu_specs.py
robot_controller/controller.py
```

### 권장 방향

`robot_spec`은 robot_platform과 can_device를 모두 받아 구성합니다.

```python
def robot_spec_from_config(
    robot_platform: RobotPlatformConfig,
    can_device: CanDeviceConfig,
) -> QHRR0RobotSpec:
    ...
```

또는 분리합니다.

```text
actuator_specs_from_robot_platform(robot_platform)
imu_spec_from_can_device(can_device)
```

### actuator spec 변경

```python
@dataclass(frozen=True)
class QHRR0ActuatorSpec:
    name: str
    driver: str
    can_id: int
    joint_index: int
    joint_name: str
    sign: float
    offset_rad: float
```

`enabled` 제거.

### controller actuator driver 생성 변경

현재:

```python
iq_full_scale_count = self.config.platform.spg_mit.iq_full_scale_count
```

신규:

```python
spg = self.config.can_device.drivers["spg_mit"]
iq_full_scale_count = spg.iq_full_scale_count
```

각 actuator의 `driver`를 보고 driver profile을 선택해야 합니다.

---

## 9. Step 9 — processes.yaml의 `env_vars` 반영

### 현재 YAML

```yaml
env_vars: {}
```

### 현재 문제

`robot_controller/supervisor/config.py`는 아직 `env`를 요구합니다.

```python
env_raw = require_mapping(item, "env", f"processes[{index}]")
```

`ProcessConfig`도 `env` 필드를 갖고, `ProcessSupervisor`는 `config.env`를 읽습니다.

### 수정 대상

```text
robot_controller/supervisor/config.py
robot_controller/supervisor/process_supervisor.py
```

### 변경 지침

```python
@dataclass
class ProcessConfig:
    ...
    env_vars: dict[str, str]
```

parser:

```python
env_vars_raw = require_mapping(item, "env_vars", f"processes[{index}]")
...
env_vars={str(key): str(value) for key, value in env_vars_raw.items()}
```

supervisor:

```python
env.update(config.env_vars)
```

### strict validation

옛 키가 들어오면 에러를 내십시오.

```python
if "env" in item:
    raise ConfigError("processes[*].env was renamed to processes[*].env_vars")
```

---

## 10. Step 10 — subprocess CLI를 `--config-key` 방식으로 이관

### 현재 YAML

```yaml
command:
  - "python3"
  - "-m"
  - "robot_controller.subprocesses.can_daemon.main"
  - "--config-key"
  - "robot_controller"
```

### 현재 문제

각 subprocess는 아직 `--config-key`를 지원하지 않습니다.

현재 주요 CLI:

```text
can_daemon.main: --config
aux_reader.main: --controller-config
task_controller.main: --controller-config, --policy-config-dir 등
dashboard.main/backend: DASHBOARD_CONFIG env 기반
```

### 수정 대상

```text
robot_controller/subprocesses/can_daemon/main.py
robot_controller/subprocesses/aux_reader/main.py
robot_controller/subprocesses/task_controller/main.py
robot_controller/subprocesses/joint_initializer/main.py
robot_controller/subprocesses/dashboard/backend/app.py
run_mujoco_simulation.py
```

### 권장 방식

모든 subprocess에서 `config_paths.yaml`를 직접 읽고 key로 resolve합니다.

```python
parser.add_argument("--config-key", default="robot_controller")
```

또는 controller용은 명확히:

```python
parser.add_argument("--controller-config-key", default="robot_controller")
```

하지만 일관성을 위해 `--config-key` 하나로 통일하는 쪽이 좋습니다.

### 과도기 호환

기존 `--config`도 당분간 유지할 수 있습니다.

```text
우선순위:
1. --config 직접 path가 있으면 그것 사용
2. --config-key가 있으면 config_paths.yaml에서 resolve
3. default key 사용
```

---

## 11. Step 11 — dashboard backend를 새 schema로 이관하고 direct CAN TX 제거

### 현재 YAML

```yaml
dashboard:
  host: "127.0.0.1"
  port: 8000
  state_update_rate: 24.0

can_monitor: ...
spg_monitor: ...
zero_set_presets: ...
```

### 현재 문제

dashboard backend는 아직 다음을 기대합니다.

```text
dashboard.yaml의 platform_config
dashboard.yaml의 robot_controller_config
dashboard.transmit_ids
dashboard.state_hz
platform.enabled_actuators
platform.can.*
platform.shm.*
platform.imu.*
platform.spg_mit.*
```

그리고 raw CAN 송신 endpoint와 actuator direct command endpoint가 남아 있습니다.

```text
POST /api/tx/lock
POST /api/tx/unlock
POST /api/can/send
POST /api/actuator/{can_id}/enter
POST /api/actuator/{can_id}/exit
CommandService.send_raw()
CommandService.motor_enter()
CommandService.motor_exit()
frontend transmit panel
```

### 수정 대상

```text
robot_controller/subprocesses/dashboard/backend/app.py
robot_controller/subprocesses/dashboard/backend/command_api.py
robot_controller/subprocesses/dashboard/backend/state.py
robot_controller/subprocesses/dashboard/frontend/app.js
```

### 변경 지침

1. `load_config()`는 `config_paths.yaml`에서 다음을 로드합니다.

```text
dashboard.yaml
robot_controller.yaml
robot_platform.yaml
can_device_config.yaml
processes.yaml
```

2. `state_hz` 사용을 `state_update_rate`로 변경합니다.

```python
state_update_rate = require_hz(
    nested(config, "dashboard", "state_update_rate"),
    "dashboard.state_update_rate",
    lo=1.0,
    hi=60.0,
)
```

3. `resolve_transmit_ids()` 삭제.

4. raw CAN TX API 삭제 또는 410/403으로 고정.

권장 삭제:

```text
RawSendRequest
/api/tx/lock
/api/tx/unlock
/api/can/send
/api/actuator/{can_id}/enter
/api/actuator/{can_id}/exit
CommandService.send_raw direct exposure
frontend transmit panel
```

5. zero-set은 operator command 경로만 유지합니다.

```text
Dashboard -> OperatorCommandShm -> RobotController FSM -> CAN frame
```

6. `ZERO_SET_ALLOWED_STATES = {"NORMAL"}` 재검토.

FSM 정책상 zero-set을 `DISABLED`에서만 허용할 계획이면 다음으로 바꾸십시오.

```python
ZERO_SET_ALLOWED_STATES = {"DISABLED"}
```

다만 실제 state source가 `ControllerMode`인지 `safety state` 문자열인지 확인 후 통일해야 합니다.

---

## 12. Step 12 — mujoco.yaml 및 run_mujoco_simulation.py 이관

### 현재 YAML

```yaml
imu_sensors:
  quat_sensor_name: "imu_quat"
  gyro_sensor_name: "imu_gyro"
```

### 현재 문제

`run_mujoco_simulation.py`는 아직 `mujoco.yaml` 안의 `platform_config`를 요구합니다. 또한 policy path와 pd config path를 옛 `platform.robots[robot_name]`에서 얻습니다.

### 수정 대상

```text
run_mujoco_simulation.py
tests/test_mujoco_config.py
```

### 변경 지침

`run_mujoco_simulation.py`는 config path registry를 통해 다음을 로드해야 합니다.

```text
mujoco.yaml
robot_platform.yaml
policy_runner.yaml
policy paths from config_paths.yaml
```

모델 경로:

```python
model_path = robot_platform.assets.mujoco_model_path
```

policy config 경로:

```python
policy_list_path = paths.policy_path("policy_list")
pd_config_path = paths.policy_path("pd_config")
```

또는 `config/app_config/policy_runner.yaml`를 통해 active policy를 결정합니다.

### 테스트 수정

현재 `tests/test_mujoco_config.py`는 `mujoco_can`을 기대합니다. 새 schema 기준으로 다음을 검사하도록 변경합니다.

```text
- mujoco.yaml에는 imu_sensors가 root에 존재
- mujoco.yaml에는 mujoco_can 없음
- mujoco.yaml에는 socketcan 없음
- mujoco.yaml에는 spg_mit 없음
- can_device_config.yaml의 drivers.spg_mit.set_zero_hold_s 존재
```

---

## 13. Step 13 — policy runner config 책임 정리

### 현재 상태

`config/app_config/config_paths.yaml`는 `policy_runner`를 가리킵니다.

```yaml
configs:
  policy_runner: "config/app_config/policy_runner.yaml"
```

하지만 ZIP 내부에서 `config/app_config/policy_runner.yaml`는 없습니다. 대신 다음 파일이 존재합니다.

```text
config/policy_config/qhrr/qhrr/policy_runner.yaml
```

### 현재 문제

`config_paths.yaml`가 가리키는 파일이 누락되어 있습니다. 이건 즉시 고쳐야 합니다.

### 선택지 A — app_config에 policy_runner.yaml 추가

```yaml
# config/app_config/policy_runner.yaml

policy_runner:
  active_policy: "qhrr"
  control_hz: 50.0
  rate_log_interval_s: 1.0

pd_control:
  enabled: true
```

이 경우 기존 `config/policy_config/qhrr/qhrr/policy_runner.yaml`는 이름을 `runner_config.yaml`로 바꾸는 것을 권장합니다. 현재 `task_controller.policy_runner.load_bundles()`는 `runner_config.yaml`을 찾기 때문입니다.

### 선택지 B — config_paths.yaml을 실제 존재 파일로 수정

```yaml
configs:
  policy_runner: "config/policy_config/qhrr/qhrr/policy_runner.yaml"
```

하지만 이 경우 app runtime config와 policy session config가 섞입니다. 저는 선택지 A를 권장합니다.

### 추가로 확인할 것

현재 policy loader는 session directory에서 `runner_config.yaml`를 기대합니다.

```text
session_dir / "runner_config.yaml"
```

그런데 현재 파일명은 `policy_runner.yaml`입니다. 따라서 현재 task_controller가 실제로 동작하려면 파일명 또는 loader 중 하나를 수정해야 합니다.

권장:

```text
config/policy_config/qhrr/qhrr/policy_runner.yaml -> runner_config.yaml 로 rename
```

---

## 14. Step 14 — strict removed-key validation 추가

새 schema로 이관한 뒤에는 옛 key를 조용히 무시하면 안 됩니다.

### robot_controller.yaml에서 금지할 key

```text
platform_config
processes_config
hardware.require_manual_arm
hardware.require_estop
hardware.allow_enable_on_start
hardware.allowed_can_interfaces
shm.cleanup_stale_on_start
shm.unlink_on_shutdown
can.motors
```

### dashboard.yaml에서 금지할 key

```text
platform_config
robot_controller_config
dashboard.state_hz
dashboard.transmit_ids
safety.allow_direct_can_transmit
```

### mujoco.yaml에서 금지할 key

```text
platform_config
mujoco_can
socketcan
spg_mit
```

### robot_platform.yaml에서 금지할 key

```text
robots
can.interface
can.bitrate
can.daemon_socket
shm
imu
spg_mit
actuators[].enabled
policy_config_dir
pd_config_path
```

### processes.yaml에서 금지할 key

```text
processes[*].env
```

---

## 15. Step 15 — tests 갱신

### 현재 바로 깨지는 테스트

```text
tests/test_config_safety_gate.py
- load_robot_controller_config가 현재 YAML에서 실패
- config.platform.spg_mit 기대
- config.can.motors.enter_on_start 기대

tests/test_mujoco_config.py
- mujoco_can 기대
```

### 추가해야 할 테스트

1. config path registry 테스트

```text
- config_paths.yaml에 모든 required key 존재
- 각 path가 실제 파일을 가리킴
- policy_runner path 존재 여부 검사
```

2. robot platform loader 테스트

```text
- robot_platform.yaml 로드 성공
- actuator name/can_id 중복 검사
- enabled key가 있으면 에러
- can.allowed_interfaces non-empty
```

3. can device loader 테스트

```text
- can_device_config.yaml 로드 성공
- spg_mit.set_zero_hold_s >= 0
- imu CAN ID hex string parse
```

4. composed app config 테스트

```text
- load_robot_controller_config() 성공
- len(config.can.motors.can_ids) == len(config.robot_platform.actuators)
- config.shm.mit_command.target_count == len(config.robot_platform.actuators)
- config.can.interface in config.robot_platform.can.allowed_interfaces
- velocity_damping_kd <= spg_mit.kd_max
```

5. removed key strict test

```text
- old key를 임시 YAML에 넣으면 ConfigError 발생
```

6. dashboard config 테스트

```text
- dashboard.state_update_rate 사용
- dashboard.state_hz 금지
- transmit_ids 금지
- raw CAN transmit endpoint 제거 또는 403/404 확인
```

---

## 16. Step 16 — 문서 갱신은 코드 이관 후 진행

이번 검토 기준에서 `docs/` 문서는 최신 상태로 간주하지 않았습니다. 코드 이관이 끝난 뒤 다음 문서를 갱신하십시오.

```text
docs/CONFIG_SCHEMA.md
docs/ARCHITECTURE.md
docs/CONTROL_LOOP.md
docs/SAFETY.md
docs/IPC_SHM.md
docs/RUNBOOK.md
README.md
TODO_CONFIG_REFACTOR.md
```

단, 문서는 마지막 단계입니다. 먼저 실제 loader와 subprocess가 새 YAML로 실행되는지 확인해야 합니다.

---

## 17. 우선순위별 작업 목록

### P0 — 현재 실행을 막는 항목

```text
1. config_paths.yaml loader 추가
2. load_robot_controller_config에서 platform_config/processes_config 의존 제거
3. robot_platform.yaml 새 loader 구현
4. can_device_config.yaml loader 구현
5. parse_shm_config가 새 shm.name 구조를 읽도록 수정
6. parse_can_config가 robot_controller + robot_platform + can_device 조합을 사용하도록 수정
7. HardwareSafetyConfig에서 삭제된 필드 제거
8. ProcessConfig env -> env_vars 반영
9. config/app_config/policy_runner.yaml 누락 해결 또는 config_paths.yaml 수정
```

### P1 — 실행은 되지만 책임이 깨지는 항목

```text
1. qhrr0_hw robot_spec 생성 경로 수정
2. controller.py에서 config.platform.spg_mit 의존 제거
3. controller.py에서 cleanup_stale_on_start / unlink_on_shutdown 참조 제거
4. validate_runtime_safety에서 삭제된 safety option 참조 제거
5. dashboard backend에서 platform_config / robot_controller_config 요구 제거
6. dashboard state_hz -> state_update_rate 변경
7. dashboard transmit_ids / raw CAN send 제거
8. run_mujoco_simulation.py에서 platform_config 의존 제거
```

### P2 — 테스트/정리 항목

```text
1. tests/test_config_safety_gate.py 갱신
2. tests/test_mujoco_config.py 갱신
3. dashboard frontend에서 transmit panel 제거
4. compatibility alias 정리
5. old PlatformConfig export 이름 정리
6. docs 갱신
```

---

## 18. 권장 적용 순서

```text
Step 1  config_paths.py 추가
Step 2  robot_platform loader 새로 구현
Step 3  can_device_config loader 추가
Step 4  RobotControllerConfig composition 변경
Step 5  shm config parser 변경
Step 6  can config parser 변경
Step 7  validation.py에서 삭제된 key 참조 제거
Step 8  controller.py 생성/cleanup/shutdown 기본 동작 반영
Step 9  qhrr0_hw robot_spec 생성 경로 변경
Step 10 processes env_vars 반영
Step 11 subprocess CLI --config-key 반영
Step 12 dashboard backend/frontend 이관
Step 13 run_mujoco_simulation.py 이관
Step 14 policy_runner 파일명/경로 정리
Step 15 tests 갱신
Step 16 docs 갱신
```

---

## 19. 즉시 확인용 명령

이관 중간마다 아래를 반복 실행하십시오.

```bash
python - <<'PY'
from robot_controller.config.loader import load_yaml_mapping
from pathlib import Path
for p in Path('config/app_config').glob('*.yaml'):
    print(p, list(load_yaml_mapping(p).keys()))
PY
```

최종적으로는 아래가 성공해야 합니다.

```bash
python - <<'PY'
from robot_controller.config import load_robot_controller_config
cfg = load_robot_controller_config()
print(cfg)
PY
```

그리고:

```bash
python -m py_compile \
  robot_controller/config/app.py \
  robot_controller/config/can.py \
  robot_controller/shm/config.py \
  robot_controller/platform/config.py \
  robot_controller/controller.py \
  run_mujoco_simulation.py

python -m pytest -q
```
