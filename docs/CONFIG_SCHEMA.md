# Config Schema

근거 파일: `config/app_config/*.yaml`, `robot_controller/core/config.py`, `robot_controller/config/validate_hardware_safety.py`.

## Config files

| File | Role |
| --- | --- |
| `config/app_config/platform.yaml` | robot, CAN, SHM, IMU, SPG MIT, actuator identity |
| `config/app_config/robot_controller.yaml` | runtime mode, hardware gate, safety policy, controller/SHM/CAN params |
| `config/app_config/processes.yaml` | child process list |
| `config/app_config/dashboard.yaml` | dashboard host/state rate/TX UI |
| `config/app_config/mujoco.yaml` | MuJoCo CAN bridge config |

## `robot_controller.yaml`

### Runtime / Hardware / Safety

| Key | Type | Required | Safety-critical | Validation |
| --- | --- | --- | --- | --- |
| `runtime.mode` | `simulation | hardware` | yes | **yes** | static value check |
| `hardware.allow_real_can` | bool | yes | **yes** | hardware mode requires true |
| `hardware.require_manual_arm` | bool | yes | **yes** | hardware mode requires true |
| `hardware.require_estop` | bool | yes | **yes** | if true, hardware CLI requires `--estop-ok` |
| `hardware.allow_enable_on_start` | bool | yes | **yes** | hardware mode requires false |
| `hardware.allowed_can_interfaces[]` | list[str] | yes | **yes** | hardware `can.interface` must be listed |
| `safety.velocity_damping_kd` | float | yes | **yes** | `>=0`, `<= can.mit_protocol_range.kd` |
| `safety.damping_timeout_s` | float | yes | **yes** | `>0` |
| `safety.command_loss_action` | enum | yes | **yes** | `damping`, `disable`, or `fault` |
| `safety.feedback_stale_action` | enum | yes | **yes** | `damping`, `disable`, or `fault` |

### Controller / SHM / CAN

| Key | Type | Unit | Required | Safety-critical | Validation |
| --- | --- | --- | --- | --- | --- |
| `robot_controller.control_hz` | float | Hz | yes | **yes** | `>0` |
| `robot_controller.shutdown_timeout_s` | float | s | yes | yes | `>=0` |
| `shm.cleanup_stale_on_start` | bool | none | yes | yes | bool |
| `shm.unlink_on_shutdown` | bool | none | yes | yes | bool |
| `shm.aux_command.size_bytes` | int | bytes | yes | no | `>=4096` |
| `shm.aux_command.publish_hz` | float | Hz | yes | no | `>0` |
| `shm.operator_command.size_bytes` | int | bytes | yes | **yes** | `>=4096` |
| `shm.control_state.size_bytes` | int | bytes | yes | yes | `>=4096` |
| `shm.control_state.publish_hz` | float | Hz | yes | yes | `>0` |
| `shm.dashboard_state.size_bytes` | int | bytes | yes | no | `>=4096` |
| `shm.dashboard_state.publish_hz` | float | Hz | yes | no | `>0` |
| `can.command_timeout_s` | float | s | yes | **yes** | `>0` |
| `can.bringup_delay_s` | float | s | yes | yes | `>=0` |
| `can.motors.enter_on_start` | bool | none | yes | **yes** | runtime gate forbids in simulation/hardware |
| `can.motors.exit_on_shutdown` | bool | none | yes | **yes** | bool |
| `can.motors.set_zero_on_start` | bool | none | yes | **yes** | bool |
| `can.imu.request_all_each_tick` | bool | none | yes | yes | bool |
| `can.mit_protocol_range.*` | float | rad/rad/s/Nm | derived | protocol-critical | derived from `platform.spg_mit`; used for MIT payload quantization, not a safety envelope |

Derived values from `platform.yaml`: CAN interface, daemon socket, SHM names, enabled actuator CAN IDs.

## Runtime Gate Examples

| Config/CLI | Expected |
| --- | --- |
| `runtime.mode: simulation`, `can.interface: vcan0` | pass |
| `runtime.mode: simulation`, `can.interface: can0` | reject |
| `runtime.mode: simulation`, `can.motors.enter_on_start: true` | reject |
| `runtime.mode: hardware`, no `--hardware` | reject |
| `runtime.mode: hardware`, `can.interface: vcan0` | reject |
| `runtime.mode: hardware`, `hardware.allow_real_can: false` | reject |
| `runtime.mode: hardware`, no `--estop-ok` while `require_estop: true` | reject |
| `runtime.mode: hardware`, `can.motors.enter_on_start: true` | reject |

## `platform.yaml`

| Key | Type | Safety-critical | Notes |
| --- | --- | --- | --- |
| `can.interface` | str | **yes** | runtime gate interprets simulation/hardware safety |
| `can.bitrate` | int | **yes** | SocketCAN setup must match |
| `can.daemon_socket` | str | yes | Unix socket path |
| `shm.*` | str | yes | names must be unique |
| `imu.*_id` | int/hex | **yes** | CAN IDs |
| `spg_mit.*` | float | **yes** | protocol scale/limit |
| `actuators[].enabled` | bool | **yes** | at least one enabled |
| `actuators[].can_id` | int/hex | **yes** | duplicate rejected |
| `actuators[].mujoco_joint` | str | sim-critical | MJCF mapping |
| `actuators[].mujoco_actuator` | str | sim-critical | MJCF mapping |
| `actuators[].sign`, `offset_rad` | float | **yes** | mapping/sign convention |

## `processes.yaml`

| Key | Type | Validation |
| --- | --- | --- |
| `processes[].name` | str | duplicate rejected, `can_daemon` required |
| `processes[].command` | list[str] | non-empty |
| `start_order`, `stop_order` | int | sort key |
| `new_terminal` | bool | bool |
| `terminal_command` | list[str] | required when `new_terminal` |
| `working_dir` | str | non-empty |
| `env` | mapping | required |
| `task_controller.env.TASK_CONTROL_HZ` | float-as-string | policy inference loop Hz. Missing value makes `task_controller` startup fail unless `--control-hz` is passed explicitly. |
| `task_controller.env.TASK_RATE_LOG_INTERVAL_S` | float-as-string | seconds between policy output rate reports. Reports successful `qhrr_mit_command` publishes. |

## `mujoco.yaml`

| Key | Type | Unit | Required | Safety-critical | Notes |
| --- | --- | --- | --- | --- | --- |
| `mujoco_can.enabled` | bool | none | yes | sim-critical | MuJoCo CAN bridge on/off |
| `mujoco_can.socketcan.enabled` | bool | none | yes | sim-critical | SocketCAN adapter on/off |
| `mujoco_can.spg_mit.set_zero_hold_s` | float | s | yes | sim-critical | set-zero 후 control ignore hold |

MuJoCo SPG MIT driver는 MIT enter 직후 zero torque command를 latch하고, 이후 MIT control RX frame이 들어올 때마다 latched command를 교체한다. MIT enter는 angle zero reference를 바꾸지 않는다. MuJoCo feedback angle은 `platform.yaml`의 actuator별 `sign`, `offset_rad`를 적용한 logical joint angle이다. `mujoco_can.command_timeout_s`는 지원하지 않는다.

SPG actuator feedback은 MIT enter 이후 MIT command/response 경로에서만 나온다. `task_controller`는 actuator/IMU numeric `qhrr_control_state`가 생길 때까지 policy inference를 시작하지 않고 대기한다.

## 검증 필요 항목

| 항목 | 질문 |
| --- | --- |
| Hardware config split | TODO(owner): simulation/hardware 별도 YAML 파일 분리 여부 |
| E-stop config | TODO(owner): CLI `--estop-ok`를 실제 monitor config로 대체 |
| Actuator mapping | TODO(owner): policy output index와 MJCF actuator mapping 자동 검증 추가 |
