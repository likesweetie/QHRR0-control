# Config Schema

근거 파일: `app_config/*.yaml`, `robot_controller/core/platform_config.py`, `robot_controller/core/config.py`, `robot_controller/process/dashboard/backend/app.py`, `run_mujoco_simulation.py`.

## YAML Config 파일 목록

| 파일 | 역할 |
| --- | --- |
| `app_config/platform.yaml` | robot, CAN, SHM, IMU, SPG MIT, actuator identity를 공유하는 upstream config |
| `app_config/robot_controller.yaml` | robot controller runtime, SHM size/frequency, CAN daemon, bringup, MIT runtime limit |
| `app_config/processes.yaml` | `ProcessSupervisor`가 관리하는 subprocess 목록 |
| `app_config/dashboard.yaml` | FastAPI dashboard, monitor windows, dashboard TX policy |
| `app_config/mujoco.yaml` | MuJoCo CAN bridge 설정 |

Config dataclass에 safety-critical 상수 default를 넣지 않는 구조다. 대부분 key는 loader에서 required로 읽고 누락 시 exception이 난다.

## `platform.yaml`

| Key | Type | Unit | Required | Default | Safety-critical | Validation |
| --- | --- | --- | --- | --- | --- | --- |
| `robot.name` | str | none | yes | no | yes | `robots` entry 존재 |
| `robots.<name>.model_path` | str | path | yes | no | no | load 시 path existence는 `run_mujoco_simulation.py`에서 확인 |
| `robots.<name>.policy_config_dir` | str | path | yes | no | yes | task_controller에서 존재 확인 |
| `robots.<name>.pd_config_path` | str | path | yes | no | yes | MuJoCo launch env에서 존재 확인 |
| `can.interface` | str | interface | yes | no | **yes** | non-empty semantic validation 없음 |
| `can.bitrate` | int | bit/s | yes | no | **yes** | int conversion |
| `can.daemon_socket` | str | path | yes | no | yes | non-empty validation은 controller config에서 수행 |
| `shm.*` | str | name | yes | no | yes | controller config에서 unique check |
| `imu.request_id/quat_id/gyro_id` | int or hex str | CAN ID | yes | no | **yes** | int conversion |
| `imu.cmd_get_*` | int or hex str | byte | yes | no | yes | int conversion |
| `imu.quat_scale/gyro_scale` | float | scale | yes | no | **yes** | numeric conversion |
| `imu.normalize_quat` | bool | none | yes | no | yes | bool conversion |
| `spg_mit.*` | float | rad/rad/s/Nm/etc | yes | no | **yes** | numeric conversion |
| `actuators[].name` | str | none | yes | no | yes | duplicate name rejected |
| `actuators[].enabled` | bool | none | yes | no | **yes** | at least one enabled |
| `actuators[].can_id` | int or hex str | CAN ID | yes | no | **yes** | duplicate CAN ID rejected |
| `actuators[].mujoco_joint` | str | name | yes | no | sim-critical | no existence check here |
| `actuators[].mujoco_actuator` | str | name | yes | no | sim-critical | no existence check here |
| `actuators[].sign` | float | multiplier | yes | no | **yes** | numeric conversion |
| `actuators[].offset_rad` | float | rad | yes | no | **yes** | numeric conversion |

## `robot_controller.yaml`

| Key | Type | Unit | Required | Default | Safety-critical | Validation |
| --- | --- | --- | --- | --- | --- | --- |
| `platform_config` | str | path | yes | no | yes | required |
| `processes_config` | str | path | yes | no | yes | required |
| `robot_controller.name` | str | none | yes | no | no | required |
| `robot_controller.control_hz` | float | Hz | yes | no | **yes** | `> 0` |
| `robot_controller.shutdown_timeout_s` | float | s | yes | no | yes | `>= 0` |
| `shm.cleanup_stale_on_start` | bool | none | yes | no | yes | bool |
| `shm.unlink_on_shutdown` | bool | none | yes | no | yes | bool |
| `shm.aux_command.size_bytes` | int | bytes | yes | no | no | `>= 4096` |
| `shm.aux_command.publish_hz` | float | Hz | yes | no | no | TODO(owner): `>0` validation not confirmed |
| `shm.control_state.size_bytes` | int | bytes | yes | no | yes | `>= 4096` |
| `shm.control_state.publish_hz` | float | Hz | yes | no | **yes** | `> 0` |
| `shm.dashboard_state.size_bytes` | int | bytes | yes | no | no | `>= 4096` |
| `shm.dashboard_state.publish_hz` | float | Hz | yes | no | no | `> 0` |
| `can.command_timeout_s` | float | s | yes | no | **yes** | `> 0` |
| `can.bringup_delay_s` | float | s | yes | no | yes | `>= 0` |
| `can.daemon.rx_timeout_s` | float | s | yes | no | yes | `>= 0` |
| `can.daemon.tx_timeout_s` | float | s | yes | no | yes | `>= 0` |
| `can.daemon.join_timeout_s` | float | s | yes | no | yes | `>= 0` |
| `can.daemon.max_tx_queue_size` | int | count | yes | no | yes | `> 0` |
| `can.daemon.send_block` | bool | none | yes | no | yes | bool |
| `can.daemon.send_timeout_s` | float/null | s | yes | no | yes | null or `>= 0` |
| `can.daemon.connect_timeout_s` | float | s | yes | no | yes | `> 0` |
| `can.motors.enter_on_start` | bool | none | yes | no | **yes** | bool |
| `can.motors.exit_on_shutdown` | bool | none | yes | no | **yes** | bool |
| `can.motors.set_zero_on_start` | bool | none | yes | no | **yes** | bool |
| `can.imu.enabled` | bool | none | yes | no | yes | bool |
| `can.imu.request_all_on_start` | bool | none | yes | no | yes | bool |
| `can.imu.request_all_each_tick` | bool | none | yes | no | yes | bool |
| `can.imu.startup_request_count` | int | count | yes | no | no | `>= 0` |
| `can.imu.startup_request_delay_s` | float | s | yes | no | no | `>= 0` |
| `can.mit_limits.*` | float | rad/rad/s/Nm | yes | no | **yes** | position/velocity/torque `>0`, kp `>=0`, kd `>=0.5` |

Derived from `platform.yaml`: `shm.*.name`, `shm.mit_command.target_count`, `can.interface`, `can.daemon.ipc_socket_path`, `can.motors.can_ids`.

## `processes.yaml`

| Key | Type | Required | Safety-critical | Validation |
| --- | --- | --- | --- | --- |
| `processes[]` | list | yes | yes | list |
| `name` | str | yes | yes | duplicate rejected; `can_daemon` required |
| `command` | list[str] | yes | yes | non-empty |
| `start_order` | int | yes | yes | int |
| `stop_order` | int | yes | yes | int |
| `new_terminal` | bool | yes | no | bool |
| `terminal_command` | list[str] | yes | no | non-empty when `new_terminal` |
| `working_dir` | str | yes | yes | non-empty |
| `env` | mapping[str,str] | yes | yes | mapping |

## `dashboard.yaml`

| Key | Type | Unit | Required | Default | Safety-critical | Validation |
| --- | --- | --- | --- | --- | --- | --- |
| `platform_config` | str | path | yes | no | yes | required |
| `dashboard.host` | str | host | yes | no | no | required |
| `dashboard.port` | int | TCP port | yes | no | no | int |
| `dashboard.state_hz` | float | Hz | yes | no | no | websocket requires `[1,60]` |
| `dashboard.transmit_ids[]` | list | none | yes | no | yes | each item requires `platform_ref` or `actuator`; direct `can_id` forbidden |
| `robot_controller_state.enabled` | bool | none | yes | no | no | required |
| `robot_controller_state.stale_timeout_s` | float | s | yes | no | no | numeric |
| `can.bus_window_s` | float | s | yes | no | no | numeric |
| `can.heartbeat_window_s` | float | s | yes | no | no | no explicit range check |
| `can.node_timeout_s` | float | s | yes | no | no | numeric |
| `can.stuff_factor` | float | multiplier | yes | no | no | numeric |
| `can_daemon.connect_timeout_s` | float | s | yes | no | no | numeric |
| `imu.default_poll_hz` | float | Hz | yes | no | yes | `[0.1,1000]` |
| `spg.default_mit_poll_hz` | float | Hz | yes | no | yes | `[0.1,1000]` |
| `safety.tx_enabled_by_default` | bool | none | yes | no | **yes** | bool |
| `safety.allow_actuator_commands` | bool | none | yes | no | **yes** | bool |

Dashboard config is not allowed to override platform-owned keys: CAN interface/bitrate, daemon socket, SHM names, IMU IDs/scales, SPG limits, actuators.

## `mujoco.yaml`

| Key | Type | Unit | Required | Default | Safety-critical | Validation |
| --- | --- | --- | --- | --- | --- | --- |
| `platform_config` | str | path | yes | no | yes | `run_mujoco_simulation.py` requires |
| `mujoco_can.enabled` | bool | none | yes | no | sim-critical | C++ loader behavior 확인 필요 |
| `mujoco_can.base_body_name` | str | MJCF body | yes | no | sim-critical | UNKNOWN validation |
| `mujoco_can.command_timeout_s` | float | s | yes | no | sim-critical | UNKNOWN validation |
| `mujoco_can.socketcan.enabled` | bool | none | yes | no | sim-critical | UNKNOWN validation |
| `mujoco_can.spg_mit.periodic_feedback` | bool | none | yes | no | sim-critical | UNKNOWN validation |
| `mujoco_can.spg_mit.periodic_feedback_s` | float | s | yes | no | sim-critical | UNKNOWN validation |
| `mujoco_can.spg_mit.set_zero_hold_s` | float | s | yes | no | sim-critical | UNKNOWN validation |

## 잘못된 Config 예시와 기대 동작

| 잘못된 config | 기대 동작 |
| --- | --- |
| `actuators[].can_id` 중복 | `PlatformConfigError("Duplicate actuator CAN ID...")` |
| enabled actuator가 0개 | `PlatformConfigError("at least one actuator must be enabled")` |
| `can.motors.can_ids` empty derived condition | `ConfigError("can.motors.can_ids must not be empty")` |
| `shm` name 중복 | `ConfigError("shm segment names must be unique")` |
| `can.mit_limits.kd < 0.5` | `ConfigError("can.mit_limits.kd must be >= 0.5 for shutdown damping")` |
| `dashboard.transmit_ids[].can_id` 직접 지정 | `ValueError("...can_id must come from platform_ref or actuator")` |
| unknown dashboard actuator reference | `ValueError("...references unknown actuator...")` |
| process list에 `can_daemon` 없음 | `ConfigError("processes must include a 'can_daemon' subprocess")` |

## 검증 필요 항목

| 항목 | 질문 |
| --- | --- |
| dashboard ranges | TODO(owner): `can.node_timeout_s`, `stuff_factor` 등 dashboard numeric key range 검증 필요 여부 |
| mujoco validation | TODO(owner): C++ `mujoco.yaml` loader의 validation rule 표 보강 |
| aux publish_hz | TODO(owner): `shm.aux_command.publish_hz`가 의미 있는 config인지 삭제/검증 결정 |
| hardware config split | TODO(owner): 실제 로봇용 config와 vcan/sim config 분리 정책 |
