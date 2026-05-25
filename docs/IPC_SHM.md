# IPC and Shared Memory

근거 파일: `robot_controller/utils/shm_manager.py`, `robot_controller/utils/shm_command_router.py`, `robot_controller/command/shm_policy_command_source.py`, `robot_controller/core/robot_state_shm.py`, `robot_controller/state/state_publisher.py`, `robot_controller/process/task_controller/shm_io.py`, `config/app_config/platform.yaml`, `config/app_config/robot_controller.yaml`.

## Shared Memory 목록

| SHM name | Owner creates | Writer | Reader | Size/source | Frequency/source |
| --- | --- | --- | --- | --- | --- |
| `qhrr_mit_command` | `ShmManager.create_all()` | `task_controller` via `ShmMitCommandWriter` | `RobotController` via `ShmPolicyCommandSource` | header + N targets | task default 50 Hz |
| `qhrr_aux_command` | `ShmManager.create_all()` | `aux_reader` | `task_controller` | `4096` bytes | event-driven; config `publish_hz: 100` is stored but writer publishes on joystick events |
| `qhrr_operator_command` | `ShmManager.create_all()` | dashboard via `OperatorCommandShmWriter` | `RobotController` via `OperatorCommandShmSource` | `4096` bytes | event-driven |
| `qhrr_control_state` | `ShmManager.create_all()` | `StatePublisher` | `task_controller`, dashboard | `16384` bytes | `500` Hz |
| `qhrr_dashboard_state` | `ShmManager.create_all()` | `StatePublisher` | dashboard | `65536` bytes | `10` Hz |

## Lifetime

| 단계 | 동작 |
| --- | --- |
| startup | `cleanup_stale_on_start: true`이면 기존 SHM 이름들을 unlink |
| startup | `create_all()`로 5개 segment 생성 |
| runtime | writer/reader는 `create=False` attach |
| shutdown | `close_all()` 후 `unlink_all()` if `unlink_on_shutdown: true` |

## MIT Command SHM Layout

Constants:

| constant | value |
| --- | --- |
| `MIT_COMMAND_MAGIC` | `0x4D495443` |
| `MIT_COMMAND_VERSION` | `1` |
| `MIT_HEADER_FMT` | `<IIIQQI` |
| `MIT_TARGET_FMT` | `<iddddd` |

Header fields:

| Field | C/Python type | Unit | 설명 |
| --- | --- | --- | --- |
| `magic` | uint32 | none | layout guard |
| `version` | uint32 | none | version guard |
| `target_count` | uint32 | count | enabled actuator count와 같아야 함 |
| `seq` | uint64 | counter | odd/even seqlock-style write marker |
| `timestamp_ns` | uint64 | ns Unix time | writer publish time |
| `source_id` | uint32 | id | writer source. `ShmMitCommandWriter` default `2` |

Target fields:

| Field | C/Python type | Unit |
| --- | --- | --- |
| `can_id` | int32 | CAN standard ID |
| `position_rad` | double | rad |
| `velocity_rad_s` | double | rad/s |
| `kp` | double | MIT gain |
| `kd` | double | MIT gain |
| `torque_ff_nm` | double | Nm |

`target_count`는 `len(platform.enabled_actuators)`에서 유도된다. 불완전한 batch는 invalid이다.

## Robot State SHM Layout

Constants:

| constant | value |
| --- | --- |
| `ROBOT_STATE_MAGIC` | `0x52535453` |
| `ROBOT_STATE_VERSION` | `1` |
| `ROBOT_STATE_HEADER_FMT` | `<IIQQI` |

Header fields:

| Field | C/Python type | Unit | 설명 |
| --- | --- | --- | --- |
| `magic` | uint32 | none | layout guard |
| `version` | uint32 | none | version guard |
| `seq` | uint64 | counter | odd/even seqlock-style marker |
| `timestamp_ns` | uint64 | ns Unix time | writer publish time |
| `payload_len` | uint32 | bytes | UTF-8 JSON payload length |

Payload는 UTF-8 JSON mapping이다. `json.dumps(..., allow_nan=False)`가 사용된다.

## Robot State Field Table

### `qhrr_control_state`

| Field | Type | 설명 |
| --- | --- | --- |
| `schema` | string | `qhrr.control_state.v1` |
| `timestamp_monotonic` | float | `time.monotonic()` |
| `timestamp_unix` | float | `time.time()` |
| `controller_state` | string | `RobotControllerState.name` |
| `imu.quat_xyzw` | list[float] or null | E2Box decoded quaternion |
| `imu.projected_gravity_b` | list[float] or null | body-frame projected gravity |
| `imu.angular_velocity_rad_s` | list[float] or null | gyro |
| `imu.last_quat_t` | float | monotonic timestamp from driver |
| `imu.last_gyro_t` | float | monotonic timestamp from driver |
| `imu.quat_online`, `gyro_online` | bool | driver comm status |
| `imu.quat_stale`, `gyro_stale` | bool | driver comm status |
| `actuators[]` | list[dict] | compact control state per CAN ID |

`actuators[]` fields: `can_id`, `position_rad`, `velocity_rad_s`, `torque_nm`, `current_a`, `is_enabled`, `fault_code`, `last_feedback_t`, `age_s`, `online`, `stale`.

### `qhrr_dashboard_state`

| Field | Type | 설명 |
| --- | --- | --- |
| `schema` | string | `qhrr.dashboard_state.v1` |
| `timestamp_monotonic` | float | `time.monotonic()` |
| `timestamp_unix` | float | `time.time()` |
| `controller_state` | string | `RobotControllerState.name` |
| `can.iface` | string | CAN interface |
| `can.command_timeout_s` | float | timeout |
| `processes` | mapping | `ChildProcessManager.status()` |
| `imu` | mapping | expanded IMU state/comm |
| `actuators[]` | list[dict] | expanded actuator state/comm/raw |

### `qhrr_aux_command`

| Field | Type | 설명 |
| --- | --- | --- |
| `schema` | string | `qhrr.aux_command.v1` |
| `timestamp_monotonic` | float | publish time |
| `timestamp_unix` | float | publish time |
| `lin_vel_target` | list[float] length 3 | joystick axis derived |
| `ang_vel_target` | list[float] length 3 | joystick axis derived |
| `buttons` | mapping[str,bool] | `a_button`, `b_button`, ... |

### `qhrr_operator_command`

| Field | Type | 설명 |
| --- | --- | --- |
| `schema` | string | `qhrr.operator_command.v1` |
| `timestamp_monotonic` | float | publish time |
| `timestamp_unix` | float | publish time |
| `command_id` | int | one-shot command id |
| `source` | string | writer source, currently `dashboard` |
| `arm` | bool | operator arm request |
| `clear_fault` | bool | `FAULT_LATCHED` clear request |
| `estop` | bool | E-stop request |

## Synchronization 방식

| SHM | 방식 |
| --- | --- |
| Robot State JSON SHM | writer가 odd seq로 header를 먼저 쓰고 payload 작성 후 even seq로 commit. reader는 header 두 번 읽어 seq mismatch/odd면 `None` |
| Operator Command JSON SHM | Robot State JSON SHM header를 재사용하고 `command_id` 중복을 one-shot으로 무시 |
| MIT Command SHM | writer가 odd seq 후 target array 작성, even seq commit. reader는 collision 시 짧게 retry |
| Mutex/lock | 확인되지 않음 |

## Python/C++ ABI 주의사항

| 항목 | 주의 |
| --- | --- |
| endianness | format string이 `<` little-endian을 명시 |
| padding | Python `struct` standard size/no alignment. C/C++로 접근 시 packed layout 필요 |
| double size | MIT target은 Python double 5개 |
| JSON state | C++ reader가 필요하면 header 뒤 UTF-8 JSON payload를 decode해야 함 |
| old MuJoCo SHM | legacy `ShmData` 구조와 현재 Robot State JSON SHM은 다른 layout이다 |

## Struct 변경 규칙

| 변경 | 규칙 |
| --- | --- |
| MIT header/target 변경 | `MIT_COMMAND_VERSION` bump 필요 |
| Robot State header 변경 | `ROBOT_STATE_VERSION` bump 필요 |
| JSON field 추가 | reader가 optional로 처리 가능한지 확인 |
| JSON field 삭제/rename | dashboard/task_controller reader 업데이트 필요 |
| target_count 변경 | `platform.enabled_actuators`와 writer/reader를 동시에 변경 |

## 검증 필요 항목

| 항목 | 질문 |
| --- | --- |
| source_id | TODO(owner): MIT `source_id=2`의 의미와 source registry 정의 |
| aux_command publish_hz | TODO(owner): config 값이 실제 writer scheduling에 쓰이지 않는 것이 의도인지 확인 |
| C++ consumers | TODO(owner): 현재 JSON Robot State SHM을 읽는 C++ process 존재 여부 |
| SHM cleanup | TODO(owner): 비정상 종료 후 resource_tracker warning/segment orphan 운영 절차 |
