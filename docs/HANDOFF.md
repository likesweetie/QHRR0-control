# QHRR Robot Controller Handoff

이 문서는 `README.md`를 근거로 사용하지 않고, 현재 코드와 `config/app_config/*.yaml`에서 확인한 내용만 정리한다. `third_party/`는 handoff 분석 대상에서 제외한다.

## 한눈에 보는 요약

이 프로젝트의 핵심 의도는 **CAN, SHM, 제어 루프, safety decision, 대시보드, 시뮬레이션을 명확한 런타임 경계로 분리**하는 것이다. 최상위 실행점은 `python3 -m robot_controller.main --config config/app_config/robot_controller.yaml`이며, `RobotController`가 SHM을 만들고 `ChildProcessManager`가 하위 프로세스를 시작/종료한다.

현재 확인된 기본 구성은 `runtime.mode: simulation`, `vcan0` 대상이다. 실제 CAN interface, actuator CAN ID, SHM 이름, policy path는 모두 `config/app_config/`에서 읽는다. **motor_id 개념은 런타임 문서 기준에서 사용하지 않는다. 장치 식별은 CAN ID 기준이다.**

안전 철학은 `FALLBACK_POLICY.md`에 정리되어 있다. silent fallback은 금지이며, missing critical config는 fatal이어야 한다. 런타임에서 확인된 fallback은 `SAFETY.md`에 trigger, action, recovery, log, test 형태로 표기했다. 새 개발자는 실제 로봇에서 실행하기 전에 `SAFETY.md`, `CAN_INTERFACE.md`, `CONFIG_SCHEMA.md`를 먼저 확인해야 한다.

## 프로젝트 목적

| 항목 | 내용 |
| --- | --- |
| 목적 | QHRR 계열 로봇의 CAN 기반 actuator/IMU bringup, policy inference, MIT command 송신, Robot State dashboard 제공 |
| 기본 대상 | `config/app_config/platform.yaml` 기준 `robot.name: qhrr`, CAN interface `vcan0` |
| 제어 경로 | CAN feedback -> `MotorBus`/`ImuBus` -> `qhrr_control_state` SHM -> `task_controller` -> `qhrr_mit_command` SHM -> `ShmPolicyCommandSource` -> `SafetyController` -> `MotorBus` -> CAN daemon -> SocketCAN |
| 관측/모니터링 | `qhrr_control_state` 500 Hz, `qhrr_dashboard_state` 10 Hz, dashboard websocket `state_hz: 24` |
| 시뮬레이션 | `run_mujoco_simulation.py`가 MuJoCo `mujoco_simulate`를 build/launch |

## 책임 범위

| 범위 | 포함 |
| --- | --- |
| Robot controller | SHM 생성/해제, CAN daemon 연결, `run_once()` control tick 실행 |
| Child process manager | `can_daemon`, `aux_reader`, `task_controller`, `dashboard` 프로세스 시작/종료, pidfile/log/health 관리 |
| Safety controller | command/process/feedback/hardware 상태를 보고 `ControlAction` 결정 |
| Hardware layer | `MotorBus`, `ImuBus`, `CanTransport`로 실제 CAN command/feedback 처리 |
| CAN daemon | HAL `SocketCANBus`와 `CANDaemon`을 소유하고 Unix socket으로 TX/RX 중계 |
| Task controller | `control_state`와 `aux_command`를 읽어 ONNX policy action을 계산하고 MIT command SHM에 target batch 작성 |
| Dashboard | Robot State, CAN monitor, process status, SHM view를 표시하고 제한된 CAN command API 제공 |

## 비책임 범위

| 항목 | 상태 |
| --- | --- |
| 실제 로봇 전원/전기적 안전 절차 | UNKNOWN |
| hardware CAN interface bringup script | UNKNOWN |
| actuator firmware 내부 safety state | UNKNOWN |
| 물리 emergency stop 입력 처리 | UNKNOWN: 코드에서 `emergency_stop` state 또는 E-stop input 처리 확인 안 됨 |
| `third_party/` 내부 동작 설명 | handoff 대상 제외 |

## 빠른 시작

현재 config는 `vcan0`를 가리킨다.

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
python3 -m robot_controller.main --config config/app_config/robot_controller.yaml
```

대시보드는 `config/app_config/dashboard.yaml` 기준 `http://127.0.0.1:8000`에서 뜬다. `dashboard` 프로세스는 `ChildProcessManager`가 자동으로 실행한다.

MuJoCo 시뮬레이션 실행 entrypoint:

```bash
python3 run_mujoco_simulation.py
```

## 문서 맵

| 문서 | 읽어야 하는 이유 |
| --- | --- |
| `docs/ARCHITECTURE.md` | 프로세스, 스레드, 디렉토리, 런타임 책임 경계 |
| `docs/RUNBOOK.md` | 실행, 종료, 복구, 로그 확인 절차 |
| `docs/SAFETY.md` | fallback, fault, timeout, 실제 로봇 실행 전 checklist |
| `docs/CONTROL_LOOP.md` | 500 Hz controller loop, 50 Hz task policy, MIT command 흐름 |
| `docs/IPC_SHM.md` | SHM 이름, layout, ownership, synchronization |
| `docs/CAN_INTERFACE.md` | CAN IDs, payload, daemon protocol, candump/cansend |
| `docs/CONFIG_SCHEMA.md` | YAML schema와 safety-critical config |
| `docs/LOGGING.md` | `log/YYYYMMDD_HHMMSS/*.log`, pidfile, CSV legacy logger |
| `docs/TEST_PLAN.md` | smoke/integration/fault injection test |
| `docs/CHANGELOG.md` | handoff 기준 변경 요약 |

## 실제 로봇 실행 전 Checklist

| 확인 | 항목 |
| --- | --- |
| [ ] | **`config/app_config/platform.yaml can.interface`가 실제 CAN interface인지 확인** |
| [ ] | **`config/app_config/platform.yaml actuators[].can_id`가 실제 actuator CAN ID와 일치하는지 확인** |
| [ ] | **`config/app_config/robot_controller.yaml can.motors.enter_on_start`가 의도된 bringup 동작인지 확인** |
| [ ] | **`can.command_timeout_s: 0.05`가 실제 feedback 주기와 맞는지 확인** |
| [ ] | **`platform.spg_mit` protocol range가 firmware MIT packing range와 일치하는지 확인** |
| [ ] | `FALLBACK_POLICY.md`와 `docs/SAFETY.md`의 fallback table을 검토 |
| [ ] | `candump <iface>`에서 dashboard/robot_controller TX echo와 feedback 구분 확인 |
| [ ] | `task_controller` policy path와 `pd_config.yaml`의 `kp`, `kd` 확인 |
| [ ] | `log/<timestamp>/*.log` 저장 위치와 터미널 유지 동작 확인 |

## 가장 중요한 위험 경고 5개

| 위험 | 이유 |
| --- | --- |
| **hardware mode는 CLI flag와 gate 없이는 실행되지 않음** | YAML만으로 real CAN/motor enable로 넘어가지 않게 막는다. |
| **`enter_on_start: true`는 startup gate에서 금지됨** | simulation/hardware mode 모두 startup enable을 reject한다. |
| **MIT command batch가 stale/missing이면 `DAMPING`으로 전환됨** | MIT-enabled actuator feedback이 있으면 q=0, qd=0, kp=0, kd=`safety.velocity_damping_kd`, tau=0 command를 매 tick 보낸다. `safety.damping_timeout_s` 이후에는 `FAULT_LATCHED`로 승격하지만 damping command는 계속 보낸다. |
| **feedback stale은 현재 config에서 fault latch** | `safety.feedback_stale_action: fault` 기준 `FAULT_LATCHED` + disable. |
| **SHM payload/schema mismatch는 reader에서 error 또는 exception이 될 수 있음** | struct 변경 시 writer/reader를 동시에 맞춰야 한다. |

## 검증 필요 항목

| 항목 | 질문 |
| --- | --- |
| 실제 로봇 CAN | TODO(owner): real interface 이름, bitrate bringup 절차, bus-off recovery 절차 |
| E-stop | TODO(owner): 물리 E-stop 입력이 software state와 연결되는지 확인 |
| firmware safety | TODO(owner): actuator firmware의 MIT enter/exit/timeout state machine 문서화 |
| policy 검증 | TODO(owner): ONNX policy output range와 `can.mit_protocol_range`의 관계 검증 |
| dependency install | TODO(owner): Python package/system package 설치 명령 확정 |
