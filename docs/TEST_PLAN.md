# Test Plan

이 문서는 현재 코드 entrypoint를 기준으로 한 수동/반자동 test plan이다. repository에서 공식 automated test suite는 확인되지 않았다.

## Smoke Test

| Test | Command | Expected result |
| --- | --- | --- |
| Config import | `python3 -m robot_controller.main --help` | help 출력, import error 없음 |
| CAN daemon help | `python3 -m robot_controller.process.can_daemon.main --help` | help 출력 |
| Task controller help | `python3 -m robot_controller.process.task_controller.main --help` | help 출력 |
| MuJoCo launcher help | `python3 run_mujoco_simulation.py --help` | help 출력 |
| Dashboard import/run | `python3 -m robot_controller.process.dashboard.backend.app` | `127.0.0.1:8000` bind 시도 |

## Unit Test

UNKNOWN: 현재 repository에서 pytest/unittest test suite는 확인되지 않았다.

권장 최소 단위 테스트 대상:

| Target | Acceptance criteria |
| --- | --- |
| `SPGActuatorProtocol._pack_mit_payload()` | limit boundary와 out-of-range rejection |
| `E2BoxIMUProtocol._decode_quat()` | `qz,qy,qx,qw` raw order, `qx` sign correction, normalize |
| `ShmMitCommandWriter/Router` | complete batch read, incomplete batch reject, seq collision retry |
| `RobotStateShmWriter/Reader` | schema payload publish/read, invalid magic/version reject |
| `load_robot_controller_config()` | missing key, duplicate CAN ID, kd lower bound rejection |

## Integration Test: vcan CAN Daemon

준비:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

터미널 1:

```bash
candump -td vcan0
```

터미널 2:

```bash
python3 -m robot_controller.process.can_daemon.main --config app_config/robot_controller.yaml --replace-existing-socket
```

터미널 3:

```bash
cansend vcan0 221#03
```

Expected result:

| 확인 | 기대 |
| --- | --- |
| `candump` | `221#03` frame 표시 |
| `can_daemon` | fatal traceback 없음 |
| `/tmp/qhrr_can_daemon.sock` | daemon 실행 중 존재 |

## Integration Test: Full Controller on vcan

```bash
python3 -m robot_controller.main --config app_config/robot_controller.yaml
```

Expected result:

| 확인 | 기대 |
| --- | --- |
| child terminals | `can_daemon`, `aux_reader`, `task_controller`, `dashboard` 시작 시도 |
| log dir | `log/<timestamp>/` 생성 |
| dashboard | `http://127.0.0.1:8000` 접속 가능 |
| shutdown | Ctrl+C 후 child process 정리, SHM unlink |

주의: `/dev/input/js0`가 없으면 `aux_reader`가 실패할 수 있다. 이 경우 `task_controller`는 `aux_command` read 실패로 종료할 수 있다.

## Simulation Test

터미널 1:

```bash
python3 run_mujoco_simulation.py
```

터미널 2:

```bash
python3 -m robot_controller.main --config app_config/robot_controller.yaml
```

Expected result:

| 확인 | 기대 |
| --- | --- |
| MuJoCo | `mujoco_simulate` window/process 시작 |
| CAN | `vcan0`에 IMU request, actuator command/feedback traffic |
| Dashboard Robot State | control_state/dashboard_state online |

UNKNOWN: 현재 환경에서 GUI/OpenGL availability는 문서 작성 중 검증하지 않았다.

## Hardware Dry-run Test

실제 actuator enable 전:

| Step | Command/확인 | Expected result |
| --- | --- | --- |
| 1 | `ip link show <iface>` | interface UP |
| 2 | `candump -td <iface>` | background traffic 확인 |
| 3 | `app_config/platform.yaml` review | CAN ID/interface/bitrate 확인 |
| 4 | `can.motors.enter_on_start: false` dry-run config 검토 | 시작 시 enable frame 방지 여부 확인 |
| 5 | controller start | IMU request와 expected traffic 확인 |

UNKNOWN: 별도 dry-run config 파일은 현재 확인되지 않았다.

## Fault Injection Test

| Fault | Injection | Expected result |
| --- | --- | --- |
| No MIT command batch | `task_controller` 미실행 또는 종료 | log `Sending one damping command because no MIT command batch available` |
| Stale MIT command | `task_controller`를 멈춤 | `can.command_timeout_s` 이후 stale damping once |
| Incomplete MIT batch | target count 부족한 writer로 publish | `ValueError("Incomplete MIT command batch...")`, controller shutdown |
| NaN action | MIT target field에 NaN publish | `validate_mit_batch()` fatal, shutdown path |
| CAN daemon socket stale | socket file 남기고 daemon start without replace flag | RuntimeError: socket already exists |
| Child ignores SIGTERM | SIGTERM 무시 process로 대체 | supervisor warning 후 SIGKILL |
| Robot State SHM version mismatch | magic/version 변조 | reader RuntimeError/dashboard SHM error |

## Acceptance Criteria

| Area | Criteria |
| --- | --- |
| Startup | configured processes start in `start_order`; logs created |
| CAN | TX command appears on `candump`; RX callbacks update state |
| SHM | `qhrr_control_state` publishes schema `qhrr.control_state.v1` |
| Policy | `task_controller` writes complete MIT batch for all configured CAN IDs |
| Safety | missing/stale MIT batch never holds previous command indefinitely |
| Shutdown | Ctrl+C sends damping once, optional MIT_EXIT, stops child processes |
| Dashboard | process and SHM pages show current process/SHM status |

## 검증 필요 항목

| 항목 | 질문 |
| --- | --- |
| automated tests | TODO(owner): pytest/unit test harness 추가 여부 |
| hardware acceptance | TODO(owner): 실제 로봇 dry-run acceptance criteria 확정 |
| simulator determinism | TODO(owner): MuJoCo regression scenario와 expected telemetry 정의 |
| CI | TODO(owner): config load/static validation CI 도입 여부 |
