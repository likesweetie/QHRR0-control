# QHRR0 Control

GitHub: [likesweetie/QHRR0-control](https://github.com/likesweetie/QHRR0-control)

QHRR 계열 로봇을 위한 Python 기반 제어기 프로젝트입니다. CAN 기반 actuator/IMU bringup, ONNX policy inference, MIT command 송신, child process supervision, Robot State dashboard를 하나의 런타임으로 묶어 관리합니다.

현재 기본 설정은 `runtime.mode: simulation`, CAN interface `vcan0` 대상입니다. 실제 로봇에서 실행하기 전에는 반드시 `docs/SAFETY.md`, `docs/CAN_INTERFACE.md`, `docs/CONFIG_SCHEMA.md`를 먼저 확인하세요.

## Simulation Quick Start

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

python3 -m robot_controller.main --config config/app_config/robot_controller.yaml
```

Dashboard는 기본 설정 기준으로 아래 주소에서 실행됩니다.

```text
http://127.0.0.1:8000
```

MuJoCo simulation은 별도 터미널에서 실행합니다.

```bash
python3 run_mujoco_simulation.py
```

## Hardware Mode

Hardware mode는 YAML 변경만으로 실행되지 않습니다. `config/app_config/robot_controller.yaml`에서 `runtime.mode: hardware`를 설정한 뒤, 실제 CAN interface와 hardware gate를 명시적으로 통과해야 합니다.

```bash
python3 -m robot_controller.main \
  --config config/app_config/robot_controller.yaml \
  --hardware \
  --i-understand-this-can-enable-motors \
  --estop-ok
```

Hardware mode startup validation:

| Gate | Required behavior |
| --- | --- |
| `--hardware` | hardware mode 실행 의도 확인 |
| `--i-understand-this-can-enable-motors` | 실제 motor enable 가능성을 명시적으로 확인 |
| `--estop-ok` | `hardware.require_estop: true`일 때 E-stop 확인 |
| `hardware.allow_real_can` | hardware mode에서 `true`여야 함 |
| `hardware.require_manual_arm` | hardware mode에서 `true`여야 함 |
| `hardware.allow_enable_on_start` | hardware mode에서 `false`여야 함 |
| `can.motors.enter_on_start` | hardware mode에서 금지 |

현재 코드에는 operator manual arm 입력 경로가 아직 연결되어 있지 않습니다. Hardware mode는 `SafetyState.DISARMED`에서 시작하며, startup 중 motor enable command를 보내지 않습니다.

## Project Layout

| Path | Description |
| --- | --- |
| `config/app_config/` | project-wide YAML config |
| `robot_controller/hardware/` | `MotorBus`, `ImuBus`, `RobotHardware`, CAN transport |
| `robot_controller/command/` | policy command source와 command validator |
| `robot_controller/safety/` | `SafetyController`, `SafetyState`, `ControlAction` |
| `robot_controller/state/` | Robot State SHM publisher |
| `robot_controller/processes/` | child process health/management |
| `robot_controller/process/` | child process entrypoints |
| `docs/` | handoff, architecture, safety, runbook 문서 |
| `config/` | policy/controller 관련 설정 |
| `policy/` | ONNX policy artifacts |
| `third_party/` | external dependencies and assets |

## Main Documents

| Document | Purpose |
| --- | --- |
| `docs/HANDOFF.md` | 새 개발자를 위한 핵심 요약 |
| `docs/ARCHITECTURE.md` | runtime/process/data-flow 구조 |
| `docs/RUNBOOK.md` | 실행, 종료, 복구 절차 |
| `docs/SAFETY.md` | fallback, timeout, fault, 실제 로봇 checklist |
| `docs/CONTROL_LOOP.md` | controller tick, policy inference, MIT command 흐름 |
| `docs/IPC_SHM.md` | shared memory layout와 ownership |
| `docs/CAN_INTERFACE.md` | CAN ID, payload, debugging |
| `docs/CONFIG_SCHEMA.md` | YAML config schema |
| `docs/TEST_PLAN.md` | smoke/integration/fault injection test |

## Safety Notes

- `RobotController.run_once()`는 `read feedback -> read command -> evaluate safety -> act -> publish` 흐름을 드러냅니다.
- Safety decision은 `SafetyController`에서 수행합니다.
- `runtime.mode: simulation`에서 `can0` 같은 real CAN interface는 reject됩니다.
- `runtime.mode: hardware`에서 `vcan0`는 reject됩니다.
- `can.motors.enter_on_start: true`는 simulation/hardware startup gate에서 금지됩니다.
- `motor_id` 대신 CAN ID를 기준으로 actuator를 식별합니다.
- silent fallback은 금지합니다. fallback policy는 `FALLBACK_POLICY.md`와 `docs/SAFETY.md`를 따릅니다.

## Useful Commands

```bash
python3 -m robot_controller.main --config config/app_config/robot_controller.yaml
python3 -m robot_controller.process.can_daemon.main --config config/app_config/robot_controller.yaml --replace-existing-socket
python3 -m robot_controller.process.task_controller.main --help
python3 run_mujoco_simulation.py --help
candump -td vcan0
```
