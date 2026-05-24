# QHRR0 Control

GitHub: [likesweetie/QHRR0-control](https://github.com/likesweetie/QHRR0-control)

QHRR 계열 로봇을 위한 Python 기반 제어기 프로젝트입니다. CAN 기반 actuator/IMU bringup, ONNX policy inference, MIT command 송신, process supervision, Robot State dashboard를 하나의 런타임으로 묶어 관리합니다.

현재 기본 설정은 `vcan0` 대상입니다. 실제 로봇에서 실행하기 전에는 반드시 `docs/SAFETY.md`, `docs/CAN_INTERFACE.md`, `docs/CONFIG_SCHEMA.md`를 먼저 확인하세요.

## Quick Start

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

python3 -m robot_controller.main --config app_config/robot_controller.yaml
```

Dashboard는 기본 설정 기준으로 아래 주소에서 실행됩니다.

```text
http://127.0.0.1:8000
```

MuJoCo simulation은 별도 터미널에서 실행합니다.

```bash
python3 run_mujoco_simulation.py
```

## Project Layout

| Path | Description |
| --- | --- |
| `app_config/` | project-wide YAML config |
| `robot_controller/` | main controller, process modules, CAN/SHM runtime |
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

- 실제 hardware mode가 별도 flag로 분리되어 있지 않습니다. `app_config/platform.yaml`의 CAN interface를 바꾸면 실제 SocketCAN에 붙을 수 있습니다.
- `can.motors.enter_on_start: true`이면 controller start 중 actuator enable frame이 전송됩니다.
- `motor_id` 대신 CAN ID를 기준으로 actuator를 식별합니다.
- silent fallback은 금지합니다. fallback policy는 `FALLBACK_POLICY.md`와 `docs/SAFETY.md`를 따릅니다.

## Useful Commands

```bash
python3 -m robot_controller.main --config app_config/robot_controller.yaml
python3 -m robot_controller.process.can_daemon.main --config app_config/robot_controller.yaml --replace-existing-socket
python3 -m robot_controller.process.task_controller.main --help
python3 run_mujoco_simulation.py --help
candump -td vcan0
```
