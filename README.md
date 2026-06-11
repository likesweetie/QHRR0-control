# QHRR0 Control

GitHub: [likesweetie/QHRR0-control](https://github.com/likesweetie/QHRR0-control)

QHRR 계열 로봇을 위한 Python 기반 제어기 프로젝트입니다.


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

## Safety Notes

- `RobotController.loop()`는 operator command를 읽고 상태 머신을 업데이트한 뒤, 현재 `ControllerMode`별로 정확히 하나의 actuator output path만 실행합니다.
- `ENABLING` 상태에서는 enable command만 송신하며, policy/damping/zero/disable command를 섞지 않습니다.
- Arm 이후에는 `DAMPING` 상태로 머물며, dashboard `Run` 버튼이 `RUN` command를 보낼 때만 `NORMAL`로 전환됩니다.
- `NORMAL` 상태에서만 `ControlCommandShm.read_relaxed()`를 호출하고 policy MIT command를 송신합니다.
- 여러 actuator 대상 enable/disable/zero/damping/policy 송신은 `RobotController` private method의 단순 for-loop에서 직접 보입니다.
- `ControlCommandShm`은 ctypes C-compatible layout이며 motor command tearing을 의도적으로 허용합니다.
- seqlock, sequence counter, zero-set generation은 사용하지 않습니다.


## Useful Commands

```bash
python3 -m robot_controller.main --config config/app_config/robot_controller.yaml
python3 -m robot_controller.subprocesses.can_daemon.main --config-key robot_controller --replace-existing-socket
python3 -m robot_controller.subprocesses.task_controller.main --help
python3 run_mujoco_simulation.py --help
candump -td vcan0
```
