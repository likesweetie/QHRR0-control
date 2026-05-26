# Test Plan

## Smoke Tests

```bash
python3 -m robot_controller.main --config config/app_config/robot_controller.yaml --help
python3 -m robot_controller.subprocesses.can_daemon.main --help
python3 -m robot_controller.subprocesses.task_controller.main --help
python3 -m robot_controller.subprocesses.aux_reader.main --help
```

## Unit Tests

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests
```

Current acceptance criterion: all tests pass.

## Architecture Checks

```bash
test ! -d robot_controller/QHRR0_HW
test ! -d robot_controller/process
test ! -d robot_controller/processes
test ! -d robot_controller/hardware
test ! -d robot_controller/command
test ! -d robot_controller/safety
test ! -d robot_controller/state
rg -n "qhrr0_hw|QHRR0|SPG|DongilC|E2BOX|joint|calibration" hal -g '*.py'
```

Expected: `test` commands succeed; `rg` against `hal` returns no matches.

## State Machine Tests

Covered by `tests/test_state_machine.py`.

| Case | Expected |
| --- | --- |
| startup | `DISABLED` |
| `ENABLE` | `ENABLING` |
| elapsed `enable_duration_s` | `NORMAL` |
| `DAMPING` | `DAMPING` |
| `ZERO_SET` | `ZERO_SETTING` |
| `ESTOP` | `ESTOP`, latched |
| `RESET_FAULT` from `ESTOP` | `DISABLED` |

## SHM Tests

Covered by `tests/test_control_command_shm.py` and `tests/test_robot_state_shm.py`.

| SHM | Expected |
| --- | --- |
| `ControlCommandShm` | ctypes struct read/write, no seqlock requirement |
| `RobotStateShm` | telemetry dict conversion works |
| `OperatorCommandShm` | command code can be published/read |

## Integration Dry Run

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
python3 -m robot_controller.main --config config/app_config/robot_controller.yaml
```

Expected: controller starts subprocesses, dashboard opens on configured host/port, and `Ctrl-C` shuts children down.
