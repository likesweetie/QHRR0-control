# Runbook

## Install / Environment

Use the project Python environment. Dependencies are project-local; do not install third-party dependencies into `third_party/` manually.

## vcan Setup

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip link show vcan0
```

## Simulation Runtime

```bash
python3 -m robot_controller.main --config config/app_config/robot_controller.yaml
```

Optional MuJoCo simulation:

```bash
python3 run_mujoco_simulation.py
```

## Hardware Runtime

Only run after checking `config/app_config/robot_controller.yaml` and `config/app_config/platform.yaml`.

```bash
python3 -m robot_controller.main \
  --config config/app_config/robot_controller.yaml \
  --hardware \
  --i-understand-this-can-enable-motors \
  --estop-ok
```

## Child Process Entrypoints

```bash
python3 -m robot_controller.subprocesses.can_daemon.main --help
python3 -m robot_controller.subprocesses.task_controller.main --help
python3 -m robot_controller.subprocesses.aux_reader.main --help
python3 -m robot_controller.subprocesses.dashboard.main
```

`ProcessSupervisor` usually starts these from `config/app_config/processes.yaml`.

## Shutdown

Use `Ctrl-C` in the controller terminal. `RobotController.shutdown()` requests disable-all once if CAN is connected, stops child processes, closes SHM handles, and unlinks SHM when configured.

## Logs

Each controller run creates:

```text
log/<YYYYMMDD_HHMMSS>/<process>.log
```

For child processes launched in a new terminal, stdout/stderr is also tee'd to the process log file.

## Debug Commands

```bash
candump -td vcan0
cansend vcan0 221#03
rg -n "ControllerMode|OperatorCommandCode" robot_controller
rg -n "qhrr0_hw|SPG|DongilC|E2BOX|joint|calibration" hal -g '*.py'
```

## Recovery After Abnormal Exit

```bash
pkill -f robot_controller.subprocesses
find /dev/shm -maxdepth 1 -name 'qhrr_*' -print
```

If stale SHM exists, `shm.cleanup_stale_on_start: true` lets the controller cleanup configured segments at startup.
