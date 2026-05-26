# Config Schema

Primary config files:

| File | Purpose |
| --- | --- |
| `config/app_config/robot_controller.yaml` | runtime mode, safety gates, controller rate, SHM sizes, CAN daemon options |
| `config/app_config/platform.yaml` | robot selection, CAN interface, SHM names, IMU IDs, actuator CAN IDs/calibration |
| `config/app_config/processes.yaml` | child process command/start/stop policy |
| `config/app_config/dashboard.yaml` | dashboard backend/frontend config |
| `config/app_config/mujoco.yaml` | MuJoCo launcher config |

## robot_controller.yaml

| Key | Type | Required | Notes |
| --- | --- | --- | --- |
| `platform_config` | string | yes | path resolved relative to this file |
| `processes_config` | string | yes | path resolved relative to this file |
| `runtime.mode` | string | yes | `simulation` or `hardware` |
| `hardware.allow_real_can` | bool | yes | must be true in hardware mode |
| `hardware.require_manual_arm` | bool | yes | must be true in hardware mode |
| `hardware.require_estop` | bool | yes | if true, hardware run needs `--estop-ok` |
| `hardware.allow_enable_on_start` | bool | yes | must be false in hardware mode |
| `hardware.allowed_can_interfaces` | list[string] | yes | hardware mode allowlist |
| `state_machine.enable_duration_s` | float seconds | yes | `ENABLING` dwell time |
| `robot_controller.control_hz` | float Hz | yes | main tick rate |
| `robot_controller.shutdown_timeout_s` | float seconds | yes | process stop timeout |
| `shm.*.size_bytes` | int bytes | yes | SHM allocation size |
| `shm.control_state.publish_hz` | float Hz | yes | high-rate state publisher |
| `shm.dashboard_state.publish_hz` | float Hz | yes | low-rate state publisher |
| `can.daemon.*` | mapping | yes | daemon queue/timeouts/socket connect |
| `can.imu.request_all_each_tick` | bool | yes | request IMU every controller tick |
| `can.motors.enter_on_start` | bool | yes | forbidden by startup validation |

## platform.yaml

| Key | Type | Required | Notes |
| --- | --- | --- | --- |
| `can.interface` | string | yes | `vcan0` in simulation config |
| `can.bitrate` | int | yes | setup is external to Python runtime |
| `can.daemon_socket` | string | yes | IPC socket path |
| `shm.*` | string | yes | shared memory segment names |
| `imu.*_id` | int/string hex | yes | E2BOX CAN IDs |
| `spg_mit.*` | float | yes | protocol numeric ranges, not safety limits |
| `actuators[].can_id` | int/string hex | yes | actuator CAN ID |
| `actuators[].sign` | float | yes | calibration sign |
| `actuators[].offset_rad` | float radians | yes | calibration offset |

## processes.yaml

| Key | Type | Required |
| --- | --- | --- |
| `processes[].name` | string | yes |
| `processes[].command` | list[string] | yes |
| `processes[].start_order` | int | yes |
| `processes[].stop_order` | int | yes |
| `processes[].new_terminal` | bool | yes |
| `processes[].terminal_command` | list[string] | yes |
| `processes[].working_dir` | string | yes |
| `processes[].env` | mapping | yes |

## Validation

Validation lives in `robot_controller/core/config.py` and `robot_controller/config/validate_hardware_safety.py`. Missing required keys are fatal; config classes do not silently inject hardware-critical defaults.
