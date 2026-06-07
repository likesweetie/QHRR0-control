# Config Schema

Primary app config paths are registered in `config/app_config/config_paths.yaml`.
Sibling YAML files should not point at each other with ad hoc path keys.

| File | Purpose |
| --- | --- |
| `config_paths.yaml` | project-wide config and policy path registry |
| `robot_controller.yaml` | runtime mode, hardware gate, safety policy, SHM, CAN transport, controller rate |
| `robot_platform.yaml` | robot name, MuJoCo model asset, allowed CAN interfaces, actuator wiring |
| `can_device_config.yaml` | E2BOX IMU protocol IDs and SPG MIT driver profile |
| `processes.yaml` | child process command/start/stop policy |
| `dashboard.yaml` | dashboard server, monitor rates, robot-state SHM reader, zero-set presets |
| `mujoco.yaml` | MuJoCo-specific IMU sensor names |
| `policy_runner.yaml` | policy runner app settings |

## robot_controller.yaml

Removed keys are rejected: `platform_config`, `processes_config`,
`hardware.require_manual_arm`, `hardware.require_estop`,
`hardware.allow_enable_on_start`, `hardware.allowed_can_interfaces`,
`shm.cleanup_stale_on_start`, `shm.unlink_on_shutdown`, and `can.motors`.

| Key | Type | Required | Notes |
| --- | --- | --- | --- |
| `runtime.mode` | string | yes | `simulation` or `hardware` |
| `hardware.allow_real_can` | bool | yes | must be true in hardware mode |
| `safety.velocity_damping_kd` | float | yes | must be <= SPG MIT `kd_max` |
| `safety.command_loss_action` | string | yes | `damping`, `disable`, or `fault` |
| `safety.feedback_stale_action` | string | yes | `damping`, `disable`, or `fault` |
| `safety.damping_timeout_s` | float seconds | yes | parsed and validated |
| `state_machine.enable_duration_s` | float seconds | yes | `ENABLING` dwell time |
| `robot_controller.control_hz` | float Hz | yes | main loop rate |
| `robot_controller.shutdown_timeout_s` | float seconds | yes | process stop timeout |
| `shm.*.name` | string | yes | SHM segment names owned by this file |
| `shm.*.size_bytes` | int bytes | yes | SHM allocation size where applicable |
| `shm.*.publish_hz` | float Hz | yes | state publish rates where applicable |
| `can.interface` | string | yes | must be in `robot_platform.can.allowed_interfaces` |
| `can.bitrate` | int | yes | SocketCAN setup is external to Python runtime |
| `can.daemon_socket` | string | yes | CAN process IPC socket path |
| `can.daemon.*` | mapping | yes | daemon queue/timeouts/client settings |
| `can.imu.*` | mapping | yes | controller-side IMU request cadence |

## robot_platform.yaml

Removed keys are rejected: `robots`, `can.interface`, `can.bitrate`,
`can.daemon_socket`, `shm`, `imu`, `spg_mit`, `actuators[].enabled`,
`policy_config_dir`, and `pd_config_path`.

| Key | Type | Required | Notes |
| --- | --- | --- | --- |
| `robot.name` | string | yes | selected robot name |
| `assets.mujoco_model_path` | string | yes | path under project root |
| `can.allowed_interfaces` | list[string] | yes | interface allowlist for simulation/hardware validation |
| `actuators[].name` | string | yes | unique actuator name |
| `actuators[].driver` | string | yes | must exist in `can_device_config.yaml.drivers` |
| `actuators[].can_id` | int/string hex | yes | unique actuator CAN ID |
| `actuators[].mujoco_joint` | string | yes | joint name |
| `actuators[].mujoco_actuator` | string | yes | actuator name |
| `actuators[].sign` | float | yes | must not be zero |
| `actuators[].offset_rad` | float radians | yes | calibration offset |

## can_device_config.yaml

| Key | Type | Required | Notes |
| --- | --- | --- | --- |
| `imu.type` | string | yes | currently `e2box` only |
| `imu.*_id` | int/string hex | yes | E2BOX CAN IDs |
| `imu.cmd_get_*` | int/string hex | yes | E2BOX request command bytes |
| `imu.quat_scale` | float | yes | must be > 0 |
| `imu.gyro_scale` | float | yes | must be > 0 |
| `imu.normalize_quat` | bool | yes | normalize decoded quaternion |
| `drivers.spg_mit.*` | float | yes | MIT ranges, current scale, and zero hold |

## processes.yaml

`processes[].env` was renamed to `processes[].env_vars` and is rejected.

| Key | Type | Required |
| --- | --- | --- |
| `processes[].name` | string | yes |
| `processes[].command` | list[string] | yes |
| `processes[].start_order` | int | yes |
| `processes[].stop_order` | int | yes |
| `processes[].new_terminal` | bool | yes |
| `processes[].terminal_command` | list[string] | yes |
| `processes[].working_dir` | string | yes |
| `processes[].env_vars` | mapping | yes |

## dashboard.yaml

Removed keys are rejected: `platform_config`, `robot_controller_config`,
`dashboard.state_hz`, `dashboard.transmit_ids`, and
`safety.allow_direct_can_transmit`. Raw CAN TX and direct actuator
enable/disable/MIT polling endpoints return `410 Gone`.

| Key | Type | Required | Notes |
| --- | --- | --- | --- |
| `dashboard.host` | string | yes | bind host |
| `dashboard.port` | int | yes | bind port |
| `dashboard.state_update_rate` | float Hz | yes | websocket publish rate |
| `robot_controller_state.enabled` | bool | yes | read controller/dashboard SHM |
| `robot_controller_state.stale_timeout_s` | float | yes | stale threshold |
| `can_monitor.*` | mapping | yes | dashboard CAN monitor windows |
| `can_daemon.connect_timeout_s` | float | yes | dashboard CAN daemon client timeout |
| `spg_monitor.default_mit_poll_hz` | float | yes | retained monitor setting |
| `zero_set_presets` | list | no | actuator-name based zero-set presets |

## Validation

YAML loading and app assembly live in `robot_controller/config/`. SHM
validation lives in `robot_controller/shm/config.py`, process validation lives
in `robot_controller/supervisor/config.py`, platform validation lives in
`robot_controller/platform/config.py`, and runtime CAN validation lives in
`robot_controller/config/can.py`. Cross-component and runtime safety checks
remain in `robot_controller/config/validation.py`.
