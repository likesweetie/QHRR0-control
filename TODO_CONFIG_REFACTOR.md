# TODO: Config Refactor Follow-ups

This file tracks the remaining work after the first pass that split the
previous central `robot_controller/core/config.py`.

## Moved Config Ownership

- `robot_controller/config/loader.py`
  - YAML load, path resolution, and raw mapping helper functions.
- `robot_controller/config/app.py`
  - `RobotControllerConfig` top-level assembly.
  - `RuntimeModeConfig`, `HardwareSafetyConfig`, `SafetyPolicyConfig`,
    `StateMachineConfig`, and `RobotControllerCoreConfig`.
- `robot_controller/config/validation.py`
  - Cross-component validation and runtime hardware safety checks.
- `robot_controller/platform/config.py`
  - Platform config, robot asset paths, actuator mapping, platform CAN names,
    platform SHM names, platform IMU IDs, and MIT protocol range source values.
- `robot_controller/config/can.py`
  - Runtime CAN config, CAN daemon options, motor startup flags, IMU polling
    options, and MIT protocol range config derived from platform data.
- `robot_controller/shm/config.py`
  - `ShmConfig` and SHM channel config parsing/local validation.
- `robot_controller/supervisor/config.py`
  - `ProcessConfig` parsing/local validation for supervised child processes.

## Compatibility Still Present

- `robot_controller/core/config.py` is now a compatibility re-export layer.
- `robot_controller/core/platform_config.py` is now a compatibility re-export
  layer.
- `robot_controller.config.validate_hardware_safety` re-exports
  `HardwareSafetyOptions` and `validate_runtime_safety` for the documented
  safety path.

## Raw Dict Consumers Still To Refactor

- `robot_controller/subprocesses/dashboard/backend/app.py`
  - Still builds and passes a dashboard config dictionary.
  - Follow-up: create `robot_controller/subprocesses/dashboard/config.py` with
    typed dashboard config objects and local validation for host/port/state_hz,
    transmit IDs, zero-set presets, CAN monitor options, and safety toggles.
- `run_mujoco_simulation.py`
  - Still reads `mujoco.yaml` as a raw mapping.
  - Follow-up: create a small typed MuJoCo launcher config parser.
- `robot_controller/subprocesses/task_controller/policy_runner.py`
  - Still reads policy YAML dictionaries directly.
  - Follow-up: introduce typed policy config objects near the policy runner.
- `robot_controller/subprocesses/joint_initializer/policy_runner.py`
  - Same policy config follow-up as task controller.

## Global Validation Kept Central

- Runtime mode validity.
- Runtime hardware safety gate.
- SHM target count vs enabled CAN motor count.
- Safety damping gain vs MIT protocol kd range.
- Required `can_daemon` process presence.

## Follow-up Tests

- Add tests for `robot_controller.config.app.load_robot_controller_config()`
  using the new module paths.
- Add tests for local validation in `robot_controller/platform/config.py`,
  `robot_controller/config/can.py`, `robot_controller/shm/config.py`, and
  `robot_controller/supervisor/config.py`.
- Add dashboard config parser tests after dashboard raw dict usage is replaced.
- Add policy config parser tests after policy YAML dictionaries are replaced.

## Principle

YAML structure changes should be absorbed in loader/parser modules. Runtime
consumers should receive typed config objects, not raw YAML dictionaries.
