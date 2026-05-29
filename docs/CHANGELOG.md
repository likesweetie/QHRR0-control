# Changelog

## [Unreleased]

### Added

- Top-level `qhrr0_hw/` package for QHRR0-specific actuator protocol, IMU protocol, robot spec, joint map, and calibration.
- HAL CAN process client/transport under `hal/can_bus/`.
- `robot_controller/shm/` ctypes C-compatible SHM views.
- `robot_controller/state_machine.py` with `ControllerMode` and `OperatorCommandCode`.
- Split telemetry publishers: `ShmStatePublisher` and `DashboardPublisher`.

### Changed

- `RobotController.tick()` now directly dispatches exactly one actuator output path per controller mode.
- Arm and policy run are split: `ENABLING` now transitions to `DAMPING`, and `RUN` is required for `NORMAL`.
- `robot_controller/process` moved to `robot_controller/subprocesses`.
- `robot_controller/processes` moved to `robot_controller/supervisor`.
- `robot_controller/state` moved to `robot_controller/telemetry`.
- Task controller now writes `ControlCommandShm` ctypes targets.
- Operator/dashboard commands now write `OperatorCommandShm`.

### Removed

- Nested `robot_controller/QHRR0_HW`.
- `robot_controller/hardware`, `robot_controller/command`, `robot_controller/safety`, and old `robot_controller/state` packages.
- Runtime command consistency counter path.

### Safety

- Hardware startup gate still rejects accidental real CAN use in simulation and requires explicit hardware flags in hardware mode.
- Motor command tearing is intentionally accepted in `ControlCommandShm`.
- `ENABLING` sends only enable commands; `NORMAL` is the only mode that reads policy command SHM.
