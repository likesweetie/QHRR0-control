# TODO: SHM Refactor Follow-ups

This refactor keeps `robot_controller/shm/` as the only edited code area and
does not directly modify subprocesses, telemetry, dashboard, controller, docs,
or tests outside the SHM package.

## Why external files were not edited

The current change is intentionally scoped to `robot_controller/shm/` plus this
root TODO file. Several callers outside `shm` still depend on convenience APIs
that build commands, convert button masks, convert quaternion conventions, or
turn raw ctypes state into dashboard dictionaries. Those behaviors should move
to application-level code in a follow-up change so that SHM wrappers remain raw
transport only.

## Deprecated SHM convenience API migration

- `ControlCommandShm.write_targets()` is still used by
  `robot_controller/subprocesses/task_controller/main.py`,
  `robot_controller/subprocesses/joint_initializer/main.py`, and
  `tests/test_control_command_shm.py`. Move `ControlCommandC` construction and
  timestamp assignment into the application layer, then call `write()`.
- `AuxCommandShm.publish()` is still used by
  `robot_controller/subprocesses/aux_reader/main.py`. Move `AuxCommandC`
  construction, timestamp assignment, and button mask conversion into the
  aux reader/application layer, then call `write()`.
- `OperatorCommandShm.publish()` and `publish_zero_set()` are still used through
  `OperatorCommandShmWriter` and dashboard endpoints in
  `robot_controller/subprocesses/dashboard/backend/app.py`, as well as
  `tests/test_operator_command_shm.py`. Move operator command construction,
  command-code selection, target mask handling, and zero-set target validation
  into the dashboard/operator application layer, then call `write()`.
- `RobotStateShm.read_latest()` is still used by
  `robot_controller/subprocesses/dashboard/backend/robot_state_shm.py` and
  `tests/test_robot_state_shm.py`. Move dashboard dictionary conversion out of
  the SHM wrapper and have the dashboard reader call `read_relaxed()`.

## Raw SHM field access that can use ctypes helpers

- In `robot_controller/subprocesses/task_controller/main.py`, replace the inline
  actuator dictionary built from
  `control_state.actuators[: int(control_state.actuator_count)]` with
  `RobotStateC.valid_actuators()` or `RobotStateC.actuator_by_can_id()`.
- Apply the same actuator lookup migration in
  `robot_controller/subprocesses/joint_initializer/main.py`.
- Use `RobotStateC.is_initialized()` instead of repeated
  `int(control_state.timestamp_ns) == 0` checks where possible.

## Quaternion convention cleanup

- `task_controller/main.py` and `joint_initializer/main.py` currently convert
  IMU quaternion order inline from `quat_xyzw` to policy-facing `wxyz`.
  Centralize that conversion at the application boundary and use
  `ImuStateC.quat_wxyz()` where the source is `RobotStateC.imu`.
- Keep SHM field names explicit. `quat_xyzw` remains the shared-memory layout
  field; policy code should decide whether it needs `xyzw` or `wxyz`.

## Button mask and dashboard conversion cleanup

- Move `buttons_to_mask()` and `mask_to_buttons()` to the joystick/task
  application layer after callers are updated. They are currently kept for
  compatibility.
- Move `robot_state_to_dict()` and the dashboard-specific helpers in
  `robot_controller/shm/robot_state.py` into dashboard/application code once
  `RobotStateShm.read_latest()` is retired.
- Move the compatibility `ControlTarget` dataclass out of SHM when
  `write_targets()` callers are migrated.
- Move `OperatorCommandShmWriter` out of SHM when dashboard/operator callers
  construct `OperatorCommandC` directly.

## Future SHM consistency and metadata

- Consider adding `read_consistent()` with a sequence counter or double-read
  guard for readers that cannot tolerate torn multi-field frames.
- Consider adding a small SHM header with magic, schema/version, payload size,
  and possibly writer timestamp/sequence fields.
- Keep `read_relaxed()` available for low-overhead callers that explicitly
  accept tearing.

## Tests likely affected by follow-up migrations

- `tests/test_robot_state_shm.py`
- `tests/test_control_command_shm.py`
- `tests/test_operator_command_shm.py`
- Any dashboard reader tests that cover
  `robot_controller/subprocesses/dashboard/backend/robot_state_shm.py`
- Any task controller or joint initializer tests that assert policy input
  construction, actuator lookup, quaternion convention, or command publication

## Follow-up instructions

1. First migrate external callers to build ctypes structures directly and call
   `write()`/`read_relaxed()`.
2. Then remove the deprecated compatibility methods from SHM wrappers.
3. Finally move dashboard/button/operator helper functions out of
   `robot_controller/shm/` so that the SHM package contains only ctypes layout
   definitions, thin layout helpers, and raw transport.
