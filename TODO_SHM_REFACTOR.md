# TODO: SHM Refactor Follow-ups

This file tracks SHM refactor follow-ups and the current migration state.

## Completed in current migration

- External callers now build `ControlCommandC`, `AuxCommandC`, and
  `OperatorCommandC` directly and call `write()`.
- `ControlCommandShm.write_targets()`, `AuxCommandShm.publish()`,
  `OperatorCommandShm.publish()`, `OperatorCommandShm.publish_zero_set()`, and
  `RobotStateShm.read_latest()` were removed from SHM wrappers.
- Button mask helpers moved out of SHM to
  `robot_controller/subprocesses/aux_buttons.py`.
- Dashboard operator command construction moved out of SHM to
  `robot_controller/subprocesses/dashboard/backend/operator_commands.py`.
- Dashboard robot-state dict conversion moved out of SHM to
  `robot_controller/subprocesses/dashboard/backend/robot_state_shm.py`.
- `robot_controller/telemetry/` was removed. The former `RobotSnapshot`,
  `ShmStatePublisher`, and `DashboardPublisher` layer was only an intermediate
  conversion path, not a real telemetry transport.
- `RobotController` now builds `RobotStateC` directly and writes both
  control/dashboard `RobotStateShm` channels with private rate limiting.
- `RobotController.tick()` now returns `RobotStateC` instead of `RobotSnapshot`.
- SHM and dashboard/controller policy paths now use `quat_wxyz`.
- `ControlTarget` compatibility dataclass was removed from SHM.
- Buffer backends are exported from `robot_controller.shm.__init__`.
- Unused `struct_size()` and `clear_buffer()` helpers were removed from
  `cstruct_type.py`.
- Focused buffer backend tests were added.

## Deprecated SHM convenience API migration

Complete. SHM wrappers now expose raw transport methods only.

## Current SHM update notes

- `robot_controller/shm/cstruct_type.py` is now the common ctypes SHM wrapper
  module. `robot_controller/shm/cstruct.py` no longer exists.
- `robot_controller/shm/buffer.py` introduces byte-level buffer backends:
  `PlainBuffer`, `SeqLockBuffer`, and `DoubleBuffer`.
- ctypes structures and thin SHM wrappers live under
  `robot_controller/shm/types/`.
- `CStructShm` still defaults to `PlainBuffer`, so existing SHM payload layout
  remains compatible unless callers explicitly pass a different backend.
- `size_bytes()` still means ctypes payload size. `segment_size_bytes()` now
  means the actual shared-memory segment size required by the selected backend.
- No creator/reader/writer path currently selects `SeqLockBuffer` or
  `DoubleBuffer`; `ShmManager` and application callers still use the default
  `PlainBuffer`.

## Raw SHM field access that can use ctypes helpers

Complete for `task_controller/main.py` and `joint_initializer/main.py`.

## Quaternion convention cleanup

- Project-facing controller/dashboard/SHM quaternion convention is `wxyz`.
- `RobotStateC.imu.quat_wxyz` is the shared-memory layout field.
- Hardware IMU sources that still expose `quat_xyzw` are converted once at the
  controller snapshot boundary.
- Task controller and joint initializer read `quat_wxyz` directly from SHM.

## Button mask and dashboard conversion cleanup

Complete.

## Telemetry removal notes

- Removal reason: the deleted telemetry package was not ROS2, WebSocket,
  logging, or another real transport. It only created `RobotSnapshot`
  dataclasses and converted them into `RobotStateC`, duplicating the SHM SOT.
- `snapshot_to_cstruct()` was removed. Its field population logic now lives in
  `RobotController._build_robot_state_c()`.
- `RobotStateShm` remains raw transport only; no dashboard/application
  conversion functions were added back to the wrapper.
- Dashboard functionality remains active through `RobotStateShm`; dashboard
  dict conversion currently lives in
  `robot_controller/subprocesses/dashboard/backend/robot_state_shm.py`.
- Follow-up: add focused dashboard reader tests around
  `robot_state_to_dict()` and `_read_channel()` now that conversion lives in the
  dashboard backend.
- `RobotStateShm.read_latest()` has already been removed. Future callers should
  use `read_relaxed()` or `read(consistent=True)` once a non-plain backend is
  intentionally selected.

## Buffer backend follow-ups

- Decide backend policy per channel before enabling consistency protocols.
  Likely candidates: keep command channels on `PlainBuffer`; evaluate
  `SeqLockBuffer` or `DoubleBuffer` for `RobotStateShm` control/dashboard state
  channels where torn multi-field frames matter more.
- Wire backend selection through configuration or manager construction only
  after all creator/reader/writer processes can agree on the same backend and
  segment size.
- Update config/docs if non-plain backends are enabled. Existing
  `shm.*.size_bytes` values are allocation sizes; for `SeqLockBuffer` and
  `DoubleBuffer`, the allocation must be at least `segment_size_bytes()`, not
  just `size_bytes()`.
- Add compatibility checks before mixed backends can be used in production. A
  reader opened with `PlainBuffer` against a segment created with
  `SeqLockBuffer` or `DoubleBuffer` will interpret the bytes incorrectly unless
  both sides agree out of band.
- Consider adding a small SHM header with magic, schema/version, backend id,
  payload size, segment size, and possibly writer timestamp/sequence fields.
- Keep `read_relaxed()` available for low-overhead callers that explicitly
  accept tearing. Treat `read(consistent=True)` as an opt-in path that requires
  a backend supporting `read_consistent()`.
- Consider a temporary `robot_controller/shm/cstruct.py` compatibility shim only
  if external code outside this repository imports `robot_controller.shm.cstruct`.
  Current in-repo imports use `cstruct_type.py`.

## Tests likely affected by follow-up migrations

Updated:

- `tests/test_robot_state_shm.py`
- `tests/test_control_command_shm.py`
- `tests/test_operator_command_shm.py`
- `tests/test_shm_buffer.py`
- Targeted compile/test coverage for direct `RobotStateC` construction path.

Still useful to add later:

- Dashboard reader tests for
  `robot_controller/subprocesses/dashboard/backend/robot_state_shm.py`
- Controller tests that assert `tick()` / `run_once()` return `RobotStateC`
  and that rate-limited control/dashboard writes preserve dashboard behavior.
- Task controller / joint initializer tests that assert policy input
  construction, actuator lookup, quaternion convention, and command publication

## Follow-up instructions

1. Decide per-channel backend policy and whether to enable `read_consistent()`.
2. Add SHM header/magic/version/backend metadata before production use of
   non-plain backends.
3. Update config/docs once backend selection is exposed.
