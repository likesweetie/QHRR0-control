# TODO: SHM Refactor Follow-ups

This file tracks only SHM refactor items that are not finished yet or still need
extra validation.

## Backend Consistency Policy

- Decide per-channel backend policy before enabling consistency protocols.
  Current creator/reader/writer paths use the default `PlainBuffer`.
- If `SeqLockBuffer` or `DoubleBuffer` is enabled for any channel, wire backend
  selection through config or `ShmManager` so every process agrees on the same
  backend and segment size.
- Update config/docs when non-plain backends are enabled. Existing
  `shm.*.size_bytes` values are allocation sizes; non-plain backends require at
  least `segment_size_bytes()`.
- Add compatibility checks before mixed backends can be used in production. A
  reader using the wrong backend will interpret the segment bytes incorrectly.
- Keep `read_relaxed()` available for low-overhead callers. Treat
  `read(consistent=True)` as opt-in and valid only for backends that support
  `read_consistent()`.

## SHM Metadata

- Consider adding a SHM header with magic, schema/version, backend id, payload
  size, segment size, and optional writer timestamp/sequence fields.
- Add header validation on create/open before production use of non-plain
  backends.

## Import Compatibility

- `ctypes` structures and thin SHM wrappers now live under
  `robot_controller/shm/types/`.
- In-repo imports have been updated, and package-root re-exports are preserved.
- Check whether external code imports old direct modules such as
  `robot_controller.shm.robot_state` or `robot_controller.shm.control_command`.
  If so, either update those callers or add temporary compatibility shims.
- Consider a temporary `robot_controller/shm/cstruct.py` compatibility shim only
  if external code imports `robot_controller.shm.cstruct`.

## Tests To Add

- Dashboard reader tests for
  `robot_controller/subprocesses/dashboard/backend/robot_state_shm.py`,
  especially `robot_state_to_dict()` and `_read_channel()`.
- Controller tests that assert `tick()` / `run_once()` return `RobotStateC` and
  that rate-limited control/dashboard `RobotStateShm` writes preserve dashboard
  behavior.
- Task controller / joint initializer tests that assert policy input
  construction, actuator lookup, `quat_wxyz` convention, and command
  publication.
