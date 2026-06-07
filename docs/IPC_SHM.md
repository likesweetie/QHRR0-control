# IPC / Shared Memory

SHM is implemented as thin ctypes C-compatible views in `robot_controller/shm/`. JSON payload SHM and consistency counters are not used.

## Segments

Names come from `config/app_config/platform.yaml`.

| Segment | Writer | Reader | Purpose |
| --- | --- | --- | --- |
| `qhrr_mit_command` | `task_controller` | `RobotController` | policy actuator targets |
| `qhrr_aux_command` | `aux_reader` | `task_controller` | joystick/aux command |
| `qhrr_operator_command` | dashboard/operator process | `RobotController` | operator mode command |
| `qhrr_control_state` | `RobotController` | `task_controller` | high-rate robot state |
| `qhrr_dashboard_state` | `RobotController` | dashboard | low-rate robot state |

## ControlCommandShm

```python
class ControlTargetC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("q", ctypes.c_float),
        ("dq", ctypes.c_float),
        ("kp", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("tau", ctypes.c_float),
    ]

class ControlCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("num_targets", ctypes.c_uint32),
        ("targets", ControlTargetC * 12),
    ]
```

`read_relaxed()` returns a copy. Tearing between fields is accepted by design.

## OperatorCommandShm

```python
class OperatorZeroTargetC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("offset_count", ctypes.c_int16),
        ("reserved", ctypes.c_uint16),
    ]

class OperatorCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("command", ctypes.c_uint32),
        ("target_mask", ctypes.c_uint32),
        ("zero_target_count", ctypes.c_uint32),
        ("zero_target_magic", ctypes.c_uint32),
        ("zero_targets", OperatorZeroTargetC * 12),
    ]
```

`command` uses `OperatorCommandCode`: `NONE`, `ENABLE`, `DISABLE`, `DAMPING`, `ZERO_SET`, `ESTOP`, `RESET_FAULT`, `RUN`.
For `ZERO_SET`, `zero_targets` may carry per-actuator `can_id` plus signed int16 centidegree `offset_count`; the controller uses those offsets only when `zero_target_magic` is valid.
`timestamp_ns` is the command id for controller-side one-shot consumption; repeated non-`NONE` commands with the same timestamp are treated as `NONE`.

## RobotStateShm

`RobotStateShm` is telemetry only. It contains controller mode, IMU fields, and up to 12 actuator states. It does not store a separate safety mode.

## Lifetime Rule

Current SHM reads use `from_buffer_copy()`, so persistent ctypes buffer views are avoided. If future code uses `from_buffer()`, close must clear the view reference before `SharedMemory.close()`.
