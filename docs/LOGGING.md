# Logging

## Directory

`ProcessSupervisor` creates a run-specific directory:

```text
log/<YYYYMMDD_HHMMSS>/
```

Each managed child process gets one file:

| Process | Log file |
| --- | --- |
| `can_daemon` | `log/<run>/can_daemon.log` |
| `dashboard` | `log/<run>/dashboard.log` |
| `aux_reader` | `log/<run>/aux_reader.log` |
| `task_controller` | `log/<run>/task_controller.log` |

## New Terminal Behavior

When `new_terminal: true`, the launch shell prints:

```text
[process_supervisor] log file: <path>
```

Then stdout/stderr is appended to the same file via `tee`.

## What To Check

| Symptom | File / command |
| --- | --- |
| CAN IPC connect failure | `log/<run>/can_daemon.log` |
| policy not producing commands | `log/<run>/task_controller.log` |
| dashboard not updating | `log/<run>/dashboard.log` |
| joystick/aux not available | `log/<run>/aux_reader.log` |
| raw CAN traffic | `candump -td <interface>` |

## Timestamp Basis

SHM state uses both monotonic and unix timestamps in `RobotStateC`. Process log lines are plain stdout/stderr and inherit each process's own print/log formatting.
