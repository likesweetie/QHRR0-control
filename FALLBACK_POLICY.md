# Fallback Policy

This policy applies to the entire QHRR control project, including dashboard,
robot controller, HAL integration, legacy tools, and MuJoCo CAN bridge code.

Fallback behavior is safety-critical. A fallback must never hide ambiguity,
configuration errors, transport failures, or stale command state.

## Rules

1. Silent fallback is forbidden.

2. Any fallback must either:
   - transition to a safer state, or
   - raise a fatal error.

3. Any fallback that changes runtime behavior must log a reason.

4. No fallback may infer motor ID, CAN ID, SHM layout, or command source.

5. In hardware mode, missing critical config or missing CLI confirmation is fatal.

6. Incomplete MIT command batch is invalid.

7. Previous command must never be held indefinitely.

## Project Guidance

- Prefer explicit configuration over inferred defaults for safety-critical
  identifiers and protocol layouts.
- Treat missing actuator CAN IDs, unknown command sources, malformed SHM
  headers, and partial MIT target batches as errors.
- When command freshness expires, transition through `SafetyController` to an
  explicit `ControlAction`. Do not continue replaying the last command.
- Command missing/stale/read-collision must be represented as a
  `CommandReadResult` with status and reason.
- Invalid command values must be rejected by `CommandValidator`; do not clip
  silently.
- Feedback stale, CAN daemon death, and task controller death must be visible
  to `SafetyController` through `RobotFeedback` and `ProcessHealth`.
- The current damping fallback is a MIT velocity damping-like command:
  `q=0`, `qd=0`, `kp=0`, `kd=safety.velocity_damping_kd`, `tau=0`.
  It must not be named `safe_damping` unless actuator firmware behavior is
  verified.
- Hardware mode must not be reachable by YAML changes alone. It requires
  explicit CLI flags and hardware gate validation.
- Any degraded behavior must be observable through logs, state, or an error
  surface appropriate to the component.
- Test and simulation modes may use explicit mock configuration, but must not
  depend on hidden inference that would be unsafe in real mode.
