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

5. In real mode, missing critical config is fatal.

6. Incomplete MIT command batch is invalid.

7. Previous command must never be held indefinitely.

## Project Guidance

- Prefer explicit configuration over inferred defaults for safety-critical
  identifiers and protocol layouts.
- Treat missing actuator CAN IDs, unknown command sources, malformed SHM
  headers, and partial MIT target batches as errors.
- When command freshness expires, transition to an explicit safe command such
  as damping or shutdown behavior. Do not continue replaying the last command.
- Any degraded behavior must be observable through logs, state, or an error
  surface appropriate to the component.
- Test and simulation modes may use explicit mock configuration, but must not
  depend on hidden inference that would be unsafe in real mode.
