# Changelog

## [Unreleased]

### Added

| 항목 | 내용 |
| --- | --- |
| Handoff docs | `docs/HANDOFF.md`, `ARCHITECTURE.md`, `RUNBOOK.md`, `SAFETY.md`, `CONTROL_LOOP.md`, `IPC_SHM.md`, `CAN_INTERFACE.md`, `CONFIG_SCHEMA.md`, `LOGGING.md`, `TEST_PLAN.md` 추가 |
| Runtime diagrams | process structure, data flow, startup, shutdown, fault transition, timing diagram 추가 |
| Safety tables | fallback trigger/action/recovery/log/test case 정리 |
| Config schema | `config/app_config/*.yaml` key, type, required, safety-critical 여부 정리 |
| Hardware-oriented controller modules | `hardware/`, `command/`, `safety/`, `state/`, `processes/` 패키지 추가 |
| SafetyController | `SafetyState`, `ControlAction`, `SafetyDecision` 추가 |
| Main-loop tests | config safety gate, command validator, safety decision, SHM command source, `run_once()` tests 추가 |

### Changed

| 항목 | 내용 |
| --- | --- |
| Documentation source policy | project handoff 문서는 README가 아니라 코드와 config 기준으로 작성 |
| Control loop structure | `RobotController.run_once()`가 `read -> decide -> act -> publish` 흐름을 드러내도록 변경 |
| Runtime config | `runtime.mode`, `hardware.*`, `safety.*` config 추가 |

### Fixed

| 항목 | 내용 |
| --- | --- |
| UNKNOWN handling | 코드에서 확인되지 않은 hardware/safety/dependency 항목을 `UNKNOWN` 또는 `TODO(owner)`로 분리 |
| Hardware gate | simulation real CAN, hardware vcan, hardware missing CLI confirmation reject 추가 |

### Removed

| 항목 | 내용 |
| --- | --- |
| Project README dependency | handoff 문서에서 project README 기반 설명 제거 |
| RuntimeIO | `RobotControllerRuntimeIO` 제거, 책임을 `MotorBus`, `ImuBus`, `StatePublisher`, `CommandValidator`, `SafetyController`로 분리 |

### Safety

| 항목 | 내용 |
| --- | --- |
| Fallback visibility | silent fallback 금지 원칙과 현재 확인된 fallback을 표로 문서화 |
| Critical warnings | 실제 로봇 실행 전 확인해야 할 CAN ID, interface, actuator enable, timeout, MIT limit 항목 강조 |
| Enable on start | `can.motors.enter_on_start` 기본값을 `false`로 변경하고 simulation/hardware startup gate에서 금지 |

## 검증 필요 항목

| 항목 | 질문 |
| --- | --- |
| release history | TODO(owner): 최초 release tag/버전 정책 확정 |
| migration notes | TODO(owner): `mujoco-QHRR` legacy tree 제거 후 migration changelog 정리 |
