# QHRR0 Refactor Handoff

## 0. Purpose

This document hands over the current QHRR0 refactor direction to a Codex agent.

The current project is not being refactored as a generic multi-robot framework. It is being organized as a **QHRR0-specific runtime project**. Therefore, the package root represents the QHRR0 system itself.

The key architectural intent is:

- `qhrr0/main.py` is the top-level runtime composition root.
- `qhrr0/qhrr0_spec.py` defines the canonical static QHRR0 object specification.
- `config/` contains external YAML inputs consumed during the factory/build stage.
- `factory/` validates inputs and emits implementation-agnostic build specs or pure data objects.
- `factory/` must not import `app/` runtime implementations.
- `app/` contains actual runtime implementations: HAL, controller, shared memory, subprocesses, dashboard, etc.
- The current runtime `RobotController(config)` implementation remains legacy-config-based for this refactor pass.
- Do not rewrite `app/robot_controller/controller.py` to remove config in this pass.
- Validation terminology must use `validation_rules`, not `validation_policy`, to avoid semantic conflict with RL/control policy terminology.

This refactor target is an intermediate architecture:

```text
config + qhrr0_spec
  ↓
factory validation/build stage
  ↓
current legacy RobotController(config)
```

It is not yet the final target:

```text
config + qhrr0_spec
  ↓
factory
  ↓
config-free runtime controller
```

The second target can be done later after the runtime controller is explicitly redesigned.

---

## 1. Current project structure

Current intended package root:

```text
qhrr0/
├── app/
│   ├── hal/
│   └── robot_controller/
├── config/
│   ├── app_config/
│   ├── config_paths.yaml
│   ├── policy_config/
│   └── robot_config/
├── factory/
│   ├── app_factory/
│   │   ├── app_factory.py
│   │   └── app_validation_rules.py
│   ├── policy_factory/
│   └── robot_factory/
│       ├── base/
│       │   └── robot_base.py
│       ├── robot_factory.py
│       └── robot_validation_rules.py
├── main.py
├── policy/
│   └── qhrr/
│       ├── policy.onnx
│       └── qhrr
└── qhrr0_spec.py
```

Keep this structure for now.

Important design decisions:

1. `main.py` stays at the package root.
2. `qhrr0_spec.py` stays at the package root.
3. Do not move `qhrr0_spec.py` into a generic `robots/` directory in this pass.
4. `policy/` remains a placeholder until the semantic role of policy objects is finalized.
5. `policy_factory/` remains a placeholder unless an actual policy object model is explicitly introduced.

---

## 2. Responsibility map

### 2.1 `qhrr0/main.py`

`main.py` is the **QHRR0 runtime composition root**.

It owns:

- command-line parsing
- config loading
- safety option construction
- QHRR0 static specification import
- app factory invocation
- runtime object creation
- signal handling
- start/run/shutdown lifecycle

`main.py` may import both `factory` and `app` because it is the top-level composition layer.

Expected dependency direction:

```text
qhrr0/main.py
  ├── qhrr0_spec
  ├── factory
  └── app runtime implementations
```

This is allowed.

### 2.2 `qhrr0/qhrr0_spec.py`

`qhrr0_spec.py` is the **canonical QHRR0 static object manifest**.

It should define the static QHRR0 robot object and its static controller specification using `RobotFactory`.

It should contain things like:

- `QHRR0_CONTROLLER`
- `QHRR0`

It should not:

- start processes
- open CAN sockets
- create SHM
- instantiate runtime `RobotController`
- load YAML
- perform runtime setup

### 2.3 `factory/robot_factory/base/robot_base.py`

This module defines pure static data objects only.

Expected classes:

- `Actuator`
- `IMU`
- `RobotController`
- `Robot`

Do not add:

- `__post_init__`
- `validate()`
- runtime state
- sockets
- SHM handles
- process handles
- ONNX sessions
- lookup helpers
- type conversion logic

Validation belongs in `robot_validation_rules.py`.

### 2.4 `factory/robot_factory/robot_factory.py`

This module creates pure data objects and runs robot validation rules.

It may import:

```python
from .robot_validation_rules import validate_robot_by_rules
```

It must not import `app`.

### 2.5 `factory/robot_factory/robot_validation_rules.py`

This module owns static robot validation rules only.

It validates:

- robot identity
- supported CAN interfaces
- controller static description
- actuator definitions
- IMU definition
- CAN ID conflicts
- allowed actuator driver names
- QHRR0 single-leg 3-DoF layout
- controller assumption that QHRR0 has 3 actuators

It must not validate:

- runtime mode
- process command
- SHM size
- hardware safety CLI flags
- CAN daemon timeout
- MIT protocol range
- YAML path existence

Those belong to app/config validation rules.

### 2.6 `factory/app_factory/app_factory.py`

This module consumes `config` and `Robot` during the build stage.

It should:

1. Build an `AppValidationContext`.
2. Run app validation rules.
3. Emit implementation-agnostic build specs.

It must not:

- instantiate `RobotController`
- instantiate `CANProcessTransport`
- instantiate `ShmManager`
- instantiate `ProcessSupervisor`
- instantiate `ControlModeFsm`
- import HAL runtime driver implementations
- import app runtime implementation classes

The factory layer must not depend on `app`.

### 2.7 `factory/app_factory/app_validation_rules.py`

This module validates app/config/runtime safety inputs at build time.

It should validate:

- runtime mode
- CAN interface supported by the static robot definition
- safety config
- robot controller config
- state machine config
- safety vs MIT protocol range consistency
- CAN config
- MIT protocol range
- process configs
- SHM configs
- hardware safety options

It should not validate robot static structure. That belongs in `robot_validation_rules.py`.

### 2.8 `app/`

`app/` contains actual runtime implementations.

It may import `factory`, but `factory` must not import `app`.

`app/robot_controller/controller.py` is still legacy-config-based in this pass. Leave it mostly untouched.

---

## 3. Dependency rules

### 3.1 Allowed

```text
main.py → qhrr0_spec.py
main.py → factory
main.py → app
app → factory
app → config loader
factory → pure data classes
factory → validation rules
```

### 3.2 Forbidden

```text
factory → app
robot_factory → app.robot_controller.controller
app_factory → app.robot_controller.controller
app_factory → CANProcessTransport
app_factory → ShmManager
app_factory → ProcessSupervisor
app_factory → HAL driver implementations
```

Do not add imports like this inside `qhrr0/factory/**`:

```python
from qhrr0.app.robot_controller.controller import RobotController
from qhrr0.app.hal.can_bus.process_transport import CANProcessTransport
from qhrr0.app.robot_controller.shm.manager import ShmManager
from qhrr0.app.robot_controller.process_supervisor import ProcessSupervisor
```

Those imports are allowed only in `main.py` or inside `app/`.

---

## 4. Naming rules

### 4.1 Use `validation_rules`, not `validation_policy`

Use:

```text
robot_validation_rules.py
app_validation_rules.py
validation rule
required_validation_rules
validate_*_by_rules
```

Do not use:

```text
validation_policy.py
validation policy
required_validations
validate_*_by_policy
```

Reason: the word `policy` is reserved for RL/control policy semantics in this robotics codebase.

### 4.2 Use `can_interfaces`, not `allowed_can_interfaces`

Use:

```python
can_interfaces
```

Do not use:

```python
allowed_can_interfaces
```

Reason: `can_interfaces` is now a static robot definition field, not a direct mirror of legacy YAML `allowed_interfaces`.

### 4.3 Use `qhrr0_spec.py`, not `robots/qhrr0.py`

This project is QHRR0-specific. `qhrr0_spec.py` is a root-level specification manifest for this system.

Do not move it into `robots/` unless the project later becomes a multi-robot framework.

---

## 5. Required `robot_base.py` target shape

`factory/robot_factory/base/robot_base.py` should approximately match:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, TypeAlias


RobotName: TypeAlias = str
RobotRevision: TypeAlias = str

ActuatorName: TypeAlias = str
DriverName: TypeAlias = str
CanInterfaceName: TypeAlias = str
ImuName: TypeAlias = str

ControllerName: TypeAlias = str
ControllerType: TypeAlias = str
ValidationRuleName: TypeAlias = str


@dataclass(frozen=True, slots=True, kw_only=True)
class Actuator:
    name: ActuatorName
    driver: DriverName
    can_id: int
    sign: float
    offset_rad: float
    group: str = ""


@dataclass(frozen=True, slots=True, kw_only=True)
class IMU:
    name: ImuName
    driver: DriverName
    can_id: int


@dataclass(frozen=True, slots=True, kw_only=True)
class RobotController:
    name: ControllerName
    controller_type: ControllerType
    required_validation_rules: tuple[ValidationRuleName, ...] = ()
    metadata: dict[str, Any] | None = None
    description: str = ""


@dataclass(frozen=True, slots=True, kw_only=True)
class Robot:
    name: RobotName
    actuators: tuple[Actuator, ...]
    imu: IMU
    controller: RobotController
    can_interfaces: tuple[CanInterfaceName, ...] = ()
    required_validation_rules: tuple[ValidationRuleName, ...] = ()
    description: str = ""
```

---

## 6. Required `robot_validation_rules.py` behavior

Expected rule registry API:

```python
RobotValidationRule = Callable[[Robot], None]
_ROBOT_VALIDATION_RULES: dict[str, RobotValidationRule] = {}

def robot_validation_rule(name: str) -> Callable[[RobotValidationRule], RobotValidationRule]: ...
def validate_robot_by_rules(robot: Robot) -> None: ...
def collect_required_validation_rules(robot: Robot) -> tuple[str, ...]: ...
def list_robot_validation_rules() -> tuple[str, ...]: ...
```

Required rule names:

```text
robot_identity
can_interfaces
controller
actuators
imu
can_id_conflicts
actuator_drivers_known
single_leg_3dof_layout
policy_controller_requires_3_actuators
```

Rule `single_leg_3dof_layout` should require exactly these actuator groups:

```python
{"hip_roll", "hip_pitch", "knee_pitch"}
```

Rule `policy_controller_requires_3_actuators` should require:

```python
len(robot.actuators) == 3
```

Rule `actuator_drivers_known` should read:

```python
robot.controller.metadata["allowed_actuator_drivers"]
```

Expected metadata form:

```python
{
    "allowed_actuator_drivers": ("spg_mit",)
}
```

---

## 7. Required `robot_factory.py` behavior

`factory/robot_factory/robot_factory.py` should:

- construct `Actuator`
- construct `IMU`
- construct static `RobotController`
- construct `Robot`
- call `validate_robot_by_rules(robot)` after creating `Robot`

It must not import app runtime modules.

Expected import:

```python
from .robot_validation_rules import (
    require_can_interface_supported,
    validate_robot_by_rules,
)
```

Expected method parameter names:

```python
required_validation_rules
can_interfaces
```

---

## 8. Required `qhrr0_spec.py` behavior

`qhrr0_spec.py` should define the canonical QHRR0 object.

Target conceptual content:

```python
from __future__ import annotations

from qhrr0.factory.robot_factory.robot_factory import RobotFactory


_factory = RobotFactory()


QHRR0_CONTROLLER = _factory.create_controller(
    name="qhrr0_policy_controller",
    controller_type="policy_runner",
    required_validation_rules=(
        "controller",
        "actuator_drivers_known",
        "single_leg_3dof_layout",
        "policy_controller_requires_3_actuators",
    ),
    metadata={
        "allowed_actuator_drivers": ("spg_mit",),
        "controlled_actuators": (
            "RL_hip_roll",
            "RL_hip_pitch",
            "RL_knee_pitch",
        ),
        "action_dim": 3,
    },
    description="Static controller profile for QHRR0 one-leg policy runner.",
)


QHRR0 = _factory.create_robot(
    name="qhrr0",
    description="QHRR0 single-leg robot definition.",
    can_interfaces=("vcan0", "can0"),
    required_validation_rules=(
        "robot_identity",
        "can_interfaces",
        "actuators",
        "imu",
        "can_id_conflicts",
    ),
    controller=QHRR0_CONTROLLER,
    imu=_factory.create_imu(
        name="e2box_imu",
        driver="e2box",
        can_id=0x221,
    ),
    actuators=(
        _factory.create_actuator(
            name="RL_hip_roll",
            driver="spg_mit",
            can_id=0x141,
            sign=1.0,
            offset_rad=0.0,
            group="hip_roll",
        ),
        _factory.create_actuator(
            name="RL_hip_pitch",
            driver="spg_mit",
            can_id=0x142,
            sign=1.0,
            offset_rad=0.0,
            group="hip_pitch",
        ),
        _factory.create_actuator(
            name="RL_knee_pitch",
            driver="spg_mit",
            can_id=0x143,
            sign=1.0,
            offset_rad=0.0,
            group="knee_pitch",
        ),
    ),
)


__all__ = [
    "QHRR0",
    "QHRR0_CONTROLLER",
]
```

Do not load YAML here.

Do not instantiate runtime objects here.

---

## 9. Required `app_factory.py` behavior

`factory/app_factory/app_factory.py` should define build-stage dataclasses only.

Recommended dataclasses:

- `HardwareSafetyOptions`
- `ActuatorBuildSpec`
- `ImuBuildSpec`
- `ControllerBuildSpec`
- `ProcessBuildSpec`
- `AppBuildSpec`

`AppFactory.create_app_spec(...)` should:

1. receive `robot`, `config`, `options`
2. build `AppValidationContext`
3. run `validate_app_by_rules(...)`
4. return `AppBuildSpec`

It must not create actual runtime app objects.

The returned `AppBuildSpec` may be unused by the current runtime controller.

That is intentional.

---

## 10. Required `app_validation_rules.py` behavior

Expected registry API:

```python
AppValidationRule = Callable[[AppValidationContext], None]
_APP_VALIDATION_RULES: dict[str, AppValidationRule] = {}

def app_validation_rule(name: str) -> Callable[[AppValidationRule], AppValidationRule]: ...
def validate_app_by_rules(ctx: AppValidationContext, required_validation_rules: Iterable[str]) -> None: ...
def list_app_validation_rules() -> tuple[str, ...]: ...
```

Required rule names:

```text
runtime_mode
can_interface_supported_by_robot
safety_config
robot_controller_config
state_machine_config
safety_vs_protocol_range
can_config
mit_protocol_range
processes
shm
hardware_safety
```

The rule `can_interface_supported_by_robot` should call:

```python
require_can_interface_supported(ctx.robot, str(ctx.config.can.interface))
```

Do not duplicate robot static validation here.

---

## 11. Required `main.py` behavior

`qhrr0/main.py` should be the only real runtime entrypoint for the QHRR0 system.

It should:

1. parse CLI arguments
2. load config
3. create `HardwareSafetyOptions`
4. call `AppFactory().create_app_spec(...)`
5. instantiate current legacy `RobotController(config)`
6. install signal handlers
7. run start/run/shutdown lifecycle

Expected sequence:

```python
config = load_robot_controller_config(args.config)

safety_options = HardwareSafetyOptions(
    hardware_requested=bool(args.hardware),
    motor_enable_confirmed=bool(args.i_understand_this_can_enable_motors),
    estop_ok=bool(args.estop_ok),
)

_app_spec = AppFactory().create_app_spec(
    robot=QHRR0,
    config=config,
    options=safety_options,
)

controller = RobotController(config)
```

Important:

- `_app_spec` may be unused for runtime construction now.
- Do not construct `RobotController` from `AppBuildSpec` in this pass.
- Do not remove `config` from `RobotController` in this pass.

`app/robot_controller/main.py` should either be removed as an entrypoint or turned into a thin wrapper:

```python
from qhrr0.main import main

if __name__ == "__main__":
    main()
```

---

## 12. Config path rule

Since `main.py` lives at:

```text
qhrr0/main.py
```

the default config should be:

```python
DEFAULT_CONFIG = (
    Path(__file__).resolve().parent
    / "config"
    / "app_config"
    / "robot_controller.yaml"
)
```

Do not use `parents[2]` in root-level `main.py`. That was correct only when the entrypoint lived in `app/robot_controller/main.py`.

---

## 13. Package/import cleanup

Add missing package initializers as needed:

```text
qhrr0/__init__.py
qhrr0/app/__init__.py
qhrr0/app/robot_controller/__init__.py
qhrr0/factory/__init__.py
qhrr0/factory/app_factory/__init__.py
qhrr0/factory/robot_factory/__init__.py
qhrr0/factory/robot_factory/base/__init__.py
qhrr0/factory/policy_factory/__init__.py
```

Prefer package-qualified imports:

```python
from qhrr0.qhrr0_spec import QHRR0
from qhrr0.factory.app_factory.app_factory import AppFactory, HardwareSafetyOptions
from qhrr0.app.robot_controller.controller import RobotController
```

Avoid ambiguous top-level imports:

```python
from factory.robot_factory.robot_factory import RobotFactory
```

unless the project intentionally runs with `qhrr0/` as `PYTHONPATH` root. Prefer package execution:

```bash
python -m qhrr0.main
```

---

## 14. Do not do in this refactor

Do not perform the following changes in this pass:

1. Do not rewrite `app/robot_controller/controller.py` to remove config.
2. Do not make `factory` import app runtime implementation classes.
3. Do not move `qhrr0_spec.py` into `robots/`.
4. Do not finalize the semantic role of `policy_factory`.
5. Do not move IMU protocol-specific fields into `IMU`.
6. Do not merge robot validation and app validation into one file.
7. Do not reintroduce `validation_policy` naming.
8. Do not reintroduce `robot_platform.yaml` as the canonical robot definition source.
9. Do not create a generic multi-robot registry unless explicitly requested.
10. Do not make `AppFactory` the owner of runtime object lifecycles.

---

## 15. Expected final responsibility map

```text
qhrr0/main.py
  QHRR0 runtime composition root.
  Owns runtime lifecycle.

qhrr0/qhrr0_spec.py
  Canonical QHRR0 static object manifest.

factory/robot_factory/base/robot_base.py
  Pure static data classes only.

factory/robot_factory/robot_factory.py
  Creates Robot / Actuator / IMU / static RobotController data objects.
  Calls robot validation rules.

factory/robot_factory/robot_validation_rules.py
  Validates static robot definition.

factory/app_factory/app_factory.py
  Consumes config and Robot during build stage.
  Emits AppBuildSpec.
  Does not import app runtime implementations.

factory/app_factory/app_validation_rules.py
  Validates app/config/runtime safety inputs at build stage.

app/robot_controller/controller.py
  Current runtime implementation.
  Leave mostly untouched in this pass.

app/robot_controller/main.py
  Optional compatibility wrapper around qhrr0.main.
```

---

## 16. Validation commands after refactor

Run from the repository root:

```bash
python -m compileall qhrr0
```

Import the static QHRR0 spec:

```bash
python - <<'PY'
from qhrr0.qhrr0_spec import QHRR0
print(QHRR0)
PY
```

List validation rules:

```bash
python - <<'PY'
from qhrr0.factory.robot_factory.robot_validation_rules import list_robot_validation_rules
from qhrr0.factory.app_factory.app_validation_rules import list_app_validation_rules
print(list_robot_validation_rules())
print(list_app_validation_rules())
PY
```

Run the entrypoint in simulation mode:

```bash
python -m qhrr0.main --config qhrr0/config/app_config/robot_controller.yaml
```

Do not run hardware mode unless explicitly authorized.

---

## 17. Common expected failure points

### A. `required_validations` still exists

Fix all remaining occurrences to:

```python
required_validation_rules
```

### B. `allowed_can_interfaces` still exists

Fix all remaining occurrences to:

```python
can_interfaces
```

### C. Old validation names remain

Fix:

```python
validation_policy
validate_robot_by_policy
```

to:

```python
validation_rules
validate_robot_by_rules
```

### D. Factory imports app

Remove any app runtime imports from `factory/**`.

### E. Wrong `DEFAULT_CONFIG`

For root-level `qhrr0/main.py`, use:

```python
Path(__file__).resolve().parent
```

not:

```python
Path(__file__).resolve().parents[2]
```

### F. Missing `__init__.py`

If package-qualified imports fail, add missing `__init__.py` files rather than reverting to top-level ambiguous imports.

---

## 18. Final design invariant

After this pass, the project should satisfy:

```text
QHRR0 static definition is code-owned by qhrr0_spec.py.
Config is consumed only during the factory/build validation stage.
Factory does not depend on app runtime implementations.
main.py is the composition root and owns runtime lifecycle.
Current RobotController(config) remains unchanged for now.
```
