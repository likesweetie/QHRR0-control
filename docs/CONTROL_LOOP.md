# Control Loop

근거 파일: `robot_controller/control_loop.py`, `robot_controller/robot_controller.py`, `robot_controller/safety/safety_controller.py`, `robot_controller/command/command_validator.py`, `robot_controller/process/task_controller/main.py`.

## Top-level 흐름

`RobotController.run_once()`는 `RobotControlLoop.run_once()`로 위임한다. 한 tick은 다음 순서로 고정되어 있다.

```text
IMU request on tick
-> read RobotFeedback
-> read latest policy command from SHM
-> validate command
-> read ProcessHealth and HardwareStatus
-> SafetyController.evaluate()
-> act on ControlAction
-> StatePublisher.publish()
```

```mermaid
sequenceDiagram
    participant Loop as RobotControlLoop.run_once
    participant HW as RobotHardware
    participant SRC as ShmPolicyCommandSource
    participant VAL as CommandValidator
    participant SAFE as SafetyController
    participant PUB as StatePublisher

    Loop->>HW: imu.request_on_tick(now)
    Loop->>HW: read_feedback()
    Loop->>SRC: read_latest()
    Loop->>VAL: validate(command)
    Loop->>SAFE: evaluate(SafetyInputs)
    alt SEND_POLICY_COMMAND
        Loop->>HW: motors.send_policy_mit_batch()
    else SEND_DAMPING
        Loop->>HW: motors.send_velocity_damping()
    else DISABLE_MOTORS
        Loop->>HW: motors.disable_all()
    else NO_OUTPUT
        Loop->>Loop: no motor output
    end
    Loop->>PUB: publish(feedback, command, safety_state, decision)
```

## 주기

| Loop/Data | Default | Source |
| --- | --- | --- |
| Main controller tick | 500 Hz | `robot_controller.control_hz` |
| Policy inference process | 50 Hz | `config/app_config/processes.yaml` env `TASK_CONTROL_HZ`, or explicit `task_controller --control-hz` |
| Policy output rate log | 1 s interval | `config/app_config/processes.yaml` env `TASK_RATE_LOG_INTERVAL_S`; logs successful `qhrr_mit_command` publishes |
| `qhrr_control_state` publish | 500 Hz | `shm.control_state.publish_hz` |
| `qhrr_dashboard_state` publish | 10 Hz | `shm.dashboard_state.publish_hz` |
| Dashboard websocket | 24 Hz | `dashboard.state_hz` |
| IMU request | every controller tick | `can.imu.request_all_each_tick: true` |

## Command 처리

`task_controller`는 `qhrr_control_state`와 `qhrr_aux_command`를 읽고, ONNX policy output을 MIT target batch로 `qhrr_mit_command`에 쓴다.

`ShmPolicyCommandSource`는 다음만 담당한다.

| Responsibility | Behavior |
| --- | --- |
| SHM attach/read | `qhrr_mit_command` read |
| layout guard | magic/version/size/target_count 검사 |
| sequence guard | odd/even seq collision 감지 |
| read result | `CommandReadResult(status, reason, timestamp, command)` 반환 |

`CommandValidator`는 다음을 reject한다.

| Reject condition |
| --- |
| NaN/Inf |
| CAN ID order mismatch |
| duplicate CAN ID |
| unknown CAN ID |
| missing actuator command |
| position/velocity/kp/kd/torque limit 초과 |

Invalid command는 clip하지 않는다. `SafetyController`가 invalid command reason을 받아 `FAULT_LATCHED` + `DISABLE_MOTORS`로 판단한다.

## Stale/Missing Command

| Condition | Safety state/action |
| --- | --- |
| no command | `DAMPING` + `SEND_DAMPING` |
| read collision | `DAMPING` + `SEND_DAMPING` |
| stale timestamp | `DAMPING` + `SEND_DAMPING` |
| invalid command | `FAULT_LATCHED` + `DISABLE_MOTORS` |

Command loss로 `DAMPING`에 들어갔고 actuator feedback에서 `is_enabled: true`가 확인되면, controller는 매 tick damping-like MIT command를 보낸다. enabled actuator가 확인되지 않으면 command loss만으로 motor output을 보내지 않는다. Fresh valid command와 fresh feedback이 둘 다 확인되면 `RUNNING`으로 복귀한다. `safety.damping_timeout_s` 안에 회복하지 못하면 `FAULT_LATCHED`로 전환하지만, CAN daemon과 MIT-enabled actuator feedback이 살아 있으면 motor disable 대신 damping-like MIT command를 계속 보낸다.

Dashboard E-STOP은 `ESTOP` state가 아니라 operator-latched `DAMPING`으로 들어간다. 이 latch가 active인 동안 fresh policy command가 있어도 `RUNNING`으로 자동 복귀하지 않고, operator `arm` command가 들어와야 policy command를 다시 보낼 수 있다.

## Damping-like MIT Command

`MotorBus.send_velocity_damping()`은 다음 MIT command를 모든 configured CAN ID에 보낸다.

```text
q = 0
qd = 0
kp = 0
kd = safety.velocity_damping_kd
tau = 0
```

이 command는 코드에서 `safe_damping`이라고 부르지 않는다. 실제 firmware에서 hardware-safe damping으로 동작한다는 보장은 이 코드에서 확인되지 않았다.

## Policy Observation

`task_controller`의 observation 생성 순서:

1. `ControlStateReader.latest()`로 `qhrr_control_state` read
2. `AuxCommandReader.latest()`로 `qhrr_aux_command` read
3. actuator position/velocity를 configured CAN ID 순서로 추출
4. IMU `quat_xyzw`를 policy용 `quat_wxyz`로 변환
5. joystick command를 `lin_x`, `lin_y`, `yaw`, `mode`로 전달
6. `OnnxPolicy.compute_action()`
7. `q_target = policy action + action_offset(...)`
8. `ShmMitCommandWriter.publish()`로 complete MIT batch 작성

## Joint/CAN Order

현재 enabled actuator order:

| Index | Name | CAN ID |
| --- | --- | --- |
| 0 | `RL_hip_roll` | `0x141` |
| 1 | `RL_hip_pitch` | `0x142` |
| 2 | `RL_knee_pitch` | `0x143` |

`CommandValidator`는 incoming command target CAN ID order가 `config.can.motors.can_ids`와 정확히 일치해야 통과시킨다.

## 검증 필요 항목

| 항목 | 질문 |
| --- | --- |
| Damping firmware behavior | TODO(owner): q=0, qd=0, kp=0, kd=0.5, tau=0의 실제 actuator firmware 동작 |
| Damping recovery | TODO(owner): 자동 복귀 외에 operator가 damping hold를 강제로 유지/해제하는 입력 경로 |
| Policy rate | TODO(owner): 50 Hz policy와 500 Hz CAN tick의 의도된 decimation |
