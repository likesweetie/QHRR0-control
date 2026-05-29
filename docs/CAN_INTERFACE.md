# CAN Interface

CAN runtime is split between product-independent HAL transport and QHRR0-specific protocol implementation.

## Config Source

| Value | Source |
| --- | --- |
| CAN interface | `config/app_config/platform.yaml: can.interface` |
| bitrate | `config/app_config/platform.yaml: can.bitrate` |
| daemon socket | `config/app_config/platform.yaml: can.daemon_socket` |
| actuator CAN IDs | `config/app_config/platform.yaml: actuators[].can_id` |
| IMU IDs | `config/app_config/platform.yaml: imu.*_id` |

## Layers

| Layer | File |
| --- | --- |
| CAN frame/daemon/dispatcher | `hal/can_bus/` |
| CAN process client/transport | `hal/can_bus/process_client.py`, `hal/can_bus/process_transport.py` |
| actuator base driver/protocol | `hal/hardware/can/actuator/` |
| IMU base driver/protocol | `hal/hardware/can/imu/` |
| SPG/DongilC protocol | `qhrr0_hw/actuators/dongilc_protocol.py` |
| E2BOX protocol | `qhrr0_hw/imu/e2box_protocol.py` |

## Current Device IDs

From `config/app_config/platform.yaml`:

| Device | CAN ID |
| --- | --- |
| `RL_hip_roll` | `0x141` |
| `RL_hip_pitch` | `0x142` |
| `RL_knee_pitch` | `0x143` |
| IMU request | `0x221` |
| IMU quat | `0x2A1` |
| IMU gyro | `0x321` |

## Debug Commands

```bash
candump -td vcan0
cansend vcan0 221#03
python3 -m robot_controller.subprocesses.can_daemon.main --config config/app_config/robot_controller.yaml --replace-existing-socket
```

## Notes

- Controller actuator output goes through `RobotController._send_*` methods and HAL `CANProcessTransport`.
- Dashboard raw transmit can send arbitrary frames through the daemon, but operator mode commands should go through `OperatorCommandShm`.
- Feedback TX echo filtering is handled in `RobotController._on_actuator_frame()` through `CANProcessTransport.is_recent_tx_echo()`.
