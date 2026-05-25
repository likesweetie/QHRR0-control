# CAN Interface

근거 파일: `config/app_config/platform.yaml`, `config/app_config/robot_controller.yaml`, `robot_controller/process/can_daemon/main.py`, `robot_controller/hardware/*`, `robot_controller/QHRR0_HW/*`, `robot_controller/process/dashboard/backend/can_decode.py`, `robot_controller/process/dashboard/backend/command_api.py`.

## CAN 기본값

| 항목 | 값 |
| --- | --- |
| interface | `vcan0` |
| bitrate | `1000000` |
| CAN type | Classical CAN payload `<= 8` bytes |
| ID type | Runtime TX는 standard 11-bit only (`0x000`-`0x7FF`) |
| CAN daemon IPC | Unix socket `/tmp/qhrr_can_daemon.sock` |

UNKNOWN: 실제 hardware interface 이름과 OS-level bitrate setup command.

## Device / CAN ID Table

| Device | Role | CAN ID | Source |
| --- | --- | --- | --- |
| E2Box IMU request | TX | `0x221` | `platform.imu.request_id` |
| E2Box quaternion | RX | `0x2A1` | `platform.imu.quat_id` |
| E2Box gyro | RX | `0x321` | `platform.imu.gyro_id` |
| `RL_hip_roll` | TX/RX actuator | `0x141` | `platform.actuators[]` |
| `RL_hip_pitch` | TX/RX actuator | `0x142` | `platform.actuators[]` |
| `RL_knee_pitch` | TX/RX actuator | `0x143` | `platform.actuators[]` |

## E2Box IMU Payload

| Frame | CAN ID | Payload | Unit/scale |
| --- | --- | --- | --- |
| request quat | `0x221` | `01` | command byte |
| request gyro | `0x221` | `02` | command byte |
| request all | `0x221` | `03` | command byte |
| quaternion | `0x2A1` | `<hhhh`: `qz_raw`, `qy_raw`, `qx_raw`, `qw_raw` | divide by `quat_scale=10000.0`, then `qx=-qx_raw/scale`, optional normalize |
| gyro | `0x321` | `<hhhh`: `gx_raw`, `gy_raw`, `gz_raw`, reserved | divide by `gyro_scale=100.0` deg/s, convert rad/s, swap x/y |

## SPG MIT Actuator Payload

Opcodes:

| Opcode | Name | Payload |
| --- | --- | --- |
| `0xC0` | `CMD_MIT_CONTROL` | MIT packed command/status |
| `0xC1` | `CMD_MIT_ENTER` | `C1 00 00 00 00 00 00 00` |
| `0xC2` | `CMD_MIT_EXIT` | `C2 00 00 00 00 00 00 00` |
| `0xC3` | `CMD_MIT_SET_ZERO` | `C3 00 00 00 00 00 <offset_i16_le>` |
| `0x90` | `CMD_READ_ENCODER_DATA` | `90 00 00 00 00 00 00 00` |
| `0x19` | `CMD_WRITE_CURRENT_POS_AS_ZERO` | `19 00 00 00 00 00 00 00` |
| `0x91` | `CMD_WRITE_ENCODER_OFFSET` | `91 00 00 00 00 00 <offset_u16_le>` |

MIT command packing:

| Byte(s) | Field |
| --- | --- |
| `0` | `0xC0` |
| `1:2` | position uint16, mapped from `[-p_max, p_max]` |
| `3` and high nibble `4` | velocity uint12, mapped from `[-v_max, v_max]` |
| low nibble `4` and `5` | kp uint12, mapped from `[0, kp_max]` |
| `6` | kd uint8, mapped from `[0, kd_max]` |
| `7` | torque uint8, mapped from `[-tau_max, tau_max]` |

Robot controller MIT protocol range is derived from `config/app_config/platform.yaml` `spg_mit`.
This is a payload quantization range, not a safety envelope:

| Field | Value |
| --- | --- |
| `position_rad` | `12.5` |
| `velocity_rad_s` | `45.0` |
| `kp` | `500.0` |
| `kd` | `5.0` |
| `torque_ff_nm` | `33.0` |
| `feedback_position_rad` | `12.56` |

## CAN Daemon JSON IPC

Client hello:

```json
{"type":"hello","role":"tx"}
```

TX request:

```json
{"type":"tx","can_id":321,"data":"c100000000000000"}
```

TX response:

```json
{"type":"tx_result","ok":true}
```

RX broadcast:

```json
{"type":"rx","can_id":321,"data":"c100000000000000"}
```

## Timeout

| Timeout | Value |
| --- | --- |
| runtime command/feedback timeout | `can.command_timeout_s: 0.05` |
| CAN daemon RX timeout | `can.daemon.rx_timeout_s: 0.002` |
| CAN daemon TX timeout | `can.daemon.tx_timeout_s: 0.002` |
| CAN daemon client connect timeout | `can.daemon.connect_timeout_s: 5.0` |
| dashboard CAN daemon connect timeout | `can_daemon.connect_timeout_s: 1.0` |

## Bus-off 처리

UNKNOWN: bus-off detection/recovery state machine은 확인되지 않았다. SocketCAN/CAN daemon error는 exception/log로 드러난다.

## candump/cansend 디버깅

```bash
candump -td vcan0
cansend vcan0 221#03
cansend vcan0 141#C100000000000000
cansend vcan0 141#C200000000000000
```

주의: `C1`은 MIT enter, `C2`는 MIT exit이다. 실제 로봇에서는 command 전송 전 물리 안전 절차를 먼저 확인한다.

MuJoCo SPG MIT simulation driver는 `C1` enter 직후 zero torque command를 latch한다. 이때 angle zero reference는 바꾸지 않으므로 feedback angle 초기값은 MuJoCo qpos에 `config/app_config/platform.yaml`의 actuator별 `sign`, `offset_rad`를 적용한 값이다. 이후에는 최신 `C0` MIT control RX packet이 다음 `C0` 또는 `C2` exit/reset 전까지 계속 적용된다.

## vcan 기반 테스트

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
python3 -m robot_controller.process.can_daemon.main --config config/app_config/robot_controller.yaml --replace-existing-socket
```

다른 터미널:

```bash
candump -td vcan0
```

## 검증 필요 항목

| 항목 | 질문 |
| --- | --- |
| firmware payload | TODO(owner): 실제 SPG firmware MIT packing limit와 `platform.spg_mit` 값 일치 여부 |
| bus-off | TODO(owner): bus-off recovery 운영 절차 |
| RX/TX same CAN ID | TODO(owner): actuator TX/RX가 같은 CAN ID인 설계가 firmware와 일치하는지 확인 |
| dashboard raw socket | TODO(owner): dashboard raw SocketCAN monitor가 local TX echo를 어떻게 표시해야 하는지 결정 |
