# QHRR Robot State

Web-based SocketCAN dashboard for monitoring QHRR CAN traffic.

## Features

- Real-time CAN bus load, RX/TX rate, estimated kbps, and total counters
- CAN node table with heartbeat Hz, last seen age, and timeout status
- Robot State panel with E2Box IMU quaternion, projected gravity, gyro, 3D attitude preview, and simplified MJCF kinematic view
- SPG/MIT actuator status table for motor nodes
- Up to three enabled motors summarized below Robot State with angle dials
- Raw CAN transmit panel
- Per-actuator OK-gated MIT polling command: q=0, qd=0, kp=0, kd=0.5, tff=0
- TX safety lock enabled by default
- Motor enter/exit/zero endpoints gated by `allow_motor_commands`

## Layout

```text
Dashboard/
├── README.md
├── requirements.txt
├── config.yaml
├── backend/
│   ├── app.py
│   ├── bus_load.py
│   ├── can_decode.py
│   ├── command_api.py
│   ├── socketcan_io.py
│   └── state.py
└── frontend/
    ├── index.html
    ├── app.js
    └── style.css
```

The backend owns SocketCAN I/O, CAN decode, node freshness, bus-load calculation, and transmit safety. The frontend only renders snapshots from `/ws/state` and calls command endpoints.

## Install

From the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r Dashboard/requirements.txt
```

## Prepare vcan0

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

if ! ip link show vcan0 > /dev/null 2>&1; then
  sudo ip link add dev vcan0 type vcan
fi

sudo ip link set up vcan0
```

## Run

```bash
python3 -m Dashboard.backend.app
```

Open:

```text
http://127.0.0.1:8000
```

The server still starts if `vcan0` or `can0` is not available. The dashboard will show the SocketCAN connection error and retry in the background.

## Configuration

Edit `Dashboard/config.yaml`.

Important defaults:

```yaml
can:
  iface: vcan0
  bitrate: 1000000
  node_timeout_s: 0.25
  actuators:
    - name: RL_hip_roll
      motor_id: 0
      can_id: 0x141

dashboard:
  transmit_ids:
    - label: E2Box request
      can_id: 0x221
      payload: "03"
    - label: RL hip roll
      can_id: 0x141
      payload: "C100000000000000"

safety:
  tx_enabled_by_default: false
  allow_motor_commands: false
```

For real robot CAN, keep TX locked by default and leave `allow_motor_commands: false` unless you are intentionally testing command transmission.

## Command Safety

- TX starts locked.
- Raw frame send, IMU polling, and motor commands require TX unlock.
- Unlock is a single click in the dashboard.
- Each motor row has `Enable`, `Zero set`, and `MIT Poll` buttons.
- Motor enable/zero-set/MIT polling commands also require `allow_motor_commands: true`.
- MIT polling is controlled per motor row and requires one extra OK confirmation to start.
- Free-form MIT control (`0xC0`) is not exposed; the dashboard only provides the fixed confirm-gated polling command above.

## Quick Checks

Send example traffic:

```bash
cansend vcan0 141#C100000000000000
cansend vcan0 2A1#0000000000001027
cansend vcan0 321#0000000000000000
```

Expected dashboard updates:

- Node heartbeat rows become online.
- Motor 0 shows `MIT_ENTER_ACK`.
- Robot State IMU counters and decoded values update.
- CAN load bar increases with traffic rate.
