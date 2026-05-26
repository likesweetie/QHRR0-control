from __future__ import annotations

from hal.can_bus.process_transport import CANProcessTransport


class CanTransport(CANProcessTransport):
    def __init__(self, config):
        daemon_config = config.can.daemon
        super().__init__(
            socket_path=daemon_config.ipc_socket_path,
            connect_timeout_s=daemon_config.connect_timeout_s,
        )


__all__ = ["CanTransport"]
