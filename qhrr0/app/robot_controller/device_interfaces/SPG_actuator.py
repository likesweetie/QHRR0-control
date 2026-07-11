class SPGActuatorDevice:
    def __init__(
        self,
        *,
        name: str,
        can_id: int,
        driver: SPGActuatorDriver,
        can_client: CANClient,
        stale_timeout_s: float,
    ) -> None:
        self._name = name
        self._can_id = _can_id

        self._driver = driver
        self._can_client = can_client
        self._stale_timeout_s = stale_timeout_s

        self._state = ActuatorState()
        self._last_valid_rx_t = 0.0
        self._rx_count = 0
        self._decode_error_count = 0

        self._can_client.register_callback(
            feedback_id,
            self._handle_frame,
        )