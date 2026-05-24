from __future__ import annotations

import logging
import time

from .core.config import RobotControllerConfig
from .core.state import RobotControllerState
from .runtime_io import RobotControllerRuntimeIO
from .utils.process_supervisor import ProcessSupervisor
from .utils.shm_command_router import ShmMitCommandRouter
from .utils.shm_manager import ShmManager


logger = logging.getLogger(__name__)


class RobotController:
    def __init__(self, config: RobotControllerConfig):
        self.config = config
        self.state = RobotControllerState.CREATED
        self.shm_manager = ShmManager(config.shm)
        self.process_supervisor = ProcessSupervisor(config.processes)
        self.command_router = ShmMitCommandRouter(config.shm.mit_command)
        self.runtime_io = RobotControllerRuntimeIO(config, self.process_supervisor)
        self._running = False

    def start(self) -> None:
        try:
            self.state = RobotControllerState.INIT_SHM
            if self.config.shm.cleanup_stale_on_start:
                self.shm_manager.cleanup_stale()
            self.shm_manager.create_all()

            self.state = RobotControllerState.START_CAN_DAEMON
            self.process_supervisor.start_by_name("can_daemon")
            self.runtime_io.connect_can_daemon()

            self.state = RobotControllerState.BRINGUP_IMU
            self.runtime_io.bringup_imu()

            self.state = RobotControllerState.BRINGUP_MOTORS
            self.runtime_io.bringup_motors()

            self.state = RobotControllerState.START_CHILD_PROCESSES
            self.process_supervisor.start_all()

            self.state = RobotControllerState.RUNNING
            self._running = True
            self.runtime_io.publish_states(self.state)
        except Exception:
            self.state = RobotControllerState.ERROR
            self.shutdown()
            raise

    def run(self) -> None:
        control_period_s = 1.0 / float(self.config.robot_controller.control_hz)
        next_t = time.perf_counter()

        while self._running:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)
            if not self._running:
                break

            self.runtime_io.request_imu_on_tick()

            batch = self.command_router.read_latest_batch()
            if batch is None:
                self.runtime_io.send_damping_once("no MIT command batch available")
            elif not self.command_router.is_fresh(batch, self.config.can.command_timeout_s):
                self.runtime_io.send_damping_once(
                    f"MIT command batch is stale: source={batch.source}"
                )
            else:
                self.runtime_io.validate_mit_batch(batch)
                self.runtime_io.mark_mit_command_active()
                self.runtime_io.send_mit_batch(batch)

            self.runtime_io.publish_due_states(self.state)

            next_t += control_period_s
            now = time.perf_counter()
            if next_t < now:
                next_t = now + control_period_s

    def request_stop(self) -> None:
        self._running = False

    def shutdown(self) -> None:
        if self.state == RobotControllerState.STOPPED:
            return

        self.state = RobotControllerState.SHUTTING_DOWN
        self._running = False

        self.runtime_io.shutdown_actuators()
        self.process_supervisor.stop_all(self.config.robot_controller.shutdown_timeout_s)
        try:
            self.runtime_io.publish_states(self.state)
        except FileNotFoundError as exc:
            logger.warning("Robot state SHM was not available during shutdown: %s", exc)

        self.runtime_io.close()
        self.command_router.close()
        self.shm_manager.close_all()
        if self.config.shm.unlink_on_shutdown:
            self.shm_manager.unlink_all()

        self.state = RobotControllerState.STOPPED

    def get_actuator_states(self):
        return self.runtime_io.get_actuator_states()

    def get_imu_state(self):
        return self.runtime_io.get_imu_state()
