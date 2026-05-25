from __future__ import annotations

import logging
import time

from .command import CommandReadResult, CommandReadStatus, CommandValidator, ShmPolicyCommandSource
from .control_loop import ControlTickResult, RobotControlLoop
from .core.config import RobotControllerConfig
from .core.state import RobotControllerState
from .hardware import RobotHardware
from .processes import ChildProcessManager
from .safety import OperatorCommandShmSource, SafetyController
from .state import StatePublisher
from .utils.shm_manager import ShmManager


logger = logging.getLogger(__name__)


class RobotController:
    def __init__(self, config: RobotControllerConfig):
        self.config = config
        self.state = RobotControllerState.CREATED
        self.shm_manager = ShmManager(config.shm)
        self.processes = ChildProcessManager(config.processes)
        self.policy_command_source = ShmPolicyCommandSource(config.shm.mit_command)
        self.operator_command_source = OperatorCommandShmSource(config.shm.operator_command.name)
        self.command_validator = CommandValidator(
            expected_can_ids=config.can.motors.can_ids,
            protocol_range=config.can.mit_protocol_range,
        )
        self.hardware = RobotHardware(config)
        self.safety = SafetyController(config)
        self.state_publisher = StatePublisher(
            config,
            process_status_provider=self.processes.status,
        )
        self.control_loop = RobotControlLoop(
            hardware=self.hardware,
            policy_command_source=self.policy_command_source,
            operator_command_source=self.operator_command_source,
            command_validator=self.command_validator,
            safety=self.safety,
            state_publisher=self.state_publisher,
            processes=self.processes,
        )
        self._running = False

    def start(self) -> None:
        try:
            self.state = RobotControllerState.INIT_SHM
            if self.config.shm.cleanup_stale_on_start:
                self.shm_manager.cleanup_stale()
            self.shm_manager.create_all()

            self.state = RobotControllerState.START_CAN_DAEMON
            self.processes.start_by_name("can_daemon")
            self.hardware.connect_can_daemon()

            self.state = RobotControllerState.BRINGUP_IMU
            self.hardware.imu.bringup()

            self.state = RobotControllerState.BRINGUP_MOTORS
            if self.config.runtime.mode == "simulation":
                self.hardware.motors.bringup()

            self.state = RobotControllerState.START_CHILD_PROCESSES
            self.processes.start_all()

            self.state = RobotControllerState.RUNNING
            startup_decision = self.safety.start()
            self._running = True
            self.state_publisher.publish(
                feedback=self.hardware.read_feedback(),
                command=self.policy_command_source.read_latest(),
                controller_state=self.state,
                safety_state=self.safety.state,
                decision=startup_decision,
                force=True,
            )
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

            self.run_once()

            next_t += control_period_s
            now = time.perf_counter()
            if next_t < now:
                next_t = now + control_period_s

    def run_once(self) -> ControlTickResult:
        return self.control_loop.run_once(self.state)

    def request_stop(self) -> None:
        self._running = False

    def shutdown(self) -> None:
        if self.state == RobotControllerState.STOPPED:
            return

        self.state = RobotControllerState.SHUTTING_DOWN
        self._running = False

        shutdown_decision = self.safety.begin_shutdown()
        try:
            if self.hardware.status().can_connected:
                self.hardware.motors.shutdown("controller shutdown")
        except Exception as exc:
            logger.warning("Actuator shutdown warning: %s", exc)

        self.processes.stop_all(self.config.robot_controller.shutdown_timeout_s)
        try:
            self.state_publisher.publish(
                feedback=self.hardware.read_feedback(),
                command=self._read_command_for_shutdown(),
                controller_state=self.state,
                safety_state=self.safety.state,
                decision=shutdown_decision,
                force=True,
            )
        except FileNotFoundError as exc:
            logger.warning("Robot state SHM was not available during shutdown: %s", exc)

        self.state_publisher.close()
        self.hardware.close()
        self.policy_command_source.close()
        self.operator_command_source.close()
        self.shm_manager.close_all()
        if self.config.shm.unlink_on_shutdown:
            self.shm_manager.unlink_all()

        self.state = RobotControllerState.STOPPED
        self.safety.stop()

    def get_actuator_states(self):
        return self.hardware.motors.get_states()

    def get_imu_state(self):
        return self.hardware.imu.get_state()

    def _read_command_for_shutdown(self) -> CommandReadResult:
        try:
            return self.policy_command_source.read_latest()
        except Exception as exc:
            return CommandReadResult(
                command=None,
                status=CommandReadStatus.BAD_FORMAT,
                reason=str(exc),
                timestamp=None,
            )
