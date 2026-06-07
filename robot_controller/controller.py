from __future__ import annotations

import logging
import math
import time

from hal.can_bus.process_transport import CANProcessTransport
from hal.hardware.can.actuator.driver import ActuatorDriver
from hal.hardware.can.imu.driver import IMUDriver
from qhrr0_hw.actuators import SPGMITConfig, create_spg_actuator_driver
from qhrr0_hw.imu import E2BoxIMUProtocol
from qhrr0_hw.robot_spec import QHRR0RobotSpec, robot_spec_from_platform

from robot_controller.config import RobotControllerConfig
from robot_controller.core.state import RobotControllerState
from robot_controller.shm import (
    OPERATOR_ZERO_TARGET_MAGIC,
    COMMAND_OUTPUT_SOURCE_VALUES,
    MAX_ROBOT_STATE_ACTUATORS,
    ControlCommandC,
    ControlCommandShm,
    CommandTargetStateC,
    OperatorCommandC,
    OperatorCommandCode,
    OperatorCommandShm,
    RobotStateC,
    RobotStateShm,
)
from robot_controller.state_machine import ControllerMode, ControllerStateMachine
from robot_controller.supervisor import ProcessSupervisor
from robot_controller.shm.manager import ShmManager


logger = logging.getLogger(__name__)


class RobotController:
    def __init__(self, config: RobotControllerConfig):
        self.config = config
        self.controller_state = RobotControllerState.CREATED

        self.robot_spec: QHRR0RobotSpec = robot_spec_from_platform(config.platform)

        self.can = CANProcessTransport(
            socket_path=config.can.daemon.ipc_socket_path,
            connect_timeout_s=config.can.daemon.connect_timeout_s,
        )

        ################################################################
        #actuator bring up
        protocol_range = self.config.can.mit_protocol_range
        iq_full_scale_count = float(self.config.platform.spg_mit.iq_full_scale_count)
        if iq_full_scale_count <= 0.0:
            raise ValueError("spg_mit.iq_full_scale_count must be positive")
        iq_count_to_amp = (
            float(self.config.platform.spg_mit.iq_full_scale_current_a)
            / iq_full_scale_count
        )
        mit_config = SPGMITConfig(
            p_max=protocol_range.position_rad,
            v_max=protocol_range.velocity_rad_s,
            kp_max=protocol_range.kp,
            kd_max=protocol_range.kd,
            tau_max=protocol_range.torque_ff_nm,
            feedback_position_max=protocol_range.feedback_position_rad,
        )
        self.actuators = {
            spec.can_id: create_spg_actuator_driver(
                spec,
                mit_config=mit_config,
                feedback_timeout_s=self.config.can.command_timeout_s,
                feedback_speed_is_motor_side=True,
                iq_count_to_amp=iq_count_to_amp,
            )
            for spec in self.robot_spec.actuators
        }
        ################################################################

        ################################################################
        #IMU bring up
        spec = self.robot_spec.imu
        self.imu = IMUDriver(
            name=spec.name,
            protocol=E2BoxIMUProtocol(
                request_id=spec.request_id,
                quat_id=spec.quat_id,
                gyro_id=spec.gyro_id,
                cmd_get_quat=spec.cmd_get_quat,
                cmd_get_gyro=spec.cmd_get_gyro,
                cmd_get_all=spec.cmd_get_all,
                quat_scale=spec.quat_scale,
                gyro_scale=spec.gyro_scale,
                normalize_quat=spec.normalize_quat,
            ),
            quat_timeout=self.config.can.command_timeout_s,
            gyro_timeout=self.config.can.command_timeout_s,
        )
        ################################################################

        ################################################################
        #shm bring up
        self.shm_manager = ShmManager(config.shm)
        self.control_cmd_shm: ControlCommandShm | None = None
        self.operator_cmd_shm: OperatorCommandShm | None = None
        self.control_state_shm: RobotStateShm | None = None
        self.dashboard_state_shm: RobotStateShm | None = None
        self.state_machine = ControllerStateMachine(
            enable_duration_s=config.state_machine.enable_duration_s,
        )
        ################################################################


        self.processes_supervisor = ProcessSupervisor(config.processes)


        self._last_output_source = "NONE"
        self._last_output_t = 0.0
        self._last_output_targets: tuple[CommandTargetStateC, ...] = ()
        self._control_state_publish_period_s = 1.0 / float(config.shm.control_state.publish_hz)
        self._dashboard_state_publish_period_s = 1.0 / float(config.shm.dashboard_state.publish_hz)
        self._last_control_state_publish_t = 0.0
        self._last_dashboard_state_publish_t = 0.0
        self._last_consumed_operator_timestamp_ns: int | None = None
        self._last_ignored_operator_timestamp_ns: int | None = None
        self._running = False

    def start(self) -> None:
        try:
            self.controller_state = RobotControllerState.INIT_SHM
            if self.config.shm.cleanup_stale_on_start:
                self.shm_manager.cleanup_stale()
            self.shm_manager.create_all()
            self.control_cmd_shm = ControlCommandShm.open_reader(self.config.shm.mit_command.name)
            self.operator_cmd_shm = OperatorCommandShm.open_reader(self.config.shm.operator_command.name)
            self.control_state_shm = RobotStateShm.open_writer(self.config.shm.control_state.name)
            self.dashboard_state_shm = RobotStateShm.open_writer(self.config.shm.dashboard_state.name)

            self.controller_state = RobotControllerState.START_CAN_DAEMON
            self.processes_supervisor.start_by_name("can_daemon")
            self.can.connect()
            self._register_callbacks()

            #bringup_imu()
            self.controller_state = RobotControllerState.BRINGUP_IMU
            if not self.config.can.imu.enabled:
                return
            if not self.config.can.imu.request_all_on_start:
                return
            for _ in range(int(self.config.can.imu.startup_request_count)):
                self.can.send_frame(self.imu.make_request_all_frame())
                time.sleep(self.config.can.imu.startup_request_delay_s)

            self.controller_state = RobotControllerState.START_CHILD_PROCESSES
            self.processes_supervisor.start_all()

            self.controller_state = RobotControllerState.RUNNING
            self._running = True
            self._publish_state(force=True)
        except Exception:
            self.controller_state = RobotControllerState.ERROR
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
            self.tick()
            next_t += control_period_s
            now = time.perf_counter()
            if next_t < now:
                next_t = now + control_period_s

    def tick(self) -> RobotStateC:
        now = time.monotonic()
        
        #request_imu_on_tick
        if self.config.can.imu.enabled and self.config.can.imu.request_all_each_tick:
            self.can.send_frame(self.imu.make_request_all_frame())

        assert self.operator_cmd_shm is not None
        op = self.operator_cmd_shm.read_relaxed()
        op = self._consume_operator_command(op)
        mode = self.state_machine.update(op, now)

        if mode == ControllerMode.ESTOP:
            self._send_disable_all()
            return self._publish_state()

        if mode == ControllerMode.DISABLED:
            self._send_disable_all()
            return self._publish_state()

        if mode == ControllerMode.ENABLING:
            self._send_enable_all()
            return self._publish_state()

        if mode == ControllerMode.ZERO_SETTING:
            self._send_zero_set_all(op)
            return self._publish_state()

        if mode == ControllerMode.DAMPING:
            self._send_damping_all()
            return self._publish_state()

        if mode == ControllerMode.NORMAL:
            assert self.control_cmd_shm is not None
            cmd = self.control_cmd_shm.read_relaxed()
            self._send_policy_command(cmd)
            return self._publish_state()

        raise RuntimeError(f"Unhandled controller mode: {mode}")

    def request_stop(self) -> None:
        self._running = False

    def shutdown(self) -> None:
        if self.controller_state == RobotControllerState.STOPPED:
            return
        self._running = False
        self.controller_state = RobotControllerState.SHUTTING_DOWN
        try:
            if self.can.is_connected():
                self._send_disable_all()
        except Exception as exc:
            logger.warning("Actuator shutdown warning: %s", exc)
        self.processes_supervisor.stop_all(self.config.robot_controller.shutdown_timeout_s)
        self._publish_state(force=True)

        #close_runtime
        for item in (
            self.control_state_shm,
            self.dashboard_state_shm,
            self.control_cmd_shm,
            self.operator_cmd_shm,
        ):
            if item is not None:
                item.close()
        self.can.close()
        self.shm_manager.close_all()

        if self.config.shm.unlink_on_shutdown:
            self.shm_manager.unlink_all()
        self.controller_state = RobotControllerState.STOPPED




    def _register_callbacks(self) -> None:
        for driver in self.actuators.values():
            for can_id in driver.rx_can_ids():
                self.can.register_callback(
                    can_id,
                    lambda frame, driver=driver: self._on_actuator_frame(driver, frame),
                )
        for can_id in self.imu.rx_can_ids():
            self.can.register_callback(can_id, self.imu.on_frame)

    def _on_actuator_frame(self, driver: ActuatorDriver, frame) -> None:
        if self.can.is_recent_tx_echo(frame):
            return
        driver.on_frame(frame)

    def _consume_operator_command(self, command: OperatorCommandC) -> OperatorCommandC:
        try:
            code = OperatorCommandCode(int(command.command))
        except ValueError:
            return command
        if code == OperatorCommandCode.NONE:
            return command

        timestamp_ns = int(command.timestamp_ns)
        if timestamp_ns == self._last_consumed_operator_timestamp_ns:
            if timestamp_ns != self._last_ignored_operator_timestamp_ns:
                logger.warning(
                    "Ignoring already-consumed operator command: code=%s timestamp_ns=%d",
                    code.name,
                    timestamp_ns,
                )
                self._last_ignored_operator_timestamp_ns = timestamp_ns
            released = OperatorCommandC()
            released.timestamp_ns = command.timestamp_ns
            released.command = int(OperatorCommandCode.NONE)
            return released

        self._last_consumed_operator_timestamp_ns = timestamp_ns
        self._last_ignored_operator_timestamp_ns = None
        return command



    def _request_imu_on_tick(self) -> None:
        if self.config.can.imu.enabled and self.config.can.imu.request_all_each_tick:
            self.can.send_frame(self.imu.make_request_all_frame())

    def _send_disable_all(self) -> None:
        self._record_output_command("DISABLE", ())
        for actuator in self.actuators.values():
            self.can.send_frame(actuator.make_disable_frame())

    def _send_enable_all(self) -> None:
        self._record_output_command("ENABLE", ())
        for actuator in self.actuators.values():
            self.can.send_frame(actuator.make_enable_frame())

    def _send_zero_set_all(self, command: OperatorCommandC | None = None) -> None:
        offsets_by_can_id = self._zero_set_offsets_by_can_id(command)
        target_can_ids = sorted(offsets_by_can_id) if offsets_by_can_id else sorted(self.actuators)
        self._record_output_command("ZERO_SET", ())
        for can_id in target_can_ids:
            actuator = self.actuators.get(can_id)
            if actuator is None:
                logger.warning("Skipping zero-set for unknown actuator CAN ID 0x%03X", can_id)
                continue
            offset_deg = float(offsets_by_can_id.get(can_id, 0)) * 0.01
            self.can.send_frame(actuator.make_zero_position_frame(offset_deg=offset_deg))

    @staticmethod
    def _zero_set_offsets_by_can_id(command: OperatorCommandC | None) -> dict[int, int]:
        if command is None:
            return {}
        count = int(command.zero_target_count)
        if count == 0:
            return {}
        if int(command.zero_target_magic) != OPERATOR_ZERO_TARGET_MAGIC:
            raise RuntimeError("Malformed ZERO_SET operator command: invalid zero target magic")
        if count > len(command.zero_targets):
            raise RuntimeError(
                f"Malformed ZERO_SET operator command: target count {count} exceeds "
                f"capacity {len(command.zero_targets)}"
            )
        offsets: dict[int, int] = {}
        for index in range(count):
            target = command.zero_targets[index]
            can_id = int(target.can_id)
            if can_id == 0:
                continue
            offsets[can_id] = int(target.offset_count)
        return offsets

    def _send_damping_all(self) -> None:
        kd = float(self.config.safety.velocity_damping_kd)
        targets = tuple(
            _command_target_state(
                can_id=can_id,
                p_target_rad=0.0,
                v_target_rad_s=0.0,
                kp=0.0,
                kd=kd,
                tau_target_nm=0.0,
            )
            for can_id in sorted(self.actuators)
        )
        self._record_output_command("DAMPING", targets)
        for can_id in sorted(self.actuators):
            actuator = self.actuators[can_id]
            self.can.send_frame(
                actuator.make_impedance_command_frame(
                    position_rad=0.0,
                    velocity_rad_s=0.0,
                    kp=0.0,
                    kd=kd,
                    torque_ff_nm=0.0,
                )
            )

    def _send_policy_command(self, cmd: ControlCommandC) -> None:
        n = min(int(cmd.num_targets), len(cmd.targets))
        targets: list[CommandTargetStateC] = []
        for index in range(n):
            target = cmd.targets[index]
            actuator = self.actuators.get(int(target.can_id))
            if actuator is None:
                continue
            command_target = _command_target_state(
                can_id=int(target.can_id),
                p_target_rad=float(target.q),
                v_target_rad_s=float(target.dq),
                kp=float(target.kp),
                kd=float(target.kd),
                tau_target_nm=float(target.tau),
            )
            targets.append(command_target)
            self.can.send_frame(
                actuator.make_impedance_command_frame(
                    position_rad=command_target.p_target_rad,
                    velocity_rad_s=command_target.v_target_rad_s,
                    kp=command_target.kp,
                    kd=command_target.kd,
                    torque_ff_nm=command_target.tau_target_nm,
                )
            )
        self._record_output_command("POLICY", tuple(targets))

    def _record_output_command(
        self,
        source: str,
        targets: tuple[CommandTargetStateC, ...],
    ) -> None:
        self._last_output_source = source
        self._last_output_t = time.monotonic()
        self._last_output_targets = targets

    def _publish_state(self, *, force: bool = False) -> RobotStateC:
        state = self._build_robot_state_c()
        now = time.monotonic()
        if (
            self.control_state_shm is not None
            and (force or now - self._last_control_state_publish_t >= self._control_state_publish_period_s)
        ):
            self.control_state_shm.write(state)
            self._last_control_state_publish_t = now
        if (
            self.dashboard_state_shm is not None
            and (force or now - self._last_dashboard_state_publish_t >= self._dashboard_state_publish_period_s)
        ):
            self.dashboard_state_shm.write(state)
            self._last_dashboard_state_publish_t = now
        return state

    def _build_robot_state_c(self) -> RobotStateC:
        now = time.monotonic()
        self.imu.update_fault_flags()
        imu_state = self.imu.get_state()
        imu_comm = self.imu.get_comm_status()

        robot_state = RobotStateC()
        robot_state.timestamp_ns = time.time_ns()
        robot_state.timestamp_monotonic = now
        robot_state.timestamp_unix = time.time()
        robot_state.controller_mode = int(self.state_machine.mode)
        robot_state.actuator_count = min(len(self.actuators), MAX_ROBOT_STATE_ACTUATORS)

        _fill_float_array(
            robot_state.imu.quat_wxyz,
            _quat_xyzw_to_wxyz(imu_state.quat_xyzw),
            (1.0, 0.0, 0.0, 0.0),
        )
        _fill_float_array(
            robot_state.imu.projected_gravity_b,
            getattr(imu_state, "projected_gravity_b", None),
            (0.0, 0.0, -1.0),
        )
        _fill_float_array(
            robot_state.imu.angular_velocity_rad_s,
            imu_state.angular_velocity_rad_s,
            (0.0, 0.0, 0.0),
        )
        robot_state.imu.last_quat_t = float(imu_state.last_quat_t)
        robot_state.imu.last_gyro_t = float(imu_state.last_gyro_t)
        robot_state.imu.quat_online = int(bool(imu_comm["quat"].is_online))
        robot_state.imu.gyro_online = int(bool(imu_comm["gyro"].is_online))
        robot_state.imu.quat_stale = int(bool(imu_comm["quat"].is_stale))
        robot_state.imu.gyro_stale = int(bool(imu_comm["gyro"].is_stale))

        for index, (can_id, driver) in enumerate(sorted(self.actuators.items())):
            if index >= MAX_ROBOT_STATE_ACTUATORS:
                break
            actuator_state = driver.get_state()
            comm = driver.update_fault_flags()
            age_s = (
                None
                if actuator_state.last_feedback_t <= 0.0
                else max(0.0, now - actuator_state.last_feedback_t)
            )
            out = robot_state.actuators[index]
            out.can_id = int(can_id)
            out.position_rad = _float_or_zero(actuator_state.position_rad)
            out.velocity_rad_s = _float_or_zero(actuator_state.velocity_rad_s)
            out.torque_nm = _float_or_zero(actuator_state.torque_nm)
            out.current_a = _float_or_zero(actuator_state.current_a)
            out.temperature_c = _float_or_zero(actuator_state.temperature_c)
            out.fault_code = -1 if actuator_state.fault_code is None else int(actuator_state.fault_code)
            out.is_enabled = -1 if actuator_state.is_enabled is None else int(bool(actuator_state.is_enabled))
            out.last_feedback_t = float(actuator_state.last_feedback_t)
            out.age_s = -1.0 if age_s is None else float(age_s)
            out.online = int(bool(comm.is_online))
            out.stale = int(bool(comm.is_stale))

        command_output = robot_state.command_output
        command_output.timestamp_monotonic = float(self._last_output_t)
        command_output.source = int(COMMAND_OUTPUT_SOURCE_VALUES.get(self._last_output_source, 0))
        command_output.target_count = min(len(self._last_output_targets), MAX_ROBOT_STATE_ACTUATORS)
        for index, item in enumerate(self._last_output_targets[:MAX_ROBOT_STATE_ACTUATORS]):
            out = command_output.targets[index]
            out.can_id = int(item.can_id)
            out.p_target_rad = _float_or_zero(item.p_target_rad)
            out.v_target_rad_s = _float_or_zero(item.v_target_rad_s)
            out.kp = _float_or_zero(item.kp)
            out.kd = _float_or_zero(item.kd)
            out.tau_target_nm = _float_or_zero(item.tau_target_nm)

        return robot_state



def _quat_xyzw_to_wxyz(
    quat_xyzw: tuple[float, float, float, float] | None,
) -> tuple[float, float, float, float] | None:
    if quat_xyzw is None:
        return None
    qx, qy, qz, qw = quat_xyzw
    return qw, qx, qy, qz


def _command_target_state(
    *,
    can_id: int,
    p_target_rad: float,
    v_target_rad_s: float,
    kp: float,
    kd: float,
    tau_target_nm: float,
) -> CommandTargetStateC:
    target = CommandTargetStateC()
    target.can_id = int(can_id)
    target.p_target_rad = float(p_target_rad)
    target.v_target_rad_s = float(v_target_rad_s)
    target.kp = float(kp)
    target.kd = float(kd)
    target.tau_target_nm = float(tau_target_nm)
    return target


def _fill_float_array(target, values, fallback) -> None:
    source = fallback if values is None else values
    for index, value in enumerate(source):
        target[index] = float(value)


def _float_or_zero(value: float | None) -> float:
    if value is None or not math.isfinite(float(value)):
        return 0.0
    return float(value)
