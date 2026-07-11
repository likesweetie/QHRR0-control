from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum

from robot_controller.shm.types.commands import (
    OperatorCommandC,
    OperatorCommandCode,
)


class ControllerMode(IntEnum):
    UNKNOWN = 0
    DISABLED = 1
    ENABLING = 2
    NORMAL = 3
    DAMPING = 4
    ZERO_SETTING = 5
    ESTOP = 6


@dataclass(slots=True)
class ControlModeFsm:
    """Control mode finite-state transition logic.

    Priority layers:
    1st priority:
        Safety and fault-latch handling.
        ESTOP must override every other command and must remain latched
        until RESET_FAULT is received.

    2nd priority:
        Direct operator-command transitions.
        This layer handles explicit commands such as DISABLE, ENABLE, RUN,
        DAMPING, ZERO_SET, and RESET_FAULT outside ESTOP.

    3rd priority:
        Automatic mode transitions.
        This layer handles transitions caused by elapsed time or command
        release, such as ENABLING -> DAMPING and ZERO_SETTING -> DISABLED.
    """

    enable_duration_s: float
    mode: ControllerMode = ControllerMode.DISABLED
    mode_enter_time: float = 0.0

    def __post_init__(self) -> None:
        if self.mode_enter_time <= 0.0:
            self.mode_enter_time = time.monotonic()

    def update(self, command: OperatorCommandC | None, time_now: float) -> ControllerMode:
        code = self._parse_command_code(command)

        # Priority-chain pattern:
        # Each priority layer gets a chance to handle the current command/state.
        # A layer returns ControllerMode when it consumes the condition.
        # A layer returns None when it does not handle anything.
        #
        # Do not use `if mode:` here because ControllerMode is an IntEnum and
        # some modes may have value 0. Always compare with None explicitly.
        mode = self._apply_1st_priority_safety(code, time_now)
        if mode is not None:
            return mode

        mode = self._apply_2nd_priority_operator_command(code, time_now)
        if mode is not None:
            return mode

        mode = self._apply_3rd_priority_automatic_transition(code, time_now)
        if mode is not None:
            return mode

        return self.mode

    def _apply_1st_priority_safety(
        self,
        code: OperatorCommandCode,
        time_now: float,
    ) -> ControllerMode | None:

        if code == OperatorCommandCode.ESTOP:
            return self._enter(ControllerMode.ESTOP, time_now)

        if self.mode == ControllerMode.ESTOP:
            if code == OperatorCommandCode.RESET_FAULT:
                return self._enter(ControllerMode.DISABLED, time_now)
            return self.mode

        return None
    
    def _apply_2nd_priority_operator_command(
        self,
        code: OperatorCommandCode,
        time_now: float,
    ) -> ControllerMode | None:

        match code:
            case OperatorCommandCode.DISABLE | OperatorCommandCode.RESET_FAULT:
                return self._enter(ControllerMode.DISABLED, time_now)

            case OperatorCommandCode.ENABLE:
                if self.mode in (ControllerMode.DISABLED, ControllerMode.ZERO_SETTING):
                    return self._enter(ControllerMode.ENABLING, time_now)

            case OperatorCommandCode.RUN:
                if self.mode == ControllerMode.DAMPING:
                    return self._enter(ControllerMode.NORMAL, time_now)

            case OperatorCommandCode.DAMPING:
                return self._enter(ControllerMode.DAMPING, time_now)

            case OperatorCommandCode.ZERO_SET:
                return self._enter(ControllerMode.ZERO_SETTING, time_now)

            case OperatorCommandCode.NONE:
                return None

        return None

    def _apply_3rd_priority_automatic_transition(
        self,
        code: OperatorCommandCode,
        time_now: float,
    ) -> ControllerMode | None:
        match self.mode:
            case ControllerMode.ZERO_SETTING:
                # ZERO_SET is treated as a one-shot command.
                # When the command is released, return to DISABLED.
                if code == OperatorCommandCode.NONE:
                    return self._enter(ControllerMode.DISABLED, time_now)

            case ControllerMode.ENABLING:
                # ENABLING holds the enable state for a configured duration,
                # then automatically moves to DAMPING before NORMAL is allowed.
                if self._mode_elapsed_s(time_now) >= self.enable_duration_s:
                    return self._enter(ControllerMode.DAMPING, time_now)

        return None

    @staticmethod
    def _parse_command_code(command: OperatorCommandC | None) -> OperatorCommandCode:
        if command is None:
            return OperatorCommandCode.NONE

        try:
            return OperatorCommandCode(int(command.command))
        except ValueError:
            return OperatorCommandCode.NONE

    def _mode_elapsed_s(self, time_now: float) -> float:
        return time_now - self.mode_enter_time

    def _enter(self, mode: ControllerMode, time_now: float) -> ControllerMode:
        if self.mode != mode:
            self.mode = mode
            self.mode_enter_time = time_now
        return self.mode
    

 #############################################
    # 1st-priority design pattern:
    # This layer is a safety override / fault-latch layer.
    #
    # Contract:
    # - Return ControllerMode when this layer handles the condition.
    # - Return None when this layer does not apply.
    #
    # Recommended pattern:
    #     if code == OperatorCommandCode.ESTOP:
    #         return self._enter(ControllerMode.ESTOP, time_now)
    #
    #     if self.mode == ControllerMode.ESTOP:
    #         if code == OperatorCommandCode.RESET_FAULT:
    #             return self._enter(ControllerMode.DISABLED, time_now)
    #         return self.mode
    #
    # Not recommended:
    #     match code:
    #         case OperatorCommandCode.ESTOP:
    #             ...
    #         case OperatorCommandCode.ENABLE:
    #             ...
    #
    # Reason:
    # Safety rules should stay minimal and explicit.
    # Do not mix normal operation commands such as ENABLE, RUN, DAMPING,
    # or ZERO_SET into this layer.
    #
    # Not recommended:
    #     if self.mode == ControllerMode.ESTOP:
    #         return None
    #
    # Reason:
    # Returning None would allow lower-priority layers to handle normal
    # commands while ESTOP is active. ESTOP must remain latched until
    # RESET_FAULT is explicitly received.
    #
    # Not recommended:
    #     self.mode = ControllerMode.ESTOP
    #     return self.mode
    #
    # Reason:
    # Directly assigning self.mode bypasses _enter(), so mode_enter_time
    # will not be updated consistently.

 #############################################
    # 2nd-priority design pattern:
    # This layer handles direct operator-command transitions.
    #
    # Contract:
    # - Return ControllerMode when an operator command is accepted.
    # - Return None when the command is ignored, invalid for the current mode,
    #   or not relevant.
    #
    # Recommended pattern:
    #     match code:
    #         case OperatorCommandCode.ENABLE:
    #             if self.mode in allowed_source_modes:
    #                 return self._enter(ControllerMode.ENABLING, time_now)
    #
    # Reason:
    # This layer branches mainly by command type, so match-case keeps
    # the command policy readable.
    #
    # Not recommended:
    #     if code == OperatorCommandCode.ESTOP:
    #         return self._enter(ControllerMode.ESTOP, time_now)
    #
    # Reason:
    # ESTOP belongs to 1st-priority safety logic.
    # Duplicating it here makes the priority model ambiguous.
    #
    # Not recommended:
    #     if self.mode == ControllerMode.ENABLING:
    #         if self._mode_elapsed_s(time_now) >= self.enable_duration_s:
    #             return self._enter(ControllerMode.DAMPING, time_now)
    #
    # Reason:
    # Time-based transitions belong to 3rd-priority automatic logic,
    # not direct operator-command handling.
    #
    # Not recommended:
    #     self.mode = ControllerMode.NORMAL
    #     return self.mode
    #
    # Reason:
    # Always use _enter() for mode transitions so mode_enter_time is
    # updated consistently.
    #
    # Not recommended:
    #     return self.mode
    #
    # Reason:
    # Returning self.mode means "this layer handled the condition."
    # If the command is simply not applicable, return None so the next
    # priority layer can evaluate its own rules.


 #############################################
    # 3rd-priority design pattern:
    # This layer handles automatic transitions caused by elapsed time,
    # command release, or current-mode completion.
    #
    # Contract:
    # - Return ControllerMode when an automatic transition occurs.
    # - Return None when the current mode should be maintained.
    #
    # Recommended pattern:
    #     match self.mode:
    #         case ControllerMode.ENABLING:
    #             if self._mode_elapsed_s(time_now) >= self.enable_duration_s:
    #                 return self._enter(ControllerMode.DAMPING, time_now)
    #
    # Reason:
    # This layer branches mainly by current mode, not by command type.
    #
    # Also acceptable for short conditions:
    #     match self.mode:
    #         case ControllerMode.ENABLING if self._mode_elapsed_s(time_now) >= self.enable_duration_s:
    #             return self._enter(ControllerMode.DAMPING, time_now)
    #
    # Use the longer form when the condition needs comments or multiple checks.
    #
    # Not recommended:
    #     if code == OperatorCommandCode.ENABLE:
    #         return self._enter(ControllerMode.ENABLING, time_now)
    #
    # Reason:
    # Direct operator commands belong to 2nd-priority logic.
    #
    # Not recommended:
    #     self.can.send_frame(...)
    #     actuator.make_enable_frame()
    #
    # Reason:
    # The state machine should only decide mode transitions.
    # Actuator command output belongs in controller.py loop/mode dispatch.
    #
    # Not recommended:
    #     return self.mode
    #
    # Reason:
    # Returning self.mode means "this layer handled the condition."
    # If no automatic transition occurred, return None.
