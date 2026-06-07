from .core.state import RobotControllerState

__all__ = [
    "RobotController",
    "RobotControllerConfig",
    "RobotControllerState",
    "load_robot_controller_config",
]


def __getattr__(name: str):
    if name == "RobotController":
        from .controller import RobotController

        return RobotController
    if name == "RobotControllerConfig":
        from .config import RobotControllerConfig

        return RobotControllerConfig
    if name == "load_robot_controller_config":
        from .config import load_robot_controller_config

        return load_robot_controller_config
    raise AttributeError(name)
