# Based on the solution to the issue of not being able to log with different severity levels
# https://stackoverflow.com/a/78652807

from typing import Callable, overload

from rclpy.impl.rcutils_logger import RcutilsLogger as RosLogger
from rclpy.node import Node

from .domain.enums import LogLevel

class NodeAdapter(Node):
    def __init__(self, node_name: str, *args, **kwargs) -> None:
        super().__init__(node_name, *args, **kwargs)
    
    def get_logger(self, log_level = LogLevel.Info) -> RosLogger:
        return self._logger.get_child(log_level.name.lower())
    
    def log(self, message = None, log_level = LogLevel.Info) -> None:
        logger = self.get_logger(log_level)
        log_action = _map_log_level_to_action(logger, log_level)
        if message is None:
            log_action("")
            return
        log_action(message)


def _map_log_level_to_action(logger: RosLogger, log_level: LogLevel) -> Callable[[str], None]:
    match log_level:
        case LogLevel.Trace | LogLevel.Debug:
            return logger.debug
        case LogLevel.Info:
            return logger.info
        case LogLevel.Warn:
            return logger.warn
        case LogLevel.Error:
            return logger.error
        case LogLevel.Fatal:
            return logger.fatal
        case _:
            return logger.warn