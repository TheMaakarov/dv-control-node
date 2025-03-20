import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.impl.logging_severity import LoggingSeverity

#from tuw_spline_msgs.msg import Spline
from driverless_msgs.msg import Actuation

from .path_tracking.model_predictive_speed_and_steer_control import Manager
from .path_planning.providers import FakePathPlanning
from .domain.enums import Route, LogLevel
from .domain.parameters import NodeParameters

class ControlNode(Node):

    def __init__(self, mpc_manager: Manager):
        super().__init__('control_node_publisher')
        self.log("Initialized.")
        self._publisher = self.create_publisher(Actuation, 'topic', 10)
        self.log("Publisher created.")
        timer_period = NodeParameters.SPIN_PERIOD
        self._mpc = mpc_manager
        self._mpc.bind_log_action(self.log)
        self.log("Launching timer.")
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.log('Timer callback called')
        next_action = self._mpc.next_action()
        self.log(f'Received actuation action: {next_action}')
        if next_action is None:
            self.log(f'Callback ended')
            return
        
        actuation_msg = Actuation()
        actuation_msg.accel = next_action[0]

        actuation_msg.steer = map_steer(
            next_action[1],
            -np.pi/4, np.pi/4,
            -1.0, 1.0)
        
        self.log(f'Publishing message: {actuation_msg}')
        self._publish(actuation_msg)

    def log(self, message, log_level = LogLevel.Info):
        # log_severity = map_log_level(log_level)
        log = self.get_logger()
        log.log(message, LoggingSeverity.WARN)
    
    def _publish(self, msg: Actuation):
        self._publisher.publish(msg)
        self.log(f'Published: "{msg}"')

def map_log_level(log_level: LogLevel) -> LoggingSeverity:
    match log_level:
        case LogLevel.Trace:
            return LoggingSeverity.UNSET
        case LogLevel.Debug:
            return LoggingSeverity.WARN
        case LogLevel.Info:
            return LoggingSeverity.INFO
        case LogLevel.Warn:
            return LoggingSeverity.WARN
        case LogLevel.Error:
            return LoggingSeverity.ERROR
        case LogLevel.Fatal:
            return LoggingSeverity.FATAL
        case _:
            return LoggingSeverity.UNSET


def map_steer(angle, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(angle - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

	
def main(args=None):
    rclpy.init(args=args)
    path_provider = FakePathPlanning(Route.SwitchBack)
    mpc_manager = Manager(path_provider)
    pub = ControlNode(mpc_manager)

    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
