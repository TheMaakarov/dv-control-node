import numpy as np
from typing import Callable

import rclpy

from driverless_msgs.msg import Actuation

from .node_wrapper import NodeAdapter as Node
from .path_tracking.model_predictive_speed_and_steer_control import Manager
from .path_planning.providers import FakePathPlanning
from .domain.enums import Route, LogLevel
from .domain.parameters import NodeParameters, TopicNames, AlgorithmParams

class ControlNode(Node):

    def __init__(self, mpc_manager: Manager):
        super().__init__(TopicNames.PUBLISHER_CONTROL)
        self.log("Initialized.")
        self._publisher = self.create_publisher(Actuation, TopicNames.SUBSCRIBER_ACTUATION, 10)
        self.log("Publisher created.")
        timer_period = NodeParameters.SPIN_PERIOD
        self._mpc = mpc_manager
        self._mpc.bind_log_action(self.log)
        self.log("Launching timer.")
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.log()
        self.log('Timer callback called', LogLevel.Debug)
        next_action = self._mpc.next_action()
        self.log(f'Received actuation action: {next_action}')
        if next_action is None:
            self.log(f'Callback ended')
            return
        
        actuation_msg = Actuation()
        actuation_msg.accel = next_action[0]

        actuation_msg.steer = map_to_range(
            next_action[1],
            AlgorithmParams.MAX_STEER,
            1.0)
        
        self.log(f'Publishing message: {actuation_msg}')
        self._publish(actuation_msg)
    
    def _publish(self, msg: Actuation):
        self._publisher.publish(msg)
        self.log(f'Published: "{msg}"')


def map_to_range(angle, leftMax, rightMax):
    """ Maps a value from one range to another.
    <angle> should be a value in the range [-<leftMax>, <leftMax>]
    and it will be mapped into the range [-<rightMax>, <rightMax>].
    The words "left" and "right" are used to indicate the direction of the conversion:
    <angle> ∈ <Range> --> <angle> ∈ <Range>

    Args:
        angle (float): the angle to be mapped.
        leftMax (float): the maximum value of the left range.
        rightMax (float): the maximum value of the right range.

    Returns:
        float: the mapped value in the new range.
    """
    if leftMax is 0.0:
        return rightMax  # Avoid division by zero
    
    # Figure out how 'wide' each range is
    leftMax, rightMax = abs(leftMax), abs(rightMax)
    leftMin, rightMin = -leftMax, -rightMax
    leftSpan, rightSpan = leftMax - leftMin, rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(angle - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    result = rightMin + (valueScaled * rightSpan)
    return result

	
def main(args=None):
    rclpy.init(args=args)
    path_provider = FakePathPlanning(Route.SwitchBack)
    mpc_manager = Manager(path_provider)
    pub = ControlNode(mpc_manager)

    try:
        rclpy.spin(pub)
    except KeyboardInterrupt:
        pub.log('Keyboard interrupt, shutting down.', LogLevel.Warn)
    except Exception as e:
        pub.log(f'Exception occurred: {e}', LogLevel.Error)
    finally:
        pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
