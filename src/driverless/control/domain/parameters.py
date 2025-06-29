from math import radians

class VehicleParams:
    LENGTH = 4.5  # [m]
    WIDTH = 2.0  # [m]
    BACKTOWHEEL = 1.0  # [m]
    WHEEL_LEN = 0.3  # [m]
    WHEEL_WIDTH = 0.2  # [m]
    TREAD = 0.7  # [m]
    WB = 2.5  # [m]

class AlgorithmParams:
    DETECTION_DISTANCE = 20.0 # [m]
    MAX_STEER = radians(45.0)  # maximum steering angle [rad]
    MAX_DSTEER = radians(30.0)  # maximum steering speed [rad/s]
    MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
    MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
    MAX_ACCEL = 1.0  # maximum accel [m/ss]

class NodeParameters:
    SPIN_PERIOD = 0.05

class TopicNames:
    SUBSCRIBER_ACTUATION = 'actuation_node_subscriber'
    PUBLISHER_CONTROL = 'control_node_publisher'