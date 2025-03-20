#!/usr/bin/env python2

import rospy
from control.msg import Action, Pose
import numpy as np
import socket
import pickle

rate = 30
act = Action()

HOST = '127.0.0.1'
SENDER_PORT = 65431
RECEIVER_PORT = 65432
Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
Socket.bind((HOST, SENDER_PORT))

class Spline:

    def __init__(self, cx=[0.0], cy=[0.0], cyaw=[0.0], ck=[0.0]):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck


def set_straight_course():
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return (ax, ay)


def set_straight_course2():
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    return (ax, ay)


def set_straight_course3():
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]

    return (ax, ay, True)


def set_forward_course():
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]

    return (ax, ay)


def set_switch_back_course():
    ax1 = [0.0, 30.0, 6.0, 20.0, 35.0, 35.0, 10.0, 0.0, 0.0]
    ay1 = [0.0, 0.0, 20.0, 35.0, 20.0, 20.0, 30.0, 5.0, 0.0]
    return (ax1, ay1, True)


def callback():
    """
	spline.cx = data.cx
	spline.cy = data.cy
	spline.cyaw = data.cyaw
	spline.ck = data.ck
    """
    return set_forward_course()

"""
def parse():
	return get_straight_course3()
"""

def send(data):
    bytes = pickle.dumps(data, 2)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(( HOST, RECEIVER_PORT ))
    s.sendall(bytes)
    s.close()


def receive():
    Socket.listen(1)
    while True:
        sock, addr = Socket.accept()

        in_data = sock.recv(1024)
        if in_data != b'':
            break

    # print(in_data)
    data = pickle.loads(in_data)
    pose, acc, steer = (x for x in data)
    # Socket.close()
    return pose, acc, steer


def map_steer(angle, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(angle - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


def sender():

    publisher = rospy.Publisher('sender', Action)
    rospy.init_node('sender', anonymous=True)
    # rospy.Subscriber('path_planning', Spline, callback)
    r = rospy.Rate(rate)

    spline_params = callback()

    old_pose = Pose()
    old_pose.x = spline_params[0][0]
    old_pose.y = spline_params[1][0]
    old_pose.v = 0.0
    old_pose.yaw = 0.0 #! Yaw should be calculated first
    send((old_pose, spline_params))         # Send the spline received for processing

    while not rospy.is_shutdown():

        new_pose, acc, steer = receive()    # Receive the new poses
               
        # Update Action
        if acc < 0:
            act.acc = 0.0
            act.brake = abs(acc)
        else:
            act.acc = acc
            act.brake = 0.0
        act.steer = map_steer(steer, -np.pi/4, np.pi/4, -1.0, 1.0)

        print(act)

        # Update the pose
        old_pose = new_pose

        # Send Action data (via ROS)
        publisher.publish(act)

    r.sleep()

    rospy.spin()


if __name__ == '__main__':
	sender()
