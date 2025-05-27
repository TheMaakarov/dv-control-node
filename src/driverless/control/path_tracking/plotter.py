import math
import numpy as np
import matplotlib.pyplot as plt
from ..domain.parameters import VehicleParams as Car

def plot_car_state(state, di, ox, oy, cx, cy, x, y, xref, target_ind, time):
    plt.cla()
    if ox is not None:
        plt.plot(ox, oy, "xr", label="MPC")
    plt.plot(cx, cy, "-r", label="course")
    plt.plot(x, y, "ob", label="trajectory")
    plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
    plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
    _plot_car(state.x, state.y, state.yaw, steer=di)
    plt.axis("equal")
    plt.grid(True)
    plt.title("Time[s]:" + str(round(time, 2)) +
                ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
    plt.pause(0.0001)

def _plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):

    outline = np.matrix([[-Car.BACKTOWHEEL, (Car.LENGTH - Car.BACKTOWHEEL), (Car.LENGTH - Car.BACKTOWHEEL), -Car.BACKTOWHEEL, -Car.BACKTOWHEEL],
                         [Car.WIDTH / 2, Car.WIDTH / 2, - Car.WIDTH / 2, -Car.WIDTH / 2, Car.WIDTH / 2]])

    fr_wheel = np.matrix([[Car.WHEEL_LEN, -Car.WHEEL_LEN, -Car.WHEEL_LEN, Car.WHEEL_LEN, Car.WHEEL_LEN],
                          [-Car.WHEEL_WIDTH - Car.TREAD, -Car.WHEEL_WIDTH - Car.TREAD, Car.WHEEL_WIDTH - Car.TREAD, Car.WHEEL_WIDTH - Car.TREAD, -Car.WHEEL_WIDTH - Car.TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.matrix([[math.cos(yaw), math.sin(yaw)],
                      [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.matrix([[math.cos(steer), math.sin(steer)],
                      [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T * Rot2).T
    fl_wheel = (fl_wheel.T * Rot2).T
    fr_wheel[0, :] += Car.WB
    fl_wheel[0, :] += Car.WB

    fr_wheel = (fr_wheel.T * Rot1).T
    fl_wheel = (fl_wheel.T * Rot1).T

    outline = (outline.T * Rot1).T
    rr_wheel = (rr_wheel.T * Rot1).T
    rl_wheel = (rl_wheel.T * Rot1).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")