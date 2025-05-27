"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

"""
from collections.abc import Callable
import numpy as np
import math
import cvxpy
from . import plotter
from ..domain.enums import LogLevel
from ..domain.parameters import VehicleParams as Car
from ..domain.status import SplinePoint, State, Pos2D
from ..path_planning.providers import PathProvider

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.2  # [s] time tick

MAX_STEER = math.radians(45.0)  # maximum steering angle [rad]
MAX_DSTEER = math.radians(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

DO_ANIMATION = True

class ManagementStatus:
    
    def __init__(self, initial_state: State, time_tick: float):
        self._states: list[State] = [initial_state]
        self._steers: list[float] = [0.0]
        self._accels: list[float] = [0.0]

        self._time: float = 0.0
        self._time_tick: float = time_tick
        self._times: list[float] = [0.0]

        self._last_index = 0
        self._steer_matrix = []
        self._accel_matrix = []

    @property
    def states(self):
        return self._states

    @property
    def last_state(self):
        return self.states[-1]

    @property
    def last_odelta(self):
        if len(self._steer_matrix) == 0:
            return None
        return self._steer_matrix[-1]

    @property
    def last_oa(self):
        if len(self._accel_matrix) == 0:
            return None
        return self._accel_matrix[-1]

    @property
    def time(self):
        return self._time

    @property
    def last_index(self):
        return self._last_index
    
    def update(self, state: State, steer: float, acceleration: float, steer_values, accel_values, new_index: int):
        self._states.append(state)
        self._steers.append(steer)
        self._accels.append(acceleration)
        self._steer_matrix.append(steer_values)
        self._accel_matrix.append(accel_values)
        self._last_index = new_index
    
    def tick(self):
        self._time += self._time_tick
        self._times.append(self._time)

class Manager:

    def __init__(self, path_provider: PathProvider):
        self._log_action = None
        self._path_provider = path_provider
        self._raw_route: list[SplinePoint] = []
        self._soft_route: list[SplinePoint] = []
        first_point = self._path_provider.get_first_spline_point()

        if first_point is None:
            return
        
        first_state = State(first_point.x, first_point.y, first_point.yaw, v=0.0)

        # initial yaw compensation
        if first_state.yaw - first_point.yaw >= math.pi:
            first_state.yaw -= math.pi * 2.0
        elif first_state.yaw - first_point.yaw <= -math.pi:
            first_state.yaw += math.pi * 2.0

        self._management_status = ManagementStatus(first_state, DT)
    
    def bind_log_action(self, log_action: Callable[[str, LogLevel], None]):
        self._log_action = log_action
    
    def log(self, message, log_level: LogLevel):
        if self._log_action is None:
            return
        self._log_action(message, log_level)

    def next_action(self) -> tuple[float, float]:
        if self._management_status is None:
            return None
        
        spline_point = self._path_provider.get_next_spline_point(self._management_status.last_state)
        self.log(f'Retreived next point: {spline_point}', LogLevel.Debug)
        if spline_point is not None:
            self._add_point(spline_point)
            self._speed_profile = _calc_path_speed_profile(self._soft_route, TARGET_SPEED)
            self.log('Recalculations performed', LogLevel.Debug)
        
        next_action_values = self._iterate_next_action()
        self.log(f'Calculated next action: {next_action_values}', LogLevel.Debug)
        return next_action_values
    
    def _iterate_next_action(self) -> tuple[float, float]:
        if self._speed_profile is None:
            return None
        
        sp = self._speed_profile
        dl = self._path_provider.course_tick
        last_state = self._management_status.last_state
        last_index = self._management_status.last_index
        last_odelta = self._management_status.last_odelta
        last_oa = self._management_status.last_oa

        xref, target_ind, dref = _calc_ref_path_trajectory(
            last_state, self._soft_route, sp, dl, last_index)

        x0 = [last_state.x, last_state.y, last_state.v, last_state.yaw]  # current state

        oa, odelta, ox, oy, _, _ = _iterative_linear_mpc_control(
            xref, x0, dref, last_oa, last_odelta)

        if odelta is not None:
            di, ai = odelta[0], oa[0]
        
        new_state = _update_state(last_state, ai, di)

        self._management_status.update(new_state, di, ai, odelta, oa, target_ind)
        self._management_status.tick()

        if DO_ANIMATION:
            cx, cy = _extract_location(self._soft_route)
            xs, ys = _extract_location(self._management_status.states)
            plotter.plot_car_state(
                new_state, di,
                ox, oy,
                cx, cy,
                xs, ys,
                xref, target_ind,
                self._management_status.time)

        if _check_goal(new_state, self._get_goal(), target_ind, len(self._soft_route)):
            return None
        
        return (ai, new_state.yaw)
    
    def _get_goal(self):
        if self._soft_route is None or len(self._soft_route) == 0:
            return None
        
        return self._soft_route[-1]
    
    def _add_point(self, spline_point: SplinePoint) -> None:
        self._raw_route.append(spline_point)

        # Smooth the curves
        smooth_curves = self.get_smooth_curves()
        self._soft_route = smooth_curves

    def get_smooth_curves(self) -> list[SplinePoint]:

        softened_curves: list[SplinePoint] = []
        if len(self._raw_route) == 0:
            return []

        softened_curves.append(self._raw_route[0])

        for i in range(len(self._raw_route) - 1):
            current_yaw, next_yaw = self._raw_route[i].yaw, self._raw_route[i + 1].yaw
            dyaw = next_yaw - current_yaw

            while dyaw >= math.pi / 2.0:
                next_yaw -= math.pi * 2.0
                dyaw = next_yaw - current_yaw

            while dyaw <= -math.pi / 2.0:
                next_yaw += math.pi * 2.0
                dyaw = next_yaw - current_yaw
            
            next_point = self._raw_route[i + 1]
            soft_curve = SplinePoint(next_point.x, next_point.y, next_yaw, next_point.curvature, next_point.is_final)
            softened_curves.append(soft_curve)

        return softened_curves

def _extract_location(positions: list[Pos2D]) -> tuple[list[float], list[float]]:
    xs, ys = zip(*[(pos.x, pos.y) for pos in positions])
    return list(xs), list(ys)

def _pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def _get_linear_model_matrix(v, phi, delta):

    A = np.matrix(np.zeros((NX, NX)))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / Car.WB

    B = np.matrix(np.zeros((NX, NU)))
    B[2, 0] = DT
    B[3, 1] = DT * v / (Car.WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = v * delta / (Car.WB * math.cos(delta) ** 2)

    return A, B, C


def _update_state(state: State, a: float, delta: float):

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    new_x = state.x + state.v * math.cos(state.yaw) * DT
    new_y = state.y + state.v * math.sin(state.yaw) * DT
    new_yaw = state.yaw + state.v / Car.WB * math.tan(delta) * DT
    new_v = state.v + a * DT

    if new_v > MAX_SPEED:
        new_v = MAX_SPEED
    elif new_v < MIN_SPEED:
        new_v = MIN_SPEED

    new_state = State(new_x, new_y, new_yaw, new_v)
    return new_state


def _get_nparray_from_matrix(x):
    return np.array(x).flatten()


def _calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = _pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def _predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i in range(len(x0)):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = _update_state(state, ai, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def _iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC control with updating operational point iteraitvely
    """

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = _predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = _linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    else:
        pass

    return oa, od, ox, oy, oyaw, ov


def _linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = _get_linear_model_matrix(
            xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t])
                            <= MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CLARABEL, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = _get_nparray_from_matrix(x.value[0, :])
        oy = _get_nparray_from_matrix(x.value[1, :])
        ov = _get_nparray_from_matrix(x.value[2, :])
        oyaw = _get_nparray_from_matrix(x.value[3, :])
        oa = _get_nparray_from_matrix(u.value[0, :])
        odelta = _get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def _calc_ref_path_trajectory(state, route: list[SplinePoint], sp, dl, pind):
    cx, cy, cyaw, ck = zip(*[(p.x, p.y, p.yaw, p.curvature) for p in route])
    return _calc_ref_trajectory(state, list(cx), list(cy), list(cyaw), list(ck), sp, dl, pind)

def _calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = _calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref

def _check_goal(state: State, goal: Pos2D, tind: int, nind: int) -> bool:

    # check goal
    dx = state.x - goal.x
    dy = state.y - goal.y
    d = math.sqrt(dx ** 2 + dy ** 2)

    if (d <= GOAL_DIS):
        isgoal = True
    else:
        isgoal = False

    if abs(tind - nind) >= 5:
        isgoal = False

    if (abs(state.v) <= STOP_SPEED):
        isstop = True
    else:
        isstop = False

    if isgoal and isstop:
        return True

    return False


def _calc_path_speed_profile(path: list[SplinePoint], target_speed: float) -> list[float]:
    xs, ys, yaws = zip(*[(p.x, p.y, p.yaw) for p in path])
    return _calc_speed_profile(list(xs), list(ys), list(yaws), target_speed)

def _calc_speed_profile(cx: list[float], cy: list[float], cyaw: list[float], target_speed: float):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(_pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def main():
    """
    print(__file__ + " start!!")

    dl = 1.0  # course tick
    # cx, cy, cyaw, ck = get_straight_course(dl)
    # cx, cy, cyaw, ck = get_straight_course2(dl)
    cx, cy, cyaw, ck = _get_straight_course3(dl)
    # cx, cy, cyaw, ck = get_forward_course(dl)
    # CX, cy, cyaw, ck = get_switch_back_course(dl)

    sp = _calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    t, x, y, yaw, v, d, a = _do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    if DO_ANIMATION:
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()
    """


def main2():
    """
    print(__file__ + " start!!")

    dl = 1.0  # course tick
    cx, cy, cyaw, ck = _get_straight_course3(dl)

    sp = _calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    initial_state = State(x=cx[0], y=cy[0], yaw=0.0, v=0.0)

    t, x, y, yaw, v, d, a = _do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    if DO_ANIMATION:
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()
    """


if __name__ == '__main__':
    # main()
    main2()
