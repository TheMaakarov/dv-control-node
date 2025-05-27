import math
from ..domain.enums import Route
from ..domain.status import SplinePoint, State
from ..domain.parameters import AlgorithmParams
from ..domain.CubicSpline import calc_spline_course

class PathProvider:

    def __init__(self):
        pass
    
    @property
    def course_tick(self) -> float:
        return 1.0
    
    def get_first_spline_point(self) -> SplinePoint:
        return None

    def get_next_spline_point(self, car_state: State) -> SplinePoint:
        return None

class FakePathPlanning(PathProvider):
    
    def __init__(self, route: Route, course_tick: float = 1.0):
        self._course_tick = course_tick
        self._route_data = FakePathPlanning._get_route_data(
            route,
            self._course_tick)
        self._current_index = 0
        self._path_length = 0
        if self._route_data is not None:
            self._path_length = len(self._route_data[0])
    
    @property
    def course_tick(self):
        return self._course_tick
    
    def get_first_spline_point(self) -> SplinePoint:
        if not self._has_data():
            return None
        
        return self._get_point_at_index(self, 0)

    def get_next_spline_point(self, car_state: State) -> SplinePoint:
        if not self._has_data() or self._current_index >= self._path_length:
            return None
        
        next_point = self._get_point_at_index(self._current_index)
        distance = math.dist([car_state.x, car_state.y], [next_point.x, next_point.y])
        if distance > AlgorithmParams.DETECTION_DISTANCE:
            return None

        self._current_index += 1
        return next_point

    @staticmethod
    def _get_route_data(route: Route, course_tick: float):

        def _get_straight_course(dl):
            ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
            ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            cx, cy, cyaw, ck, s = calc_spline_course(
                ax, ay, ds=dl)

            return cx, cy, cyaw, ck
        
        def _get_straight_course2(dl):
            ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
            ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
            cx, cy, cyaw, ck, s = calc_spline_course(
                ax, ay, ds=dl)

            return cx, cy, cyaw, ck
        
        def _get_straight_course3(dl):
            cx, cy, cyaw, ck = _get_straight_course2(dl)
            cyaw = [i - math.pi for i in cyaw]
            return cx, cy, cyaw, ck
        
        def _get_forward_course(dl):
            ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
            ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
            cx, cy, cyaw, ck, s = calc_spline_course(
                ax, ay, ds=dl)

            return cx, cy, cyaw, ck
        
        def _get_switch_back_course(dl):
            ax = [0.0, 30.0, 6.0, 20.0, 35.0]
            ay = [0.0, 0.0, 20.0, 35.0, 20.0]
            cx, cy, cyaw, ck, s = calc_spline_course(
                ax, ay, ds=dl)
            ax = [35.0, 10.0, 0.0, 0.0]
            ay = [20.0, 30.0, 5.0, 0.0]
            cx2, cy2, cyaw2, ck2, s2 = calc_spline_course(
                ax, ay, ds=dl)
            cyaw2 = [i - math.pi for i in cyaw2]
            cx.extend(cx2)
            cy.extend(cy2)
            cyaw.extend(cyaw2)
            ck.extend(ck2)

            return cx, cy, cyaw, ck

        match route:
            case Route.Straight:
                return _get_straight_course(course_tick)
            case Route.Serpent:
                return _get_straight_course2(course_tick)
            case Route.SerpentRev:
                return _get_straight_course3(course_tick)
            case Route.Forward:
                return _get_forward_course(course_tick)
            case Route.SwitchBack:
                return _get_switch_back_course(course_tick)
        return None
    
    def _get_point_at_index(self, index: int) -> SplinePoint:
        cx, cy, cyaw, ck = self._route_data
        x, y, yaw, k = cx[index], cy[index], cyaw[index], ck[index]
        is_final = index == self._path_length - 1
        return SplinePoint(x, y, yaw, k, is_final)
    
    def _has_data(self) -> bool:
        return self._route_data is not None and self._path_length > 0
