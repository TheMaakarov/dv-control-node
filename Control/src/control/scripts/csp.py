import sys
import math

sys.path.append("/home/autonomo/control/lib/PythonRobotics/PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner as csp
except:
    raise


def get_simple_course(spline_params, dl, reverse=False):
    ax = spline_params[0]
    ay = spline_params[1]
    cx, cy, cyaw, ck, s = csp.calc_spline_course(
        ax, ay, ds=dl)

    if reverse:
        cyaw = [i - math.pi for i in cyaw]
    
    return cx, cy, cyaw, ck