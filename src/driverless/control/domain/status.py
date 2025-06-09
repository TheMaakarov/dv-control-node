from dataclasses import dataclass

@dataclass
class Pos2D:

    x: float
    y: float

@dataclass
class SplinePoint(Pos2D):

    yaw: float
    curvature: float
    is_final: bool = False

    def to_final(self):
        return SplinePoint(self.x, self.y, self.yaw, self.curvature, True)

@dataclass(frozen=False)
class State(Pos2D):
    """
    vehicle state class
    """
    yaw: float = 0.0
    v: float = 0.0
    predelta = None