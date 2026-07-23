from dataclasses import dataclass


@dataclass
class Intrinsics:
    cx: float  # pixel x-coordinate of the focal point
    cy: float  # pixel y-coordinate of the focal point
    fx: float  # focal length (in pixels) along x-axis
    fy: float  # focal length (in pixels) along y-axis
    tx: float = 0  # distance between the cameras (in mm)


@dataclass
class GridConfiguration:
    gw: float  # grid width in mm
    gh: float  # grid height in mm
    cw: float  # cell width in mm


@dataclass
class CameraPosition:
    x: float = 0  # mm, positive is right of wheelbase
    y: float = 0  # mm, positive is forward from wheelbase
    h: float = 0  # radians, positive is anticlockwise
