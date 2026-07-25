# occupancy grid generation

from .common import *
from . import plane

import numpy as np

import math


def create_ground_cloud(coords, ransac_coeffs):
    """
    Generates a (pixel-space) cloud of points on the ground plane.
    - `coords`: N by 2 numpy array containing coordinates in the image
    - `ransac_coeffs`: plane coefficients outputted by `ground_plane()`
    """
    c1, c2, c3 = ransac_coeffs
    z = 1 / (c1 * coords[:, 0] + c2 * coords[:, 1] + c3)
    z = z.reshape(-1, 1)
    return np.concatenate((coords.astype(np.float64), z), axis=1)


def pixel_to_real(pixel_cloud, real_coeffs,
                  intr: Intrinsics, orientation: float = 0.0):
    """
    Converts a point cloud from pixel-space to camera-space.
    - `pixel_cloud`: self-explanatory
    - `real_coeffs`: ground plane coefficients for real-world coordinate system
    - `intr`: camera intrinsics (not a proportion)
    - `orientation`: where the camera is pointing, with positive being leftward
    - Returns: a point cloud in camera space (x, y, z in mm) with y-values relative to the camera height.
    """

    # converts px into mm
    cloud = pixel_cloud.copy()
    cloud[:, 0] = pixel_cloud[:, 2] * (pixel_cloud[:, 0] - intr.cx) / intr.fx
    cloud[:, 1] = pixel_cloud[:, 2] * (intr.cy - pixel_cloud[:, 1]) / intr.fy

    depression = plane.real_angle(real_coeffs)
    c_1 = math.cos(depression)
    s_1 = math.sin(depression)
    # each column affects the output (x, y, z) respectively
    rotation_matrix = np.array([[1.0, 0.0,  0.0],
                                [0.0, c_1, -s_1],
                                [0.0, s_1,  c_1]]).transpose()

    c_2 = math.cos(orientation)
    s_2 = math.sin(orientation)
    rotation_matrix = rotation_matrix @ np.array([[c_2, 0.0, -s_2],
                                                  [0.0, 1.0,  0.0],
                                                  [s_2, 0.0,  c_2]]).transpose()

    return cloud @ rotation_matrix


# TODO decompose pitch + roll angles
# TODO? can shave off a few ms by computing transformation matrix and using cv2.warpPerspective
# the numpy fuckery in this just helps interpolation
# INPUT: np.uint8 array representing the image mask
def occ_grid(mask_in, real_coeffs, intr: Intrinsics, conf: GridConfiguration,
             pos: CameraPosition, thres=200):
    """
    Generates an bird's-eye view occupancy grid using bilinear interpolation.
    - `mask_in`: np.uint8 array of the image mask
    - `real_coeffs`: ground plane coefficients for real-world coordinate system
    - `intr`: camera intrinsics (not a proportion)
    - `conf`: grid configuration details like physical grid size
    - `pos`: camera position and orientation relative to robot wheel-centre
    - `thres`: bilinear interpolation threshold to mark a cell as empty.
    """
    
    res = 2
    # enforce grid symmetry
    # first and second indices are number of interpolation layers to compute
    grid_shape = (res, res, 2 * int((0.5 * conf.gh) // conf.cw),
                  2 * int((0.5 * conf.gw) // conf.cw))
    true_width = conf.cw * grid_shape[3]
    true_height = conf.cw * grid_shape[2]

    lys = np.arange(grid_shape[0])[:, None, None, None]
    lxs = np.arange(grid_shape[1])[None, :, None, None]
    gys = np.arange(grid_shape[2])[None, None, :, None]
    gxs = np.arange(grid_shape[3])[None, None, None, :]

    # apply camera rotation around the correct point
    rgxs = gxs - grid_shape[3] / 2 + 0.5 - (pos.x / conf.cw)
    rgys = grid_shape[2] - gys - 0.5 - (pos.y / conf.cw)
    rgxs_tmp = rgxs * math.cos(pos.h) + rgys * math.sin(pos.h)
    rgys_tmp = -rgxs * math.sin(pos.h) + rgys * math.cos(pos.h)
    # intr.tx term compensates for depths being centered on left camera lens
    # shift "after" position because rg{x,y}s used to poll from the mask
    rgxs = rgxs_tmp + grid_shape[3] / 2 - 0.5 + (intr.tx / conf.cw / 2)
    rgys = grid_shape[2] - rgys_tmp - 0.5

    # pixel values into mm
    cxs = conf.cw * ((lxs + 0.5) / grid_shape[0] + rgxs) - 0.5 * true_width
    cys = true_height - conf.cw * ((lys + 0.5) / grid_shape[1] + rgys)

    # project onto the camera plane
    a, b, d = real_coeffs
    theta = plane.real_angle(real_coeffs)
    cam_height = math.sin(theta) * d
    cys = cys * math.sin(theta)
    cys = cys - math.cos(theta) * cam_height

    # use mask to highlight driveable regions
    # this equation enforces that all (cxs, cys, zs) are on a 2-d surface, creating a bijection between the mask and the ground plane, eliminating false positives (where the ground plane location is under an obstacle but it maps to a pixel that is not occupied)
    zs = np.clip(a * cxs + b * cys + d, 1.0, None)
    pxs = np.round((cxs * intr.fx) / zs + intr.cx)
    pys = np.round(intr.cy - (cys * intr.fy) / zs)

    # ignore mask's outer edge
    mask = mask_in.astype(np.float16)
    mask[[0, -1], :] = np.nan
    mask[:, [0, -1]] = np.nan

    # copy the data over
    pxs = np.clip(pxs, 0, mask.shape[1] - 1).astype(np.int32)
    pys = np.clip(pys, 0, mask.shape[0] - 1).astype(np.int32)

    grid = np.zeros(grid_shape, dtype=np.float16)
    grid[lys, lxs, gys, gxs] = mask[pys, pxs]
    grid = np.mean(grid, axis=(0, 1))
    grid = np.where(grid > thres, 255, grid)
    grid = np.where(grid < thres, 0, grid)
    grid = np.nan_to_num(grid, nan=127)

    return grid.astype(np.uint8)
