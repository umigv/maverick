# occupancy grid generation

import math

import numpy as np

from . import plane
from .common import CameraPosition, GridConfiguration, Intrinsics


def create_ground_cloud(coords, ransac_coeffs):
    """
    Generate a (pixel-space) cloud of points on the ground plane.

    - `coords`: N by 2 numpy array containing coordinates in the image
    - `ransac_coeffs`: plane coefficients outputted by `ground_plane()`
    """
    c1, c2, c3 = ransac_coeffs
    z = 1 / (c1 * coords[:, 0] + c2 * coords[:, 1] + c3)
    z = z.reshape(-1, 1)
    return np.concatenate((coords.astype(np.float64), z), axis=1)


def pixel_to_real(pixel_cloud, real_coeffs, intr: Intrinsics, orientation: float = 0.0):
    """
    Convert a point cloud from pixel-space to camera-space.

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
    rotation_matrix = np.array([[1.0, 0.0, 0.0], [0.0, c_1, -s_1], [0.0, s_1, c_1]]).transpose()

    c_2 = math.cos(orientation)
    s_2 = math.sin(orientation)
    rotation_matrix = rotation_matrix @ np.array([[c_2, 0.0, -s_2], [0.0, 1.0, 0.0], [s_2, 0.0, c_2]]).transpose()

    return cloud @ rotation_matrix


# TODO decompose pitch + roll angles
# TODO can shave off a few ms by computing transformation matrix and using cv2.warpPerspective
def occ_grid(
    mask_in,
    real_coeffs,
    intr: Intrinsics,
    conf: GridConfiguration,
    pos: CameraPosition,
    thres=200,
    resolution=2,
):
    """
    Generate a bird's-eye view occupancy grid using bilinear interpolation.

    - `mask_in`: np.uint8 array of the image mask
    - `real_coeffs`: ground plane coefficients for real-world coordinate system
    - `intr`: camera intrinsics (not a proportion)
    - `conf`: grid configuration details like physical grid size
    - `pos`: camera position and orientation relative to robot wheel-centre
    - `thres`: bilinear interpolation threshold to mark a cell as empty
    - `resolution`: number of interpolation points to transform per grid cell along each axis
    """
    # enforce grid symmetry
    grid_shape = (
        resolution,
        resolution,
        2 * int((0.5 * conf.gh) // conf.cw),
        2 * int((0.5 * conf.gw) // conf.cw),
    )
    true_width = conf.cw * grid_shape[3]
    true_height = conf.cw * grid_shape[2]

    interpolation_ys = np.arange(grid_shape[0])[:, None, None, None]
    interpolation_xs = np.arange(grid_shape[1])[None, :, None, None]
    grid_ys = np.arange(grid_shape[2])[None, None, :, None]
    grid_xs = np.arange(grid_shape[3])[None, None, None, :]

    # apply camera rotation around the correct point
    rotated_grid_xs = grid_xs - grid_shape[3] / 2 + 0.5 - (pos.x / conf.cw)
    rotated_grid_ys = grid_shape[2] - grid_ys - 0.5 - (pos.y / conf.cw)
    rgxs_tmp = rotated_grid_xs * math.cos(pos.h) + rotated_grid_ys * math.sin(pos.h)
    rgys_tmp = -rotated_grid_xs * math.sin(pos.h) + rotated_grid_ys * math.cos(pos.h)
    # intr.tx term compensates for depths being centered on left camera lens
    # shift "after" position because rg{x,y}s used to poll from the mask
    rotated_grid_xs = rgxs_tmp + grid_shape[3] / 2 - 0.5 + (intr.tx / conf.cw / 2)
    rotated_grid_ys = grid_shape[2] - rgys_tmp - 0.5

    # pixel values into mm
    cell_xs = conf.cw * ((interpolation_xs + 0.5) / grid_shape[0] + rotated_grid_xs) - 0.5 * true_width
    cell_ys = true_height - conf.cw * ((interpolation_ys + 0.5) / grid_shape[1] + rotated_grid_ys)

    # project onto the camera plane
    a, b, d = real_coeffs
    theta = plane.real_angle(real_coeffs)
    camera_height = math.sin(theta) * d
    cell_ys = cell_ys * math.sin(theta)
    cell_ys = cell_ys - math.cos(theta) * camera_height

    # use mask to highlight driveable regions
    # this equation enforces that all (cxs, cys, zs) are on a 2-d surface, creating a bijection between the mask and the ground plane, eliminating false positives (where the ground plane location is under an obstacle but it maps to a pixel that is not occupied)
    depths = np.clip(a * cell_xs + b * cell_ys + d, 1.0, None)
    pixel_xs = np.round((cell_xs * intr.fx) / depths + intr.cx)
    pixel_ys = np.round(intr.cy - (cell_ys * intr.fy) / depths)

    # ignore mask's outer edge
    mask = mask_in.astype(np.float16)
    mask[[0, -1], :] = np.nan
    mask[:, [0, -1]] = np.nan

    # copy the data over
    pixel_xs = np.clip(pixel_xs, 0, mask.shape[1] - 1).astype(np.int32)
    pixel_ys = np.clip(pixel_ys, 0, mask.shape[0] - 1).astype(np.int32)

    grid = np.zeros(grid_shape, dtype=np.float16)
    grid[interpolation_ys, interpolation_xs, grid_ys, grid_xs] = mask[pixel_ys, pixel_xs]
    grid = np.mean(grid, axis=(0, 1))
    grid = np.where(grid > thres, 255, grid)
    grid = np.where(grid < thres, 0, grid)
    grid = np.nan_to_num(grid, nan=127)

    return grid.astype(np.uint8)
