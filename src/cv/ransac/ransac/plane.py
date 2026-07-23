# ground plane mask creation

import math
import random
import warnings
from multiprocessing.pool import Pool
from typing import cast

import cv2
import numpy as np
import numpy.typing as npt

from .common import Intrinsics


def _pool(depths: npt.NDArray, kernel: tuple[int, int]) -> npt.NDArray:
    """Pool a depth image using `np.nanmean` on a rectangular kernel. Internal step of RANSAC."""
    h, w = depths.shape
    w -= w % kernel[1]
    h -= h % kernel[0]
    warnings.simplefilter("ignore", category=RuntimeWarning)
    return np.nanmean(
        depths[:h, :w].reshape(h // kernel[0], kernel[0], w // kernel[1], kernel[1]),
        axis=(1, 3),
    )


def _sample(pooled_depths: npt.NDArray) -> tuple[npt.NDArray, npt.NDArray]:
    """Sample 3 linearly-independent points. Internal step of RANSAC."""
    h, w = pooled_depths.shape
    a_mat = np.zeros((3, 3))
    b = np.zeros(3)

    while True:
        for i in range(3):
            row = -1
            col = -1
            while row < 0 or pooled_depths[row][col] < 0:
                row = random.randint(0, h - 1)
                col = random.randint(0, w - 1)
            a_mat[i] = [float(col), float(row), 1.0]
            b[i] = pooled_depths[row][col]
        if np.linalg.matrix_rank(a_mat) == 3:
            break
    return a_mat, np.transpose(b)


def _plane(a_mat: npt.NDArray, b: npt.NDArray) -> npt.NDArray:
    return cast(npt.NDArray, np.linalg.lstsq(a_mat, b, rcond=None)[0])


def _metric(depths: npt.NDArray, coeffs: npt.NDArray, tol: float) -> int:
    """Compute an approximate inlier metric for a set of coefficients. Internal step of RANSAC."""
    c1, c2, c3 = coeffs
    h, w = depths.shape

    x = np.arange(w, dtype=depths.dtype)[None, :]
    y = np.arange(h, dtype=depths.dtype)[:, None]

    r = c1 * x + c2 * y
    r += c3
    r -= depths
    np.abs(r, out=r)

    return int(np.count_nonzero((depths > 0) & (r < tol)))


def _ransac_mask(depths: npt.NDArray, coeffs: npt.NDArray, tolerance: float) -> npt.NDArray:
    """Generate a final ground plane mask for some coefficients. Internal step of RANSAC."""
    h, w = depths.shape
    c1, c2, c3 = coeffs

    x = np.arange(w, dtype=depths.dtype)[None, :]
    y = np.arange(h, dtype=depths.dtype)[:, None]
    r = (c1 * x + c2 * y + c3) - depths
    return cast(npt.NDArray, (depths > 0) & (r * r < tolerance))


def _ground_plane(pooled_depths: npt.NDArray, tolerance: float, times: int) -> tuple[int, npt.NDArray]:
    """Receive delegated computation tasks. Internal step of RANSAC."""
    result = np.array([0.0, 0.0, 0.0])
    best = 0
    for _ in range(times):
        coeffs = _plane(*_sample(pooled_depths))
        score = _metric(pooled_depths, coeffs, tolerance)
        if score > best:
            result = coeffs
            best = score
    return best, result


def _clean_depths(depths: npt.NDArray) -> npt.NDArray:
    """Remove irrelevant depth data. Internal step of RANSAC."""
    return np.where(depths > 10000 | np.isinf(depths), np.nan, depths)


def ground_plane(
    depths: npt.NDArray,
    samples: int = 100,
    kernel: tuple[int, int] = (1, 16),
    tolerance: float = 0.12,
    guess: npt.NDArray | None = None,
    thread_pool: Pool | None = None,
    processes: int = 4,
) -> tuple[npt.NDArray | None, npt.NDArray | None]:
    """
    Use random sample consensus (RANSAC) to identify generic obstacles.

    - `depths`: numpy array of depths taken from `sl.Mat.get_data()`
    - `samples`: number of times to try fitting a plane to the data
    - `kernel`: the (row x col) sized pooling kernel
    - `tol`: tolerance of the RANSAC masking functions. Can usually leave as-is.
    - `guess_in`: previous computed (pixel) coefficient array, if any. Helps provide more stable results.
    - `thread_pool`: optional argument to delegate sampling process using `multiprocessing`
    - `processes`: number of processes to delegate to
    """
    depths = _clean_depths(depths)

    # ground plane needs to be fit to the inverse of depth
    # this is scaled by the maximum depth for `tol` to be scene-agnostic
    max_depth = float(np.nanmax(depths))
    if not math.isfinite(max_depth):
        return None, guess
    inv_depths = max_depth / depths

    pooled = _pool(inv_depths, kernel)

    # compute inlier score for the guessed coefficients
    best_coeffs = np.array([0.0, 0.0, 0.0])
    if guess is None or guess.shape != (3,):
        print("warning: invalid plane coefficient estimates")
    else:
        best_coeffs = max_depth * guess.astype(float)
        best_coeffs[0] *= float(kernel[1])
        best_coeffs[1] *= float(kernel[0])
    best = max(1, _metric(pooled, best_coeffs, tolerance))  # give initial guess a slightly high value

    # delegate tasks and get the best results
    if thread_pool is None:
        run_best, run_coeffs = _ground_plane(pooled, tolerance, samples)
        if run_best > best and run_coeffs is not None:
            best_coeffs = run_coeffs
    else:
        args = (pooled, tolerance, samples // processes)
        results = thread_pool.starmap(_ground_plane, [args for _ in range(processes)])
        results.append((best, best_coeffs))
        best, best_coeffs = max(results, key=lambda t: t[0])

    # divide by the kernel to work on the expanded depth image
    best_coeffs[0] /= kernel[1]
    best_coeffs[1] /= kernel[0]

    res = _ransac_mask(inv_depths, best_coeffs, tolerance)
    return 255 * res.astype(np.uint8), np.array(best_coeffs) / max_depth


def real_coeffs(px_coeffs: npt.NDArray, intrinsics: Intrinsics) -> npt.NDArray:
    """Convert pixel-space plane coefficients (`px_coeffs`) to real/camera-space plane coefficients."""
    c1, c2, c3 = px_coeffs
    # d = depth at the focal point
    d = 1 / (c1 * intrinsics.cx + c2 * intrinsics.cy + c3)
    return np.array([-d * c1 * intrinsics.fx, d * c2 * intrinsics.fy, d])


def real_angle(real_coeffs: npt.NDArray) -> float:
    """Compute the angle of depression of the camera to the ground plane."""
    a, b, _ = real_coeffs
    rad = math.acos(1 / math.hypot(a, b, 1))
    if math.isnan(rad):
        return 0
    return math.pi / 2 - rad


def merge_masks(ground: npt.NDArray, lane: npt.NDArray) -> npt.NDArray:
    """Merge the `ground` plane mask with a mask of the `lane` lines, closing artifacts using morophological operations."""
    driveable = (ground == 255) & (lane == 0)
    driveable = driveable.astype(np.uint8) * 255

    close_kernel = np.ones((2, 2), np.uint8)
    driveable = cv2.morphologyEx(driveable, cv2.MORPH_CLOSE, close_kernel)
    open_kernel = np.ones((7, 7), np.uint8)
    return cv2.morphologyEx(driveable, cv2.MORPH_OPEN, open_kernel)
