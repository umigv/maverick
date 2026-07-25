# ground plane mask creation

from .common import *

from typing import cast
import warnings
import random
import math

import numpy as np
import cv2


def _pool(depths, kernel: tuple[int, int]):
    """Internal step of RANSAC, pools a depth image using `np.nanmean` on a rectangular kernel."""
    h, w = depths.shape
    w -= w % kernel[1]
    h -= h % kernel[0]
    warnings.simplefilter("ignore", category=RuntimeWarning)
    return np.nanmean(depths[:h, :w].reshape(h//kernel[0], kernel[0], w//kernel[1], kernel[1]), axis=(1, 3))


def _sample(pooled):
    """Internal step of RANSAC, samples 3 linearly-independent points."""

    h, w = pooled.shape
    A = np.zeros((3, 3))
    b = np.zeros(3)

    while True:
        for i in range(3):
            row = -1
            col = -1
            while row < 0 or pooled[row][col] < 0:
                row = random.randint(0, h - 1)
                col = random.randint(0, w - 1)
            A[i] = [float(col), float(row), 1.0]
            b[i] = pooled[row][col]
        if np.linalg.matrix_rank(A) == 3:
            break
    return A, np.transpose(b)


def _plane(A, b):
    return np.linalg.lstsq(A, b, rcond=None)[0]


def _metric(depths, coeffs, tol: float):
    """Internal step of RANSAC, computes an approximate inlier metric for a set of coefficients."""
    c1, c2, c3 = coeffs
    h, w = depths.shape

    x = np.arange(w, dtype=depths.dtype)[None, :]
    y = np.arange(h, dtype=depths.dtype)[:, None]

    r = c1 * x + c2 * y
    r += c3
    r -= depths
    np.abs(r, out=r)

    return np.count_nonzero((depths > 0) & (r < tol))


def _ransac_mask(depths, coeffs, tol: float):
    """Internal step of RANSAC, generates a final ground plane mask for some coefficients."""

    h, w = depths.shape
    c1, c2, c3 = coeffs

    x = np.arange(w, dtype=depths.dtype)[None, :]
    y = np.arange(h, dtype=depths.dtype)[:, None]
    r = (c1 * x + c2 * y + c3) - depths
    return (depths > 0) & (r * r < tol)


def _ground_plane(pooled, tol, times):
    """Internal RANSAC function, receives delegated computation tasks."""
    res = None
    best = 0
    for _ in range(times):
        coeffs = _plane(*_sample(pooled))
        score = _metric(pooled, coeffs, tol)
        if score > best:
            res = coeffs
            best = score
    return best, res


def _clean_depths(depths):
    """Internal step of RANSAC, removes irrelevant depth data."""
    depths = np.where(depths > 10000 | np.isinf(depths), np.nan, depths)
    return depths


# TODO: make this handle multiple depth frames?
def ground_plane(depths, samples=100, kernel=(1, 16), tol=0.12,
                 guess=np.array([0.0, 0.0, 0.0]), thread_pool=None, procs=4):
    """
    Uses random sample consensus (RANSAC) to identify generic obstacles.

    - `depths`: numpy array of depths taken from the `sl.Mat.get_data()`
    - `samples`: number of times to try fitting a plane to the data
    - `kernel`: the (row x col) sized pooling kernel
    - `tol`: tolerance of the RANSAC masking functions. Can usually leave as-is.
    - `guess`: previous computed (pixel) coefficient array, if any. Helps provide more stable results.
    - `thread_pool`: optional argument to delegate sampling process using `multiprocessing`
    - `procs`: number of processes to delegate to
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
    if guess.shape != (3,) or guess is None:
        print("warning: invalid plane coefficient estimates")
    else:
        best_coeffs = max_depth * guess.astype(float)
        best_coeffs[0] *= float(kernel[1])
        best_coeffs[1] *= float(kernel[0])
    best = max(1, _metric(pooled, best_coeffs, tol)) # give initial guess a slightly high value

    # delegate tasks and get the best results
    if thread_pool is None:
        run_best, run_coeffs = _ground_plane(pooled, tol, samples)
        if run_best > best:
            best_coeffs = run_coeffs
    else:
        args = (pooled, tol, samples // procs)
        results = thread_pool.starmap(
            _ground_plane, [args for _ in range(procs)])
        results.append((best, best_coeffs))
        best, best_coeffs = max(results, key=lambda t: t[0])

    # divide by the kernel to work on the expanded depth image
    best_coeffs = cast(np.ndarray, best_coeffs)
    best_coeffs[0] /= kernel[1]
    best_coeffs[1] /= kernel[0]

    res = _ransac_mask(inv_depths, best_coeffs, tol)
    return 255 * res.astype(np.uint8), np.array(best_coeffs) / max_depth


def real_coeffs(px_coeffs, intrinsics: Intrinsics):
    """Converts pixel-space plane coefficients (`px_coeffs`) to real/camera-space plane coefficients using the supplied `intrinsics`"""
    c1, c2, c3 = px_coeffs
    # d = depth at the focal point
    d = 1 / (c1 * intrinsics.cx + c2 * intrinsics.cy + c3)
    return (-d * c1 * intrinsics.fx, d * c2 * intrinsics.fy, d)


def real_angle(real_coeffs):
    """Computes the angle of depression of the camera to the ground plane described by the provided real-space coefficients."""
    a, b, _ = real_coeffs
    rad = math.acos(1 / math.hypot(a, b, 1))
    if (math.isnan(rad)):
        return 0
    return math.pi / 2 - rad


def merge_masks(ground, lane):
    """Merges the `ground` plane mask with a mask of the `lane` lines"""
    driveable = ((ground == 255) & (lane == 0))
    driveable = driveable.astype(np.uint8) * 255

    close_kernel = np.ones((2, 2), np.uint8)
    driveable = cv2.morphologyEx(driveable, cv2.MORPH_CLOSE, close_kernel)
    open_kernel = np.ones((7, 7), np.uint8)
    driveable = cv2.morphologyEx(driveable, cv2.MORPH_OPEN, open_kernel)
    return driveable
