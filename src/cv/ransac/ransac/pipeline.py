# depth data source abstractions

import abc
import random
from multiprocessing import Pool
from typing import Any, cast

import cv2
import h5py
import numpy as np
import numpy.typing as npt

from . import occu, plane
from .common import CameraPosition, GridConfiguration, Intrinsics

try:
    import pyzed.sl as sl
except ImportError:
    print("[warn] pyzed not found, LiveSource will not work")
    sl = None


class DepthSource(metaclass=abc.ABCMeta):
    """Generic data source for for depth segmentation."""

    @abc.abstractmethod
    def timestamp(self) -> int:
        """Return the timestamp associatede with the image/depth data."""

    @abc.abstractmethod
    def update(self) -> bool:
        """Trigger an update, grabbing a new frame if possible."""

    @abc.abstractmethod
    def image(self) -> npt.NDArray:
        pass

    @abc.abstractmethod
    def depth_map(self) -> npt.NDArray:
        pass

    @abc.abstractmethod
    def intrinsics(self) -> Intrinsics:
        pass

    @abc.abstractmethod
    def about(self) -> str:
        pass


class HDF5Source(DepthSource):
    """Provides a data source from the new multi-camera hdf5 format."""

    def __init__(self, file: h5py.File, dataset_index: int = 0):
        self.file = file

        self.info = cast(h5py.Dataset, self.file["inf" + str(dataset_index)])
        self.timestamps = cast(h5py.Dataset, self.file["tim" + str(dataset_index)])
        self.images = cast(h5py.Dataset, self.file["img" + str(dataset_index)])
        self.depth_maps = cast(h5py.Dataset, self.file["dep" + str(dataset_index)])

        # scale intrinsics by the image dimensions
        h, w = self.depth_maps[0].shape
        self._intrinsics = Intrinsics(
            cx=self.info["cx_left"][()] * w,
            cy=self.info["cy_left"][()] * h,
            fx=self.info["fx_left"][()] * w,
            fy=self.info["fy_left"][()] * h,
            tx=self.info["tx"][()],
        )

        self.frame_count = len(self.timestamps)
        self.frame_number = 0
        self.update()

    def __delete__(self) -> None:
        self.file.close()

    def update(self) -> bool:
        """Trigger an update, grabbing a new frame if possible."""
        self._timestamp = int(self.timestamps[self.frame_number][()])
        self._image = np.array(self.images[self.frame_number])
        self._depth_map = np.array(self.depth_maps[self.frame_number])
        return True

    def use_frame(self, frame: int = -1) -> int:
        if frame < 0 or frame >= self.frame_count:
            self.frame_number = random.randint(0, self.frame_count - 1)
        else:
            self.frame_number = frame
        self.update()

        return self.frame_number

    def timestamp(self) -> int:
        """Return the timestamp associatede with the image/depth data."""
        return self._timestamp

    def image(self) -> npt.NDArray:
        return self._image

    def depth_map(self) -> npt.NDArray:
        return self._depth_map

    def intrinsics(self) -> Intrinsics:
        return self._intrinsics

    def about(self) -> str:
        return f"hdf5 depth source ({self.file.filename})"


class LiveSource(DepthSource):
    """Provides depth data from the zed camera.

    `update()` is blocking so should be called in a thread if needing async updates

    example:
    ````
    def grab(source: DepthSource):
        while source.update():
            time.sleep(0.001)

    grab_thread = threading.Thread(target=grab, args=(source,))
    ```
    """

    def __init__(
        self,
        zed_init_params: sl.InitParameters | None = None,
        max_res: tuple[int, int] | None = None,
    ):
        self._timestamp = 0
        self._image = np.empty((1, 1))
        self._depth_map = np.empty((1, 1))

        if sl:
            self.cam = sl.Camera()
            self._image_mat = sl.Mat()
            self._depth_map_mat = sl.Mat()

            status = self.cam.open(zed_init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                print(repr(status))
                self.cam.close()

            cam_conf = self.cam.get_camera_information().camera_configuration
            left_calib = cam_conf.calibration_parameters.left_cam
            self.res = cam_conf.resolution

            # intrinsics as proportions of resolution
            fx = left_calib.fx / self.res.width
            fy = left_calib.fy / self.res.height
            cx = left_calib.cx / self.res.width
            cy = left_calib.cy / self.res.height
            tx = cam_conf.calibration_parameters.stereo_transform.get_translation().get()[0]

            if max_res:
                self.res = sl.Resolution(min(max_res[0], self.res.width), min(max_res[1], self.res.height))

            # scale intrinsics back up
            self._intrinsics = Intrinsics(
                cx=cx * self.res.width,
                cy=cy * self.res.height,
                fx=fx * self.res.width,
                fy=fy * self.res.height,
                tx=tx,
            )
        else:
            print("[warn] LiveSource should not be initialised without the Zed SDK")

    def __delete__(self) -> None:
        if sl:
            self.cam.close()

    def update(self) -> bool:
        """Trigger an update, grabbing a new frame if possible."""
        if sl is None:
            return False

        runtime = sl.RuntimeParameters()
        err = self.cam.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            self.cam.retrieve_image(self._image_mat, sl.VIEW.LEFT, sl.MEM.CPU, self.res)
            self.cam.retrieve_measure(self._depth_map_mat, sl.MEASURE.DEPTH, sl.MEM.CPU, self.res)
            self._timestamp = self.cam.get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns
            return True

        print("[error] while grabbing frame:", err)
        return False

    def timestamp(self) -> int:
        """Return the timestamp associatede with the image/depth data."""
        return self._timestamp

    def image(self) -> npt.NDArray:
        return cast(npt.NDArray, self._image_mat.get_data())

    def depth_map(self) -> npt.NDArray:
        return cast(npt.NDArray, self._depth_map_mat.get_data())

    def intrinsics(self) -> Intrinsics:
        return self._intrinsics

    def about(self) -> str:
        return f"live depth source ({'inactive' if sl is None else 'active'})"


class MaskMethod(metaclass=abc.ABCMeta):
    """Provides a filtering process (by default, simple HSV), for `DepthSegmentation`. Object must be callable, taking an argument of the current image mask."""

    @abc.abstractmethod
    def __call__(self, image: npt.NDArray) -> npt.NDArray:
        pass


class NoMask(MaskMethod):
    def __call__(self, image: npt.NDArray) -> npt.NDArray:
        return 255 * np.zeros((image.shape[0], image.shape[1]))


class BasicHSV(MaskMethod):
    def __init__(
        self,
        lower: tuple[int, int, int] = (0, 0, 200),
        upper: tuple[int, int, int] = (179, 50, 255),
        min_area: int = 200,
    ) -> None:
        """
        Construct a basic HSV filter.

        - `lower`/`upper`: HSV bounds
        - `min_area`: Minimum contour area in pixels to keep
        """
        self.lower = np.array(lower, dtype=np.uint8)
        self.upper = np.array(upper, dtype=np.uint8)
        self.min_area = min_area

    def __call__(self, image: npt.NDArray) -> npt.NDArray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        filtered_mask = np.zeros_like(mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > self.min_area:
                cv2.drawContours(filtered_mask, [cnt], -1, 255, thickness=cv2.FILLED)

        return filtered_mask


class DepthSegementation:
    """Handles all depth segmentation processing. Requires sources to be `.update()`d before calling `process()`."""

    def __init__(
        self,
        sources: list[tuple[DepthSource, CameraPosition]],
        grid_conf: GridConfiguration,
        processes: int = 4,
        *args: Any,
        mask_method: MaskMethod | None = None,
        ignore_mask: MaskMethod | None = None,
        **kwargs: Any,
    ) -> None:
        """
        Construct a depth segmentation pipeline.

        - `sources`: a list of pairs (tuples) of a `DepthSource` and its corresponding `CameraPosition`
        - `grid_conf`: occupancy grid configuration
        - `processes`: number of processes to split the RANSAC computation between
        - `mask_method`: (kwarg-only) supply an alternative mask procedure to mask out lane lines on the ground, default is HSV mask for white
        - `ignore_mask`: (kwarg-only) supply a MaskMethod to ignore areas of the camera frame
        """
        self.grid_conf = grid_conf
        self.mask_method = mask_method if mask_method is not None else BasicHSV()  # keyword-only
        self.ignore_mask = ignore_mask  # keyword-only

        self._sources = sources
        self._guesses = [np.array([0.0, 0.0, 0.0], dtype=float) for _ in sources]

        self.timestamps = [0 for _ in sources]
        self.masks = [np.array([]) for _ in sources]
        self.grids = [np.array([]) for _ in sources]

        self._processes = processes
        self._pool = Pool(processes) if processes > 0 else None

    def process(self, force_update: bool = False) -> bool:
        """Run the depth segmentation pipeline, with optional `force_update` to recompute occupancy grids even if new data was not received."""
        updated = force_update

        index = -1
        for source, position in self._sources:
            index += 1
            # skip data that is already processed by comparing timestamp
            if not force_update and self.timestamps[index] == source.timestamp():
                continue

            # run the various ransac functions
            hsv_mask = self.mask_method(source.image())
            depth_map = source.depth_map()
            ground_mask, px_coeffs = plane.ground_plane(
                depth_map,
                200,
                (1, 16),
                0.12,
                self._guesses[index],
                self._pool,
                self._processes,
            )
            if ground_mask is None or px_coeffs is None:
                print("warning (depseg.process): no result from ground_plane")
                return False
            if self.ignore_mask is not None:
                ground_mask |= self.ignore_mask(source.image()) == 255
            lane_mask = plane.merge_masks(ground_mask, hsv_mask)

            real_coeffs = plane.real_coeffs(px_coeffs, source.intrinsics())
            occ = occu.occ_grid(lane_mask, real_coeffs, source.intrinsics(), self.grid_conf, position)

            updated = True
            self.timestamps[index] = source.timestamp()
            self._guesses[index] = px_coeffs
            self.masks[index] = lane_mask
            self.grids[index] = occ

        return updated

    def overlap(self) -> npt.NDArray:
        """Return the combined grid of locations where all individual grids are known."""
        return cast(npt.NDArray, np.logical_and.reduce([grid != 127 for grid in self.grids]))

    def merge_simple(self, strategy: Any = np.maximum) -> npt.NDArray:
        """Apply the specified `strategy` to merge the grids. This is not optimal, `merge_grids` is recommended for competition use."""
        if len(self.grids) == 0:
            return np.array([])
        return cast(npt.NDArray, strategy.reduce(self.grids))

    def merge_grids(self) -> npt.NDArray:
        """Merge occupancy grids by granting highest priority to a camera which sees a cell to be free from any angle."""
        seen = np.where(self.merge_simple(np.maximum) == 255, 255, 127)
        blocked = np.logical_or(self.merge_simple(np.minimum) == 0, seen != 255)
        return np.where(blocked, 0, seen)
