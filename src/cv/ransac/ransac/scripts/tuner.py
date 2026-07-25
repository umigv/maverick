import h5py
import numpy as np

import math

from calibrate.ui import CameraUI
import ransac as rsc

try:
    import pyzed.sl as sl
except ImportError:
    print("[warn] pyzed not found, LiveSource will not work")


def cam_init(ser=None):
    init = sl.InitParameters()
    if ser is not None:
        init.set_from_serial_number(ser)
    init.async_image_retrieval = True
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.camera_resolution = sl.RESOLUTION.VGA
    init.camera_fps = 30
    return init


def print_pos(ui_params: dict[str, dict[str, int]]):
    left_x = ui_params['left']['x_offset']
    left_z = ui_params['left']['z_offset']
    left_r = math.radians(ui_params['left']['angle'])
    print(f"left: rsc.CameraPosition({left_x}, {left_z}, {left_r})")

    right_x = ui_params['right']['x_offset']
    right_z = ui_params['right']['z_offset']
    right_r = math.radians(ui_params['right']['angle'])
    print(f"right: rsc.CameraPosition({right_x}, {right_z}, {right_r})")


def tune_live():
    left_ser = 39394535
    right_ser = 36466710

    left = rsc.LiveSource(cam_init(left_ser))
    right = rsc.LiveSource(cam_init(right_ser))
    conf = rsc.GridConfiguration(5000.0, 5000.0, 50.0)
    depseg = rsc.DepthSegementation([
        (left, rsc.CameraPosition(-110, 60, 0.52)),
        (right, rsc.CameraPosition(135, 60, -0.52))
    ], conf, mask_method=rsc.BasicHSV())

    ui = CameraUI()

    while True:
        if not ui.paused:
            left.update()
            right.update()

        depseg.process(force_update=True)

        depseg._sources = [
            (depseg._sources[0][0], rsc.CameraPosition(
                ui.params['left']['x_offset'],
                ui.params['left']['z_offset'],
                math.radians(ui.params['left']['angle'])
            )),
            (depseg._sources[1][0], rsc.CameraPosition(
                 ui.params['right']['x_offset'],
                ui.params['right']['z_offset'],
                math.radians(ui.params['right']['angle'])
            ))
        ]

        close = ui.render(
            grids=depseg.grids,
            merged=np.where(depseg.overlap(), depseg.merge_simple(np.logical_xor) * 255, 127)
        )

        if close:
            break
    print_pos(ui.params)


def tune_offline():
    file = h5py.File("res/dual_camera_calibration.hdf5", "r")
    left = rsc.HDF5Source(file, 0)
    right = rsc.HDF5Source(file, 1)

    conf = rsc.GridConfiguration(5000.0, 5000.0, 50.0)
    depseg = rsc.DepthSegementation([
        (left, rsc.CameraPosition(0, 0, 0)),
        (right, rsc.CameraPosition(0, 0, 0))
    ], conf)

    ui = CameraUI()

    frame = 0
    while True:
        if not ui.paused:
            # print(f"using frame number: {frame}")
            frame += 1
            if frame >= max(left.frame_count, right.frame_count):
                frame = 0
            left.use_frame(frame)
            right.use_frame(frame)

        depseg.process(force_update=True)

        depseg._sources = [
            (depseg._sources[0][0], rsc.CameraPosition(
                10 * ui.params['left']['x_offset'],
                10 * ui.params['left']['z_offset'],
                math.radians(ui.params['left']['angle'])
            )),
            (depseg._sources[1][0], rsc.CameraPosition(
                10 * ui.params['right']['x_offset'],
                10 * ui.params['right']['z_offset'],
                math.radians(ui.params['right']['angle'])
            ))
        ]

        close = ui.render(
            grids=depseg.grids,
            merged=depseg.merge_simple(np.logical_xor) * 255
        )

        if close:
            break

    print_pos(ui.params)


if __name__ == '__main__':
    if input("tune from camera input? (y/N):").lower().startswith("y"):
        tune_live()
    else:
        tune_offline()
