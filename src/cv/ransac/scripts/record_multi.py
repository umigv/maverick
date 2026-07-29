import signal
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import h5py
import numpy as np
import numpy.typing as npt
import pyzed.sl as sl

zed_list: list[sl.Camera] = []
left_list: list[sl.Mat] = []
depth_list: list[sl.Mat] = []
timestamp_list: list[int] = []
thread_list: list[threading.Thread] = []
stop_signal = False

resolution_list: list[sl.Resolution] = []

# maps camera serial numbers to respective data
timestamps: dict[str, list[int]] = {}
images: dict[str, list[npt.NDArray]] = {}
depths: dict[str, list[npt.NDArray]] = {}


def signal_handler(signal: Any, frame: Any) -> None:
    global stop_signal
    stop_signal = True
    time.sleep(0.5)
    exit()


def grab_run(index: int) -> None:
    global stop_signal
    global zed_list
    global timestamp_list
    global left_list
    global depth_list

    runtime = sl.RuntimeParameters()
    while not stop_signal:
        err = zed_list[index].grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed_list[index].retrieve_image(left_list[index], sl.VIEW.LEFT, sl.MEM.CPU, resolution_list[index])
            zed_list[index].retrieve_measure(depth_list[index], sl.MEASURE.DEPTH, sl.MEM.CPU, resolution_list[index])
            timestamp_list[index] = zed_list[index].get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns
        time.sleep(0.001)  # 1ms
    zed_list[index].close()


def main() -> None:
    global stop_signal
    global zed_list
    global left_list
    global depth_list
    global timestamp_list
    global thread_list
    global resolution_list
    global resolution_d_list
    signal.signal(signal.SIGINT, signal_handler)

    print("Running...")
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30  # The framerate is lowered to avoid any USB3 bandwidth issues

    # List and open cameras
    serial_list = []
    last_ts_list = []
    cameras = sl.Camera.get_device_list()
    index = 0
    for cam in cameras:
        init.set_from_serial_number(cam.serial_number)
        serial_list.append(f"{cam.serial_number}")
        print(f"Opening {serial_list[index]}")
        zed_list.append(sl.Camera())
        left_list.append(sl.Mat(mat_type=sl.MAT_TYPE.U8_C3))
        depth_list.append(sl.Mat())
        timestamp_list.append(0)
        last_ts_list.append(0)
        status = zed_list[index].open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            zed_list[index].close()

        resolution = zed_list[index].get_camera_information().camera_configuration.resolution
        resolution_list.append(sl.Resolution(min(720, resolution.width), min(404, resolution.height)))

        timestamps[f"{cam.serial_number}"] = []
        images[f"{cam.serial_number}"] = []
        depths[f"{cam.serial_number}"] = []

        index = index + 1

    # Start camera threads
    for index in range(len(zed_list)):
        if zed_list[index].is_opened():
            thread_list.append(threading.Thread(target=grab_run, args=(index,)))
            thread_list[index].start()

    # Display camera images
    key = 0
    while key != 113:  # for 'q' key
        key = cv2.waitKey(10)
        for index in range(len(zed_list)):
            if zed_list[index].is_opened() and timestamp_list[index] > last_ts_list[index]:
                serial = serial_list[index]
                timestamps[serial].append(timestamp_list[index])
                images[serial].append(left_list[index].get_data()[:, :, :3].copy())
                depths[serial].append(depth_list[index].get_data().copy())

                cv2.imshow(serial_list[index], left_list[index].get_data())

                # get depth at central pixel
                x = round(depth_list[index].get_width() / 2)
                y = round(depth_list[index].get_height() / 2)
                _, depth_value = depth_list[index].get_value(x, y)
                if np.isfinite(depth_value):
                    print(f"{serial_list[index]} depth at center: {round(depth_value)}MM")

                last_ts_list[index] = timestamp_list[index]

    output_dir = Path("./out/")
    if not output_dir.is_dir():
        output_dir.mkdir()

    hdf5_path = "out/multi_camera_record.hdf5"
    print(f"\nWriting to {hdf5_path}...")

    hf = h5py.File(hdf5_path, "w")
    for i, cam in enumerate(cameras):
        cam_info = zed_list[i].get_camera_information()
        calibration_params = cam_info.camera_configuration.calibration_parameters

        # LEFT CAMERA intrinsics
        fx_left = calibration_params.left_cam.fx / cam_info.camera_configuration.resolution.width
        fy_left = calibration_params.left_cam.fy / cam_info.camera_configuration.resolution.height
        cx_left = calibration_params.left_cam.cx / cam_info.camera_configuration.resolution.width
        cy_left = calibration_params.left_cam.cy / cam_info.camera_configuration.resolution.height

        # RIGHT CAMERA intrinsics
        fx_right = calibration_params.right_cam.fx / cam_info.camera_configuration.resolution.width
        fy_right = calibration_params.right_cam.fy / cam_info.camera_configuration.resolution.height
        cx_right = calibration_params.right_cam.cx / cam_info.camera_configuration.resolution.width
        cy_right = calibration_params.right_cam.cy / cam_info.camera_configuration.resolution.height

        tx = calibration_params.stereo_transform.get_translation().get()[0]

        serial_str = f"{cam.serial_number}"

        hf.create_dataset(f"inf{i}/serial", data=serial_str)
        hf.create_dataset(f"inf{i}/fx_left", data=fx_left)
        hf.create_dataset(f"inf{i}/fy_left", data=fy_left)
        hf.create_dataset(f"inf{i}/cx_left", data=cx_left)
        hf.create_dataset(f"inf{i}/cy_left", data=cy_left)
        hf.create_dataset(f"inf{i}/fx_right", data=fx_right)
        hf.create_dataset(f"inf{i}/fy_right", data=fy_right)
        hf.create_dataset(f"inf{i}/cx_right", data=cx_right)
        hf.create_dataset(f"inf{i}/cy_right", data=cy_right)
        hf.create_dataset(f"inf{i}/tx", data=tx)
        # {"serial": serial_str,
        #                                    "fx_left": fx_left, "fy_left": fy_left, "cx_left": cx_left, "cy_left": cy_left,
        #                                    "fx_right": fx_right, "fy_right": fy_right, "cx_right": cx_right, "cy_right": cy_right, "tx": tx})
        hf.create_dataset(f"tim{i}", data=timestamps[serial_str])
        hf.create_dataset(f"img{i}", data=images[serial_str], dtype="uint8", compression="gzip", compression_opts=9)
        hf.create_dataset(f"dep{i}", data=depths[serial_str], compression="gzip", compression_opts=9)
    hf.close()

    cv2.destroyAllWindows()

    # Stop the threads
    stop_signal = True
    for index in range(len(thread_list)):
        thread_list[index].join()

    print("\nFinished, remember to check the output file to confirm successful recording")


if __name__ == "__main__":
    main()
