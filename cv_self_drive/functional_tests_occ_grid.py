import sys
import pyzed.sl as sl
import os
import time
import cv2
import numpy as np
import math
import json

sys.path.append(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "cv-depth-segmentation",
                 "src")
)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PointStamped, Pose, Quaternion, Point

from cv_self_drive.functional_tests.right_turn import RightTurn
from cv_self_drive.functional_tests.left_turn import LeftTurn
from cv_self_drive.functional_tests.pedestrian_lane_changing import ReallyGoodStateMachine
from cv_self_drive.functional_tests.curved_lane_keeping import CurvedLanekeeping

def print_params(calibration_params: sl.CalibrationParameters):
    # LEFT CAMERA intrinsics
    fx_left = calibration_params.left_cam.fx
    fy_left = calibration_params.left_cam.fy
    cx_left = calibration_params.left_cam.cx
    cy_left = calibration_params.left_cam.cy

    # RIGHT CAMERA intrinsics
    fx_right = calibration_params.right_cam.fx
    fy_right = calibration_params.right_cam.fy
    cx_right = calibration_params.right_cam.cx
    cy_right = calibration_params.right_cam.cy

    # Translation (baseline) between left and right camera
    tx = calibration_params.stereo_transform.get_translation().get()[0]

    # Print results
    print("\n--- ZED Camera Calibration Parameters ---")
    print("Left Camera Intrinsics:")
    print(f"  fx = {fx_left:.3f}")
    print(f"  fy = {fy_left:.3f}")
    print(f"  cx = {cx_left:.3f}")
    print(f"  cy = {cy_left:.3f}\n")

    print("Right Camera Intrinsics:")
    print(f"  fx = {fx_right:.3f}")
    print(f"  fy = {fy_right:.3f}")
    print(f"  cx = {cx_right:.3f}")
    print(f"  cy = {cy_right:.3f}\n")

    print(f"Stereo Baseline (tx): {tx:.6f} meters")

def pixel_waypoint_to_odom(centroid, depth_map, ransac_coeffs,
                           real_coeffs, intrinsics):
    """Transform a pixel-space waypoint into odom-frame (meters).

    Pipeline:
        1. Look up depth at centroid pixel (fallback: neighbourhood median,
           then RANSAC ground-plane estimate).
        2. Back-project to camera-frame 3D via pixel_to_real()
           which uses intrinsics + depression-angle rotation.
        3. Rotate camera frame -> odom REP 103:
               x_odom =  z_cam   (forward)
               y_odom = -x_cam   (left)
               z_odom =  0       (ground plane)
    """
    cx, cy = centroid
    if cx is None or cy is None:
        return None

    px, py = int(cx), int(cy)
    h, w = depth_map.shape[:2]
    px = min(px, w - 1)
    py = min(py, h - 1)

    # --- resolve depth at the centroid pixel ---
    depth = depth_map[py, px]
    if depth <= 0 or not np.isfinite(depth):
        # fallback 1: try a small neighbourhood (should never happen)
        radius = 10
        y_lo = max(0, py - radius)
        y_hi = min(h, py + radius + 1)
        x_lo = max(0, px - radius)
        x_hi = min(w, px + radius + 1)
        patch = depth_map[y_lo:y_hi, x_lo:x_hi]
        valid = patch[(patch > 0) & np.isfinite(patch)]
        if len(valid) > 0:
            depth = float(np.median(valid))
        else:
            # fallback 2: RANSAC plane model  (1/depth = c1*x + c2*y + c3)
            c1, c2, c3 = ransac_coeffs
            denom = c1 * cx + c2 * cy + c3
            if denom > 0:
                depth = 1.0 / denom
            else:
                return None

    # --- project to camera-frame real-world coords (mm) ---
    pixel_cloud = np.array([[float(cx), float(cy), float(depth)]])
    real_pt = ransac.occu.pixel_to_real(pixel_cloud, real_coeffs, intrinsics)
    # real_pt[0] = (x_cam, y_cam, z_cam)  in mm
    #   x_cam = horizontal (right +)
    #   y_cam = height      (up +)
    #   z_cam = depth        (forward +)

    # --- camera frame -> odom (REP 103: x-fwd, y-left, z-up), mm -> m ---
    x_odom =  real_pt[0, 2] / 1000.0   # z_cam  -> forward
    y_odom = -real_pt[0, 0] / 1000.0   # -x_cam -> left
    z_odom =  0.0                       # ground-plane waypoint
    return (x_odom, y_odom, z_odom)


class SelfDriveNode(Node):
    def __init__(self, gw_mm: int, gh_mm: int, cw_mm: int):
        super().__init__('self_drive_node')

        self.declare_parameter("function_type", "right")
        self.function_type = self.get_parameter(
            "function_type").get_parameter_value().string_value
        
        self.declare_parameters("hsv_json_key", "1")
        self.hsv_json_key = self.get_parameter(
            "hsv_json_key").get_parameter_value().string_value
        
        init = sl.InitParameters()
        init.depth_mode = sl.DEPTH_MODE.NEURAL
        init.async_image_retrieval = False
        
        status = self.cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Camera open failed: {status}")
        
        self.runtime = sl.RuntimeParameters()
        
        cam_info = self.cam.get_camera_information()
        resolution = cam_info.camera_configuration.resolution
        self.w = min(720, resolution.width)
        self.h = min(404, resolution.height)
        self.low_res = sl.Resolution(self.w, self.h)

        calibration_params = cam_info.camera_configuration.calibration_parameters
        print_params(calibration_params)

        sx = self.w / float(resolution.width)
        sy = self.h / float(resolution.height)
        self.low_res = sl.Resolution(self.w, self.h)
        

        self.intrinsics = ransac.Intrinsics(
            calibration_params.left_cam.cx * sx,
            calibration_params.left_cam.cy * sy,
            calibration_params.left_cam.fx * sx,
            calibration_params.left_cam.fy * sy
        )

        drive_conf = ransac.GridConfiguration(5000, 5000, 50) # , thres=5
        block_conf = ransac.GridConfiguration(5000, 5000, 50) # , thres=1

        self.occ_pub = self.create_publisher(
            OccupancyGrid, 'occupancy_grid/raw', 10)
        self.wp_pub = self.create_publisher(PointStamped, '/goal', 10)

        self.gw_mm = gw_mm
        self.gh_mm = gh_mm
        self.cw_mm = cw_mm
        self.resolution_m = cw_mm / 1000.0
        self.ros_width = gh_mm // cw_mm
        self.ros_height = gw_mm // cw_mm

        if self.function_type == "right":
          self.function = RightTurn(debug=False)
        elif self.function_type == "left":
            self.function = LeftTurn(debug=True)
        elif self.function_type == "pedlanechange":
            self.function = ReallyGoodStateMachine()
        elif self.function_type == "curvedlanekeep":
            self.function = CurvedLanekeeping(debug=False)
        else:
            raise ValueError(f"Invalid function_type: {self.function_type}")
        
        self.image_mat = sl.Mat()
        self.depth_m = sl.Mat()

        base_dir = os.path.dirname(os.path.abspath(__file__))

        with open(str(os.path.join(base_dir, "hsv_values.json")), "r") as file:
            all_json_keys = json.load(file)
            json_dict = all_json_keys.get(self.hsv_json_key, {})

            if "__ZED_SETTINGS__" in json_dict:
                zed_settings = json_dict["__ZED_SETTINGS__"]
            else:
                print("No ZED settings found in JSON, using defaults.")
                zed_settings = {
                    "BRIGHTNESS": 5,
                    "CONTRAST": 5,
                    "HUE": 5,
                    "SATURATION": 5,
                    "SHARPNESS": 5,
                    "GAMMA": 6
                }

        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, zed_settings["BRIGHTNESS"])
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, zed_settings["CONTRAST"])
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.HUE, zed_settings["HUE"])
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, zed_settings["SATURATION"])
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, zed_settings["SHARPNESS"])
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, zed_settings["GAMMA"])

    def publish_occ_grid(self, grid_np):
        # ---- coordinate transform to REP 103 ----
        # 1. flipud  -> row 0 becomes nearest to camera
        # 2. fliplr  -> col 0 becomes rightmost
        # 3. .T      -> depth axis becomes columns (x-forward),
        #               lateral axis becomes rows   (y-left)
        ros_grid = np.flipud(np.fliplr(grid_np)).T

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        info = MapMetaData()
        info.width      = self.ros_width
        info.height     = self.ros_height
        info.resolution = self.resolution_m

        # Origin: where camera is roughly
        origin = Pose()
        origin.position = Point(
            x=0.0,
            y=-(self.gw_mm / 2.0) / 1000.0,
            z=0.0
        )
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        info.origin = origin
        msg.info = info

        # Convert internal 0/127/255 encoding -> ROS 0/-1/100
        flat = ros_grid.astype('uint8')
        ros = np.full(flat.shape, -1, dtype=np.int8)
        ros[flat == 0]   = 100  # occupied
        ros[flat == 255] = 0    # free

        msg.data = ros.flatten().tolist()
        self.occ_pub.publish(msg)

    def publish_waypoint(self, odom_xyz):
        """odom_xyz: (x, y, z) in meters, odom frame, or None."""
        if odom_xyz is None:
            return

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.point.x = odom_xyz[0]
        msg.point.y = odom_xyz[1]
        msg.point.z = odom_xyz[2]

        self.wp_pub.publish(msg)

    def run(self):
        last_publish = None
        key = 0
        start = time.time()

        while key != 113:
            err = self.cam.grab(self.runtime)
            if err <= sl.ERROR_CODE.SUCCESS:  # good to go
                # FIXME pointing camera at only the ground causing a crash
                self.cam.retrieve_image(self.image_mat, sl.VIEW.LEFT, sl.MEM.CPU, self.low_res)
                self.cam.retrieve_measure(
                    self.depth_m, sl.MEASURE.DEPTH, sl.MEM.CPU, self.low_res)
                
            image = self.image_mat.get_data()
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            depths = ransac.plane.clean_depths(self.depth_m.get_data())

            if time.time() - start < 5:
                print("Warming up the camera...")
                continue
            
            turn_mask, turn_centroid = self.function.run_frame(self.hsv_json_key, image)
            print(f"done, centroid: {turn_centroid}")

            ground_mask, pixel_coeffs = ransac.plane.ground_plane(depths)
            lane_mask = ransac.plane.merge_masks(ground_mask, turn_mask)

            real_coeffs = ransac.plane.real_coeffs(pixel_coeffs, self.intrinsics)

            rad = ransac.plane.real_angle(real_coeffs)
            full_occ = ransac.occu.occ_grid(lane_mask, real_coeffs, self.intrinsics, self.drive_conf, ransac.CameraPosition(0, 0, 0))

            self.publish_occ_grid(full_occ)
            self.publish_occ_grid(full_occ)
            now = self.get_clock().now()
            if last_publish is None or (now - last_publish).nanoseconds >= 2.0 * 1e9:     
                # print(f"Last publish {last_publish}, now {now}")           
                odom_waypoint = pixel_waypoint_to_odom(
                    turn_centroid, depths, pixel_coeffs, real_coeffs, self.intrinsics)
                print(f"Publishing waypoint at odom coords: {odom_waypoint}")
                self.publish_waypoint(odom_waypoint)
                last_publish = now

            full_occ_vis = cv2.cvtColor(full_occ, cv2.COLOR_GRAY2BGR)
            full_occ_vis = cv2.resize(
                full_occ_vis, (600, 600), interpolation=cv2.INTER_NEAREST_EXACT
            )
            cv2.imshow("occupancy grid", full_occ_vis)

            print(f"angle: {math.degrees(rad): .3f} deg")

            key = cv2.waitKey(1)

            rclpy.spin_once(self, timeout_sec=0.0)

        self.cam.close()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)

    node = SelfDriveNode(5000, 5000, 50)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()