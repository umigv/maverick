########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import sys
import pyzed.sl as sl
from signal import signal, SIGINT
import argparse
import os
import cv2
sys.path.append(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "cv-depth-segmentation",
                 "src")
)

import ransac.plane
import ransac.occu
import numpy as np
import math

# >>> ros2 change
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
# <<< ros2 end of change

# >>> change: import RightTurn and message types for waypoint publishing
from functional_tests.right_turn import RightTurn
from functional_tests.left_turn import LeftTurn
from geometry_msgs.msg import PointStamped, Pose, Quaternion, Point
# <<< end of change


cam = sl.Camera()


# >>> change: merged OccGridPublisher + WaypointPublisher into one node
class SelfDriveNode(Node):
    """Single ROS2 node that publishes both the occupancy grid and the
    navigation waypoint.  Everything is in the 'odom' frame using
    REP 103 conventions (x-forward, y-left, z-up).

    Occupancy grid layout
    ---------------------
    The origin is placed so that cell (0,0) corresponds to the camera
    position at the rightmost edge of the lateral field of view:
        origin.x = 0.0                    (forward = 0 at camera)
        origin.y = -(grid_width_m / 2)    (rightmost edge, negative-y = right)

    Cell values follow the ROS OccupancyGrid convention:
        0   = free
        100 = occupied
        -1  = unknown
    """

    def __init__(self, gw_mm: int, gh_mm: int, cw_mm: int):
        super().__init__('self_drive_node')

        # --- occupancy grid publisher ---
        self.occ_pub = self.create_publisher(OccupancyGrid, 'occupancy_grid/raw', 10)

        self.gw_mm = gw_mm
        self.gh_mm = gh_mm
        self.cw_mm = cw_mm
        self.resolution_m = cw_mm / 1000.0

        # After the transpose the ROS grid dimensions swap:
        #   ros_width  (columns, forward) = original H = gh / cw
        #   ros_height (rows,    lateral) = original W = gw / cw
        self.ros_width  = gh_mm // cw_mm
        self.ros_height = gw_mm // cw_mm

        # --- waypoint publisher ---
        self.wp_pub = self.create_publisher(PointStamped, '/goal', 10)

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
# <<< end of change


# >>> change: convert pixel waypoint to odom-frame meters using intrinsics + RANSAC
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
# <<< end of change


# Handler to deal with CTRL+C properly
def handler(signal_received, frame):
    cam.disable_recording()
    cam.close()
    sys.exit(0)


signal(SIGINT, handler)


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

# turn_type can be "right" or "left"
def main(turn_type="right"):
    # >>> ros2 change
    rclpy.init()
    # <<< ros2 end of change

    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.async_image_retrieval = False
    # This parameter can be used to record SVO in camera FPS even if  the grab loop is running at a lower FPS (due to compute for ex.)

    status = cam.open(init)

    if status != sl.ERROR_CODE.SUCCESS:
        print("Camera Open", status, "Exit program.")
        exit(1)

    runtime = sl.RuntimeParameters()
    frames_recorded = 0

    resolution = cam.get_camera_information().camera_configuration.resolution
    w = min(720, resolution.width)
    h = min(404, resolution.height)
    low_res = sl.Resolution(w, h)

    cam_info = cam.get_camera_information()
    calibration_params = cam_info.camera_configuration.calibration_parameters

    print_params(calibration_params)

    # >>> change: intrinsics scaling based on image resolution
    full_w = resolution.width
    full_h = resolution.height

    sx = w / float(full_w)
    sy = h / float(full_h)

    cx_full = calibration_params.left_cam.cx
    cy_full = calibration_params.left_cam.cy
    fx_full = calibration_params.left_cam.fx
    fy_full = calibration_params.left_cam.fy

    cx_scaled = cx_full * sx
    cy_scaled = cy_full * sy
    fx_scaled = fx_full * sx
    fy_scaled = fy_full * sy

    intrinsics = ransac.Intrinsics(cx_scaled, cy_scaled, fx_scaled, fy_scaled)
    # <<< end of change

    drive_conf = ransac.GridConfiguration(5000, 5000, 50) # , thres=5
    block_conf = ransac.GridConfiguration(5000, 5000, 50) # , thres=1

    # >>> change: single node for both occ grid and waypoint
    node = SelfDriveNode(
        gw_mm=drive_conf.gw, gh_mm=drive_conf.gh, cw_mm=drive_conf.cw)
    # <<< end of change

    turn = None
    if(turn_type == "right"):
        turn = RightTurn(debug=False)
    elif(turn_type == "left"):
        turn = LeftTurn(debug=False)
    else:
        print(f"Invalid turn type: {turn_type}. Must be 'right' or 'left'.")
        exit(1)
    hsv_identifier = "1"
    # # >>> change: initialize RightTurn module
    # right_turn = RightTurn(debug=False)
    # hsv_identifier = "1"
    # # <<< end of change

    image_mat = sl.Mat()
    depth_m = sl.Mat()

    key = 0
    while key != 113:  # for 'q' key
        err = cam.grab(runtime)
        if err <= sl.ERROR_CODE.SUCCESS:  # good to go
            # FIXME pointing camera at only the ground causing a crash
            cam.retrieve_image(image_mat, sl.VIEW.LEFT, sl.MEM.CPU, low_res)
            cam.retrieve_measure(
                depth_m, sl.MEASURE.DEPTH, sl.MEM.CPU, low_res)

            image = image_mat.get_data()
            depths = ransac.plane.clean_depths(depth_m.get_data())

            # >>> change: run RightTurn on the frame to get final mask + pixel waypoint
            turn_mask, turn_centroid = turn.run_frame(hsv_identifier, image)
            # <<< end of change

            # >>> change: Using new efficient ransac library
            ground_mask, pixel_coeffs = ransac.plane.ground_plane(depths)
            lane_mask = ransac.plane.merge_masks(ground_mask, turn_mask)
            real_coeffs = ransac.plane.real_coeffs(pixel_coeffs, intrinsics)
            rad = ransac.plane.real_angle(real_coeffs)
            full_occ = ransac.occu.occ_grid(lane_mask, real_coeffs, intrinsics, drive_conf, ransac.CameraPosition(0, 0, 0))

            # >>> change: publish occ grid and waypoint from single node
            node.publish_occ_grid(full_occ)

            odom_waypoint = pixel_waypoint_to_odom(
                turn_centroid, depths, pixel_coeffs, real_coeffs, intrinsics)
            node.publish_waypoint(odom_waypoint)
            # <<< end of change

            full_occ_vis = cv2.cvtColor(full_occ, cv2.COLOR_GRAY2BGR)
            full_occ_vis = cv2.resize(
                full_occ_vis, (600, 600), interpolation=cv2.INTER_NEAREST_EXACT
            )
            cv2.imshow("occupancy grid", full_occ_vis)

            print(f"angle: {math.degrees(rad): .3f} deg")

            key = cv2.waitKey(1)

            rclpy.spin_once(node, timeout_sec=0.0)

        else:
            print("Grab ZED : ", err)
            break
    cv2.destroyAllWindows()
    cam.close()

    # >>> ros2 change
    rclpy.shutdown()
    # <<< ros2 end of change


if __name__ == "__main__":
    # get command line arg to decide which turn
    turn_type = "right"
    if len(sys.argv) > 1:
        turn_type = sys.argv[1].lower()
    
    main(turn_type=turn_type)