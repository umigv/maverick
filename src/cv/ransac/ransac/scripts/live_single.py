import sys
import pyzed.sl as sl
from signal import signal, SIGINT
import argparse
import os
import cv2
import numpy as np
import math

from multiprocessing import Pool

import ransac as rsc


def run_ransac_on_zed(cam_pos=rsc.CameraPosition(), serial_number=None):
    init = sl.InitParameters()
    if serial_number is not None:
        init.set_from_serial_number(serial_number)
    init.async_image_retrieval = True
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30  # The framerate is lowered to avoid any USB3 bandwidth issues

    live = rsc.LiveSource(init, (720, 404))
    conf = rsc.GridConfiguration(5000.0, 5000.0, 50.0)
    depseg = rsc.DepthSegementation([(live, cam_pos)], conf)
    
    key = 0
    while key != 113:  # for 'q' key
        if not live.update():
            break
        
        updated = depseg.process()
        if updated:
            occ = depseg.merge_simple()

        # convert occupancy grid to image format
        occ_img = cv2.cvtColor(occ, cv2.COLOR_GRAY2BGR)
        occ_img = cv2.resize(
            occ_img, (600, 600), interpolation=cv2.INTER_NEAREST_EXACT
        )
        cv2.imshow("occupancy grid", occ_img)
        key = cv2.waitKey(1)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_ransac_on_zed()
