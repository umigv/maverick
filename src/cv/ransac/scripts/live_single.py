import cv2
import pyzed.sl as sl
from ransac.common import CameraPosition, GridConfiguration
from ransac.pipeline import DepthSegementation, LiveSource


def run_ransac_on_zed(cam_pos=None, serial_number=None):
    if cam_pos is None:
        cam_pos = CameraPosition()
    init = sl.InitParameters()
    if serial_number is not None:
        init.set_from_serial_number(serial_number)
    init.async_image_retrieval = True
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30  # The framerate is lowered to avoid any USB3 bandwidth issues

    live = LiveSource(init, (720, 404))
    conf = GridConfiguration(5000.0, 5000.0, 50.0)
    depseg = DepthSegementation([(live, cam_pos)], conf)

    key = 0
    while key != 113:  # 'q' key
        key = cv2.waitKey(1)

        if not live.update():
            break

        updated = depseg.process()
        if not updated:
            continue
        occ = depseg.merge_simple()

        occ_img = cv2.cvtColor(occ, cv2.COLOR_GRAY2BGR)
        occ_img = cv2.resize(occ_img, (600, 600), interpolation=cv2.INTER_NEAREST_EXACT)
        cv2.imshow("occupancy grid", occ_img)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_ransac_on_zed()
