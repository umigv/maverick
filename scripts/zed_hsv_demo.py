import cv2
import numpy as np
from cv_self_drive.hsv import hsv
import os
import json
import pyzed.sl as sl

class ZEDDemo():
    def __init__(self, json_path: str, json_key: str | int):
        self.image = None

        self.final_mask = None
        self.mask_dict = None

        self.hsv_obj = hsv(json_key)

        self.zed_settings = None

        if os.path.exists(json_path):
            with open(json_path, "r") as file:
                all_json_keys = json.load(file)
                json_dict = all_json_keys.get(str(json_key), {})

                self.zed_settings = json_dict["__ZED_SETTINGS__"]

    def update_masks(self) -> None:
        self.final_mask, self.mask_dict = self.hsv_obj.get_mask(self.image)

    def run(self, filter_name: str) -> None:
        zed = sl.Camera()
        init_params = sl.InitParameters()

        if isinstance(self.video_path, str) and self.video_path.endswith('.svo'):
            init_params.set_from_svo_file(self.video_path)
            init_params.svo_real_time_mode = False
                
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Error opening ZED Camera: {err}")
            return
        
        print("Applying custom ZED video settings...")
        zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, self.zed_settings["BRIGHTNESS"])
        zed.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, self.zed_settings["CONTRAST"])
        zed.set_camera_settings(sl.VIDEO_SETTINGS.HUE, self.zed_settings["HUE"])
        zed.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, self.zed_settings["SATURATION"])
        zed.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, self.zed_settings["SHARPNESS"])
        zed.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, self.zed_settings["GAMMA"])

        image_zed = sl.Mat()

        while True:
            err = zed.grab()
            if err == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_zed, sl.VIEW.LEFT)
                frame = image_zed.get_data()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                zed.set_svo_position(0)
                continue
            else:
                break
            
            self.image = frame
            self.update_masks()

            cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
            cv2.imshow("Video", self.image)
            cv2.namedWindow("Combined Mask", cv2.WINDOW_NORMAL)
            cv2.imshow("Combined Mask", self.final_mask)
            cv2.namedWindow(f"Filter: {filter_name}", cv2.WINDOW_NORMAL)
            cv2.imshow(f"Filter: {filter_name}", self.mask_dict[filter_name])

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # Press 'Esc' to exit the loop
                break
            

if __name__ == "__main__":
    zed_demo = ZEDDemo("cv_self_drive/hsv_values.json", 0)
    zed_demo.run()