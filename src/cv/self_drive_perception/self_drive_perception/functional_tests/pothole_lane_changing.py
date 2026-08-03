from typing import Any, cast

import cv2
import numpy as np
from self_drive.functional_tests.functional_test_parent import FunctionalTest
from ultralytics import YOLO
from ultralytics.engine.results import Results


class ReallyGoodStateMachine(FunctionalTest):
    def __init__(self) -> None:
        # All models are from ARV DropBox
        self.pothole_model = YOLO("../data/bestpotholemodel.pt")
        self.lines_model = YOLO("../data/best_yolov11_lane_lines.pt")
        self.barrel_model = YOLO("../data/obstacles.pt")

        # these two captures are 1 : from the google drive #9,
        # and the other is the mirrored version of the same video
        # self.cap = cv2.VideoCapture("data/pothole.mp4")
        self.cap = cv2.VideoCapture("../data/11trim.mp4")
        #
        self.y_waypoint: int = 0
        self.x_waypoint: int = 0

        self.atBarrel = False
        self.running = True
        self.one_waypoint_placed = False

        # Frame counts are for reducing frame rate``
        self.frame_count = 0
        self.process_per_frame = 3
        self.right_to_left = True

        # Values for HSV
        self.white_lower_bound = np.array([0, 0, 54])
        self.white_upper_bound = np.array([179, 37, 255])

        # State constants
        self.state_1 = 1  # pothole detection -> waits until a pothole takes up enough of the screen, then: state 2
        self.state_2 = 2  # Lane Change -> provides waypoints to change lanes until the barrel takes up enough of the screen, then: state 3
        self.state_3 = 3  # Drive forward toward barrel until we are close enough

        # starts in looking for people state
        self.state = self.state_1

        self.entered_sentinel = False
        self.exited_sentinel = False
        self.initial_frame_read = False
        self.initial_frame: cv2.typing.MatLike = np.ndarray([])

    # Determines whether a lane change should be from Left->Right or Right->Left
    # Determines this through count of white pixels
    # More white pixels on side x means leaving side x to go to lane on other side of the screen
    # ex: (less white on right side : lane change Right->Left)
    # True means right to left lane change
    def set_right_to_left(self) -> bool:

        img = self.initial_frame
        print("right to left lane change - img.shape: ", img.shape)
        _height, width = img.shape[:2]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([28, 221, 63])
        upper_yellow = np.array([68, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        cv2.imshow("yellow mask", yellow_mask)

        middle = int(width / 2)
        left = yellow_mask[:, :middle]
        right = yellow_mask[:, middle:width]
        left_count = cv2.countNonZero(left)
        right_count = cv2.countNonZero(right)

        result = cv2.bitwise_and(img, img, mask=yellow_mask)

        # Wait 1ms and check if 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        # blank_image = np.zeros((width, height, 3))

        # blank = cv2.bitwise_or(img, img, left)
        # blank_2 = cv2.bitwise_or(img_2, img_2, right)
        cv2.imshow("result", result)
        # cv2.imshow(img, "left")
        # cv2.imshow(img_2, "right")

        if left_count > right_count:
            print("Right to left= True")
            return True
        print("Right to left= False")
        return False

    def get_mask(
        self, model: Any, img: cv2.typing.MatLike, mode: str = "pothole"
    ) -> tuple[cv2.typing.MatLike, cv2.typing.MatLike]:
        results = model(img, classes=0)
        result = results[0]
        m = np.zeros(img.shape[:2], dtype=np.uint8)
        label = np.zeros(img.shape[:2], dtype=np.uint8)
        if mode == "pothole":
            results = self.pothole_model(img)
            py2 = 0
            for result in results:
                result = cast(Results, result)
                if result.boxes is None:
                    continue

                boxes = result.boxes.xyxy.tolist()
                confidences = result.boxes.conf.tolist()
                class_ids = result.boxes.cls.tolist()

                for box, _confidence, _class_id in zip(boxes, confidences, class_ids, strict=True):
                    # class 0 is barrel and adjust confidence as needed
                    px1, py1, px2, py2 = map(int, box)
                    m[py1:py2, px1:px2] = 255
                    label[py1:py2, px1:px2] = 1  # 3 indicates person class
            return m, label

        if result.masks is not None:
            # Loop through each detected object
            for mask, cls in zip(result.masks.data, result.boxes.cls, strict=True):
                if int(cls) == 0:
                    # result.masks.data is usually lower resolution,
                    # we convert to numpy and resize to match original image
                    l_ = cls.cpu().numpy()
                    m1 = cv2.resize(mask.cpu().numpy(), (img.shape[1], img.shape[0]))
                    # print(m.shape)
                    # print(m1.shape)
                    # print(type(m))
                    # print(type(m1))
                    # m = cv2.bitwise_or(m, m1)
                    m = m + m1
                    label = cv2.bitwise_or(label, l_)

            return m, label
        return m, label

    # Changes Lanes
    def change_lanes(self, img: cv2.typing.MatLike, y_in: int, prev_x: int) -> tuple[bool, int, cv2.typing.MatLike]:

        full_mask1, _lines_label = self.get_mask(self.lines_model, img, mode="lines")
        full_mask2, _barrel_label = self.get_mask(self.barrel_model, img, mode="barrel")
        full_mask3, _pothole_label = self.get_mask(self.pothole_model, img, mode="pothole")

        # full_mask = cv2.bitwise_or(full_mask1, full_mask2)
        # full_mask = cv2.bitwise_or(full_mask, full_mask3)
        full_mask = full_mask1 + full_mask2 + full_mask3

        # lines - 1, barrel - 2, pothole - 3 (for visualization purposes)
        # full_label = lines_label + barrel_label*2 + pothole_label*3
        # print(full_label)

        full_img = np.zeros(img.shape[:2], dtype=np.uint8)
        full_img[full_mask > 0.5] = 255

        lanes_mask = full_mask1
        lanes_img = np.zeros(img.shape[:2], dtype=np.uint8)
        lanes_img[lanes_mask > 0.5] = 255

        x: int = 0

        if self.right_to_left:
            x = self.find_waypoint_left(y_in, lanes_img, prev_x)
        else:
            x = self.find_waypoint_right(y_in, lanes_img, prev_x)
        done_ = False

        width = img.shape[1]
        _height, width = img.shape[:2]
        # width = width // 2
        if (x > int(width * (0.8))) and (x < (width - 150)):
            # Look for barrel being big enough = at barrel
            barrel_results = full_mask2
            for result in barrel_results:
                boxes = result.boxes.xyxy.tolist()
                confidences = result.boxes.conf.tolist()
                class_ids = result.boxes.cls.tolist()

                for box, confidence, class_id in zip(boxes, confidences, class_ids, strict=True):
                    barrel_id = 0
                    px1, py1, px2, py2 = map(int, box)
                    # cv2.rectangle(img, )
                    if class_id == barrel_id and confidence > 0.7:
                        print("BARREL")
                        _height, width = img.shape[:2]
                        size_barrel = (px2 - px1) / (width / 3)
                        cv2.rectangle(img, (px1, py1), (px2, py2), (0, 255, 0), 2)
                        if size_barrel > 0.1:
                            print("barrel within range")
                            done_ = True

        return done_, x, full_img

    # Finds pothole in lane and plots the pothole box as well as returning whether they are in range
    #
    def sees_pothole_in_lane(self, img: cv2.typing.MatLike) -> tuple[bool, int, int, cv2.typing.MatLike]:

        results = self.pothole_model(img)
        py2 = 0
        px1 = 0

        mask = np.zeros(img.shape[:2], dtype=np.uint8)

        for result in results:
            result = cast(Results, result)
            if result.boxes is None:
                continue

            boxes = result.boxes.xyxy.tolist()
            confidences = result.boxes.conf.tolist()
            class_ids = result.boxes.cls.tolist()

            for box, confidence, class_id in zip(boxes, confidences, class_ids, strict=True):
                # class 0 is barrel and adjust confidence as needed
                px1, py1, px2, py2 = map(int, box)
                if class_id == 0 and confidence > 0.7:
                    height, width = img.shape[:2]
                    range_ = int(width / 100)
                    _size_pothole = (px2 - px1) / width
                    cv2.rectangle(img, (px1, py1), (px2, py2), (0, 255, 0), 2)
                    cv2.waitKey(1)
                    # if size_pothole > 0.1: #0.12 was the initial value
                    print((height - py2) / height)
                    # print(height)
                    if ((height - py2) / height) < 0.4:  # adjust as necessary based on camera angle
                        print("pothole within range")
                        return True, (py2 + range_), px1, mask
                    return False, (py2 + range_), px1, mask

        return False, py2, px1, mask

    def at_barrel(self, capture: Any, img: cv2.typing.MatLike) -> tuple[bool, cv2.typing.MatLike, list[int]]:
        # Placeholder logic
        height, width = img.shape[:2]
        x = int(width / 2)
        y = int(height / 10)
        full_mask1, _lines_label = self.get_mask(self.lines_model, img, mode="lines")
        full_mask2, _barrel_label = self.get_mask(self.barrel_model, img, mode="barrel")

        full_mask = cv2.bitwise_or(full_mask1, full_mask2)

        return False, full_mask, [x, y]

    def add_waypoint(self, y: int, img: cv2.typing.MatLike, x: int) -> None:
        center = (x, y)
        radius = 25
        color = [255, 100, 0]
        cv2.circle(img, center, radius, color, thickness=3, lineType=8, shift=0)
        cv2.imshow("waypoint", img)

    def find_waypoint_right(self, y_in: int, img: cv2.typing.MatLike, prev_x: int) -> int:
        _height, width = img.shape
        sentinel = -100
        x = sentinel
        spacing = 10
        img_slice = img[y_in - spacing : y_in + spacing, :]

        _y_values, x_values = np.where(img_slice == 255)

        if x_values.size > 0 and np.max(x_values) > int(width / 3):
            x = np.max(x_values)
            return int(x - (width * 2 / 8))

        if x == sentinel and not self.exited_sentinel:
            self.entered_sentinel = True
            return int(width * (0.75))

        if self.entered_sentinel:
            self.exited_sentinel = True
        if self.exited_sentinel:
            return int(prev_x)
        # - (width/3)

        return 0

    def find_waypoint_left(self, y_in: int, img: cv2.typing.MatLike, prev_x: int) -> int:
        _height, width = img.shape
        sentinel = -100
        x: int = sentinel
        spacing = 10
        img_slice = img[y_in - spacing : y_in + spacing, :]

        _, x_values = np.where(img_slice == 255)

        if x_values.size > 0 and np.min(x_values) < int(width * 2 / 3):
            x = int(np.min(x_values))
            if self.entered_sentinel:
                self.exited_sentinel = True
            if x < int(width / 2):
                return int(x + (width * 2 / 8))
            self.exited_sentinel = False

        # Mirror: If nothing found, default to the left-side equivalent (30%)
        if x == sentinel and not self.exited_sentinel:
            self.entered_sentinel = True
            return int(width * 0.25)

        # Mirror: Instead of subtracting 600 (moving left),
        # add 600 to move right toward the center

        if self.exited_sentinel:
            return int(prev_x)
        #  + (width/3)

        print("RETURNING NOTHNIG")
        return x

    ## attempt at sentinel fix for certain videos

    # def find_waypoint_right(self, y_in, img, prev_x):
    #     height, width = img.shape
    #     width = width / 2 # Test use    #change before pub

    #     # Sentinel as hard-coded steer to continue lane change without visibility
    #     sentinel = -100
    #     x = sentinel
    #     spacing = 15 # Originally 10, take a larger slice to look at

    #     img_slice = img[max(0, y_in - spacing) : min(height, y_in + spacing), :]
    #     # 0, yin-spacing to ensure w/in bounds, similar logic for height

    #     y_values , x_values = np.where(img_slice == 255) # lane line

    #     if(x_values.size > 0):
    #         if(np.max(x_values) > int(width/3)):
    #             x = np.max(x_values)
    #             self.entered_sentinel = False
    #             self.exited_sentinel = False
    #             return int(x - (width * 2/8))

    #     if not self.exited_sentinel:
    #         self.entered_sentinel = True
    #         x = int(width * 0.75)
    #         return x

    #     return int(width)

    # def find_waypoint_left(self, y_in, img, prev_x):
    #     height, width = img.shape
    #     width = width / 2 # Test use
    #     # Sentinel as hard-coded steer to continue lane change without visibility
    #     sentinel = -100
    #     x = sentinel
    #     spacing = 15 # Originally 10, take a larger slice to look at

    #     img_slice = img[max(0, y_in - spacing) : min(height, y_in + spacing), :]
    #     # 0, yin-spacing to ensure w/in bounds, similar logic for height

    #     y_values , x_values = np.where(img_slice == 255) # lane line

    #     if(x_values.size > 0):
    #         if(np.min(x_values) < int(width * 2/3)): # lane in bottom 1/3
    #             # resets in case of reuse
    #             self.entered_sentinel = False
    #             self.exited_sentinel = False
    #             x = np.min(x_values)
    #             return int(x + (width * 2/8))

    #     if not self.exited_sentinel:
    #         self.entered_sentinel = True
    #         x = int(width * 0.25)
    #         return x

    #     return int(width)

    def run_frame(
        self, hsv_identifier: str = "1", frame: cv2.typing.MatLike | None = None
    ) -> tuple[cv2.typing.MatLike, tuple[int, int]]:
        if frame is not None:
            print("runframe image shape: ", frame.shape)
            _height, width = frame.shape[:2]
            # img = img[:, int(width/2) : width]
            # height, width = img.shape[:2]

            if not self.initial_frame_read:
                self.initial_frame = frame
                self.initial_frame_read = True
                self.right_to_left = self.set_right_to_left()

            prev_x = int(width / 2)
            self.frame_count += 1

            # State Logic
            if self.state == self.state_1:
                see_pothole, self.y_waypoint, self.x_waypoint, mask = self.sees_pothole_in_lane(frame)
                self.add_waypoint(self.y_waypoint, frame, self.x_waypoint)
                if see_pothole:
                    print("POTHOLE DETECTED")
                    self.state = self.state_2
                    print(self.state)
                return mask, (self.x_waypoint, self.y_waypoint)

            if self.state == self.state_2:
                ## attempt at sentinel rewrite (see above commented functions) using an actual prev_x value which is currently set to a fixed value
                # # Actually update prev_x
                # done, new_x, full_mask = self.change_lanes( img, self.y_waypoint,self.x_waypoint)

                # self.x_waypoint = new_x

                # self.add_waypoint(self.y_waypoint,img, self.x_waypoint)

                if not self.one_waypoint_placed:
                    self.add_waypoint(self.y_waypoint, frame, self.x_waypoint)
                    self.one_waypoint_placed = True
                done, self.x_waypoint, full_mask = self.change_lanes(frame, self.y_waypoint, prev_x)
                self.add_waypoint(self.y_waypoint, frame, self.x_waypoint)

                print(f"self.x_waypoint : {self.x_waypoint}")
                prev_x = self.x_waypoint
                cv2.imshow("withwaypoint", full_mask)

                if done:
                    print("state 3")
                    self.state = 3

                return full_mask, (self.x_waypoint, self.y_waypoint)

            if self.state == self.state_3:
                self.atBarrel, mask, [self.x_waypoint, self.y_waypoint] = self.at_barrel(self.cap, frame)
                if self.atBarrel:
                    # running = False
                    print("AT BARREL")

                return mask, (self.x_waypoint, self.y_waypoint)

        return (np.ndarray([]), (0, 0))

    def run(self) -> None:
        while self.running and self.cap.isOpened():
            # read frames
            ret, img = self.cap.read()
            if not ret:
                break

            _mask, _waypoint = self.run_frame("1", img)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.cap.release()
                cv2.destroyAllWindows()
                break


if __name__ == "__main__":
    machine = ReallyGoodStateMachine()
    machine.right_to_left = machine.set_right_to_left()
    machine.run()
