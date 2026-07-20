from cv_self_drive.functional_tests.functional_test_parent import FunctionalTest
import os

class ObstructedPedestrianDetection(FunctionalTest):
    def __init__(self):
        super().__init__()
        self.image = None
        self.height = 0
        self.width = 0
        self.hsv_obj = None
        self.pedestrian_model = None
        self.state = 1  # Start in lane keeping state
        self.white_mask = None
        self.yellow_mask = None

    def detect_pedestrian(self):
        results = self.pedestrian_model(self.image)
        py2 = 0
        px1 = 0

        for results in results:
            boxes = results.boxes.xyxy.tolist()
            confidences = results.boxes.conf.tolist()
            class_ids = results.boxes.cls.tolist()

            for box, confidence, class_id in zip(boxes, confidences, class_ids):
                # class 0 is person (built in) and adjust confidence as needed
                px1, py1, px2, py2 = map(int, box)
                if class_id == 0 and confidence > 0.7:
                    height, width = img.shape[:2]
                    range = int(width/100)
                    size_person = (px2-px1)/width
                    cv2.rectangle(img, (px1, py1), (px2, py2), (0, 255, 0), 2)
                    cv2.waitKey(1)    
                    if size_person > 0.12:
                        # print("person within range")
                        return True, (py2 + range), px1
                    else:
                        return False, (py2 + range), px1
                    

                
        return False, py2, px1

    def calculate_waypoint(self, left_line, right_line):
        pass

    def state_machine(self):
        # State 1: lane keeping + pedestrian search
        # State 2: pedestrian detection + stopping condition + free lane detection
        # State 3: lane keeping

        if self.state == 1:
            self.waypoint = (self.width // 2, 40)
            pedestrian_detected, py2, px1 = self.detect_pedestrian()
            if pedestrian_detected:
                self.state = 2
        elif self.state == 2:
            pedestrian_detected, py2, px1 = self.detect_pedestrian()
            if pedestrian_detected:
                self.waypoint = (self.width // 2, self.height)  # Stop
            else:
                self.state = 3
        elif self.state == 3:
            self.waypoint = (self.width // 2, 40)  # Continue lane keeping

        self.update_mask()

    def update_mask(self):
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        mask, dict = self.hsv_obj.get_mask(self.image, mask)
        self.white_mask = dict['white_mask']
        self.yellow_mask = dict['yellow_mask']
        self.final_mask = mask

    def main(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))

        cap = cv2.VideoCapture(str(os.path.join(base_dir, "../data/obstructed_pedestrian.mp4")))
        self.hsv_obj = hsv(str(os.path.join(base_dir, "../data/obstructed_pedestrian.mp4")))

        while cap.isOpened():
            ret, self.image = cap.read()
            if ret:
                self.height, self.width, _ = self.image.shape
                
                self.update_mask()
                self.state_machine()

                cv2.imshow("Obstructed Pedestrian Detection", self.final_mask)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break
        cap.release()
        cv2.destroyAllWindows()

    def run_frame(self, hsv_indentifier, frame):
        if self.hsv_obj is None:
            self.hsv_obj = hsv(hsv_indentifier)
        
        self.image = frame
        self.height, self.width, _ = self.image.shape
    
        self.update_mask()
        self.state_machine()

        cv2.imshow("Obstructed Pedestrian Detection", self.final_mask)

        return self.final_mask, self.waypoint
        