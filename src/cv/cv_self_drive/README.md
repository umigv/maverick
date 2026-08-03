# cv-self-drive

<!-- Run `python self_drive_occ_grid.py <turn_type>` to run the corresponding turn code.
Where `<turn_type>` is either "left" or "right".

Run `python functional_tests_occ_grid.py <function_type>` to run the corresponding functional test.
Where `<function_type>` is:
* `right` for right turn
* `left` for left turn
* `pedlanechange` for pedestrian lane changing
* `curvedlanekeep` for curved lane keeping -->

`cv-depth-segmentation` has been added as a submodule. To pull updates from the submodule, run `git submodule update --init --recursive`.

# Launching ```self_drive_node```

From ```~/ros2ws``` (?), run ```colcon build --packages-up-to cv_self_drive``` to build the node. Run ```ros 2 launch cv_self_drive func_tests_occ_grid.launch.py``` to start up the node.

The ```"function_type"``` parameter is:
* ```right``` for right turn
* ```left``` for left turn
* ```pedlanechange``` for pedestrian lane changing
* ```curvedlanekeep``` for curved lane keeping

The ```"hsv_json_key"``` parameter is the key to use to grab values from ```hsv_values.json```.

# Documentation

## ```hsv.py```

### Setup

Create a file named ```hsv_values.json``` and create an entry with a key that is either a camera index or the filepath of the video you want to pull from. Do this for each souce you want to use. Create entries for each named color range you want to filter, adding bounds for hue, saturation, and value. Optionally, you can add a ```"__ZED_SETTINGS__"``` key if you want to tune a zed camera.

Example:
``` json
{
    "0": {
        "white": {
            "h_upper": 29,
            "h_lower": 0,
            "s_upper": 51,
            "s_lower": 0,
            "v_upper": 255,
            "v_lower": 137
        },
        "yellow": {
            "h_upper": 29,
            "h_lower": 0,
            "s_upper": 51,
            "s_lower": 0,
            "v_upper": 255,
            "v_lower": 137
        },
        "__ZED_SETTINGS__": {
            "BRIGHTNESS": 0,
            "CONTRAST": 0,
            "HUE": 0,
            "SATURATION": 0,
            "SHARPNESS": 0,
            "GAMMA": 1
        }
    }
}
```

### Constructor

``` python
def __init__(self, video_path: str | int, barrel_mode: bool = "YOLO")
```
```video_path``` is the name of the json key you want to grab your color ranges from. ```barrel_mode``` determines how the barrel detection operates. ```"YOLO"``` uses a YOLO model, and passing in the name of a color range will use that color range instead of YOLO.

### Tuning
``` python
def tune(self, filter_name: str, use_zed: bool = False) -> None
```
```filter_name``` is the name of the color range you want to adjust. A window will come up with trackbars along with a display of your source to allow you to tune the exact color range you want to adjust. ```use_zed``` adds the option to tune zed camera parameters.

### Grabbing Masks
``` python
def get_mask(self, frame: np.ndarray) -> tuple[np.ndarray, dict[str, np.ndarray]]
```
```frame``` is the image you want hsv to process, in the form of a numpy ```ndarray```. The function returns a tuple that contains a combined mask of all your color ranges and a dictionary mapping from color range names to their respective masks.

### Example Usage
``` python
hsv_obj = hsv("data/right_turn_cropped.mp4")

hsv_obj.tune("white")
hsv_obj.tune("yellow")

cap = cv2.VideoCapture("data/right_turn_cropped.mp4")
ret, frame = cap.read()
combined, masks = hsv_obj.get_mask(frame)

cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.imshow("Image", frame)

cv2.namedWindow("Final Mask", cv2.WINDOW_NORMAL)
cv2.imshow("Final Mask", combined)

cv2.namedWindow("Yellow Mask", cv2.WINDOW_NORMAL)
cv2.imshow("Yellow Mask", masks["yellow"])

cv2.namedWindow("White Mask", cv2.WINDOW_NORMAL)
cv2.imshow("White Mask", masks["white"])
```

## ```right_turn.py```

### To run on a video
Replace the filenames in these lines in `run(self)` with the filename of the video you want to run the algorithm on. To use a webcam, replace each filename with the number representing that webcam.
``` python
cap = cv2.VideoCapture("data/right_turn_cropped.mp4")
self.hsv_obj = hsv("data/right_turn_cropped.mp4")
```

### To run on a single frame
Call this function on your `RightTurn` object, with `hsv_indentifier` being the file to look at or the number of the camera to use (for HSV tuned values). `frame` is the OpenCV image frame to process.
``` python
def run_frame(self, hsv_indentifier, frame)
```

### Algorithm
* State 1: Drive forward, setting a constant waypoint straight ahead until we can no longer see the first yellow lane lines.
* State 2: Induce a turn to the right with a constant waypoint and guidelines until we can see the next set of yellow lane lines.
* State 3: Find the midpoint of the lane we need to enter and drive toward it until the waypoint becomes low enough.
* State 4: Look for a barrel, setting a waypoint at it. If we can't find a barrel, then we set the waypoint to the top of the lane.\

## ```left_turn.py```

## ```functional_tests/functional_test_parent.py```

```FunctionalTest``` is an abstract class meant to give a common interface for running code and storing data in functional tests. To use it, have your class inherit from ```FunctionalTest``` and ensure you are storing data in ```self.final_mask``` and ```self.waypoint```.

### Sample usage:
``` python
from functional_test_parent import FunctionalTest

class PedestrianLaneChange(FunctionalTest):
    def __init__(self):
        super().__init__()
        # Initialize any additional attributes specific to this test
    
    def state_machine(self):
        # Implement the state machine logic for pedestrian lane change

    
    def update_mask(self):
        # Update the final mask based on the current state and frame


    def run_frame(self, hsv_identifier="1", frame=None):
        self.image = frame
        self.state_machine()
        self.update_mask()

        return self.final_mask, self.waypoint
```

## ```functional_tests/curved_lane_keeping.py```

### To run on a video
Replace the filenames in these lines in `run(self)` with the filename of the video you want to run the algorithm on. To use a webcam, replace each filename with the number representing that webcam.
``` python
cap = cv2.VideoCapture("data/left_curved_road.MOV")
self.hsv_obj = hsv("data/left_curved_road.MOV", barrel_mode = self.barrel_mode)
```

### To run on a single frame
Call this function on your `CurvedLanekeeping` object, with `hsv_indentifier` being the file to look at or the number of the camera to use (for HSV tuned values). `frame` is the OpenCV image frame to process.
``` python
def run_frame(self, hsv_indentifier, frame)
```

### Algorithm
* Look for a barrel. If we find one, set the waypoint on top of it.
* Otherwise, find the topmost point of each lane line within a particular search box, setting the waypoint to be the midpoint between the two points.

### Note about searchboxes:
* Keep the bounds symmetric, as this algorithm should be able to work when turning in either direction.

## ```functional_tests/obstructed_pedestrian_detection.py```

## ```functional_tests/pedestrian_lane_changing.py```

## ```functional_tests/tire_detection_img.py```