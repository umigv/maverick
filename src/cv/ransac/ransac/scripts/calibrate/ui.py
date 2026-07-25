
import cv2
import numpy as np
import numpy.typing as npt


class CameraUI:
    """
    Merge tuner UI

    Dev Usage:
    ```python
    ui = CameraUI()
    # render() returns True when 'X' is pressed to exit
    close = ui.render(grids=[grid1, grid2], merged=merged_grid)

    # Access state directly:
    is_paused = ui.paused
    l_angle = ui.params['left']['angle'] # also x_offset, z_offset (same for 'right')
    ```

    Controls:
      - P: Pause/Unpause frame
      - Left Cam: Q/E (Rotate), W/S (Z-offset), A/D (X-offset)
      - Right Cam: U/O (Rotate), I/K (Z-offset), J/L (X-offset)
      - X: Exit
    """

    def __init__(self, panel_size=240):
        self.panel_size = panel_size
        self.window_name = "Camera Merge Tuner"

        self.params = {
            "left": {"angle": 30, "x_offset": -110, "z_offset": 60},
            "right": {"angle": -30, "x_offset": 135, "z_offset": 60}
        }
        self.paused = False

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1200, 800)

    def render(self, grids: list[npt.NDArray], merged: npt.NDArray):
        grid1 = grids[0] if len(grids) > 0 else np.zeros((10, 10))
        grid2 = grids[1] if len(grids) > 1 else np.zeros((10, 10))

        def make_panel(title, g, color):
            panel = np.ones(
                (self.panel_size, self.panel_size, 3), dtype=np.uint8) * 42
            (tw, _), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
            cv2.putText(panel, title, ((self.panel_size - tw) // 2, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)

            colored = cv2.applyColorMap(g.astype(np.uint8), cv2.COLORMAP_BONE)
            res = cv2.resize(
                colored, (self.panel_size - 52, self.panel_size - 52))
            h, w = res.shape[:2]
            y0, x0 = (self.panel_size - h) // 2 + \
                26, (self.panel_size - w) // 2
            panel[y0:y0+h, x0:x0+w] = res
            return panel

        occ_row = np.hstack([
            make_panel("Camera 1", grid1, (255, 120, 120)),
            make_panel("Camera 2", grid2, (120, 255, 120)),
            make_panel("Merged", merged, (220, 220, 220))
        ])
        width = occ_row.shape[1]

        # Title Bar
        title_bar = np.ones((60, width, 3), dtype=np.uint8) * 26
        (tw, _), _ = cv2.getTextSize(
            "Camera Merge Tuner", cv2.FONT_HERSHEY_SIMPLEX, 1.05, 1)
        cv2.putText(title_bar, "Camera Merge Tuner", ((width - tw) // 2, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.05, (245, 245, 245), 1, cv2.LINE_AA)

        # Camera Top View
        cam_view = np.ones((260, width, 3), dtype=np.uint8) * 30
        pL, pR = self.params['left'], self.params['right']
        c1 = np.array([width // 2 - 50 + pL['x_offset']
                      * 2, 140 - pL['z_offset'] * 2])
        c2 = np.array([width // 2 + 50 + pR['x_offset']
                      * 2, 140 - pR['z_offset'] * 2])

        for center, yaw_deg, color in [(c1, pL['angle'], (255, 0, 0)), (c2, pR['angle'], (0, 255, 0))]:
            forward = -np.pi / 2 - np.deg2rad(yaw_deg)
            for a in (forward - np.deg2rad(55), forward + np.deg2rad(55)):
                end = center + 110 * np.array([np.cos(a), np.sin(a)])
                cv2.line(cam_view, center.astype(
                    int), end.astype(int), color, 2)
            cv2.circle(cam_view, tuple(center.astype(int)), 5, color, -1)
        cv2.putText(cam_view, "Camera Geometry (Top View)", (30, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)

        # Controls Bar
        ctrl_bar = np.ones((80, width, 3), dtype=np.uint8) * 28
        cv2.putText(ctrl_bar, f"L-Cam (Q/E/W/S/A/D): Angle: {pL['angle']} deg   X-Off: {pL['x_offset']} mm   Z-Off: {pL['z_offset']} mm", (
            20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 120, 120), 1)
        cv2.putText(ctrl_bar, f"R-Cam (U/O/I/K/J/L):  Angle: {pR['angle']} deg   X-Off: {pR['x_offset']} mm   Z-Off: {pR['z_offset']} mm", (
            20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 255, 120), 1)

        cv2.imshow(self.window_name, np.vstack(
            [title_bar, occ_row, cam_view, ctrl_bar]))

        # Key handling
        key = cv2.waitKey(30) & 0xFF

        if key in (ord("q"), ord("Q")):
            self.params['left']['angle'] = min(
                90, self.params['left']['angle'] + 1)
        elif key in (ord("e"), ord("E")):
            self.params['left']['angle'] = max(-90,
                                               self.params['left']['angle'] - 1)
        elif key in (ord("w"), ord("W")):
            self.params['left']['z_offset'] += 10
        elif key in (ord("s"), ord("S")):
            self.params['left']['z_offset'] -= 10
        elif key in (ord("a"), ord("A")):
            self.params['left']['x_offset'] -= 10
        elif key in (ord("d"), ord("D")):
            self.params['left']['x_offset'] += 10

        elif key in (ord("u"), ord("U")):
            self.params['right']['angle'] = min(
                90, self.params['right']['angle'] + 1)
        elif key in (ord("o"), ord("O")):
            self.params['right']['angle'] = max(-90,
                                                self.params['right']['angle'] - 1)
        elif key in (ord("i"), ord("I")):
            self.params['right']['z_offset'] += 10
        elif key in (ord("k"), ord("K")):
            self.params['right']['z_offset'] -= 10
        elif key in (ord("j"), ord("J")):
            self.params['right']['x_offset'] -= 10
        elif key in (ord("l"), ord("L")):
            self.params['right']['x_offset'] += 10
        elif key in (ord("p"), ord("P")):
            self.paused = not self.paused

        return key in (ord("x"), ord("X"))
