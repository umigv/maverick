import colorsys
from pathlib import Path

import cv2
import numpy as np
import plotly.graph_objects as go


def create_hsv_graph(frame: cv2.typing.MatLike, num_points: int = 50000) -> None:
    """Plot HSV data of an RGB image.

    Randomly samples ```num_points``` images from the frame to plot in a 3D space. Each point is colored and placed according to its HSV coordinates. The plotly graph is outputted to ~/data/graph_test.html.
    """
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h_channel, s_channel, v_channel = cv2.split(hsv_frame)

    h = h_channel.flatten()
    s = s_channel.flatten()
    v = v_channel.flatten()

    if len(h) > num_points:
        indices = np.random.choice(len(h), num_points, replace=False)

        h = h[indices]
        s = s[indices]
        v = v[indices]

    colors = [
        f"rgb({int(r * 255)}, {int(g * 255)}, {int(b * 255)})"
        for hue, saturation, value in zip(h, s, v, strict=True)
        for r, g, b in [colorsys.hsv_to_rgb(float(hue) / 179.0, float(saturation) / 255.0, float(value) / 255.0)]
    ]

    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=h,
                y=s,
                z=v,
                mode="markers",
                marker={"size": 3, "color": colors, "opacity": 0.7},
                hovertemplate=("Hue=%{x}<br>Saturation=%{y}<br>Value=%{z}<extra></extra>"),
            )
        ]
    )

    fig.update_layout(
        title="HSV Color Space",
        scene={
            "xaxis": {"title": "Hue", "range": [0, 179]},
            "yaxis": {"title": "Saturation", "range": [0, 255]},
            "zaxis": {"title": "Value", "range": [0, 255]},
        },
    )

    output_path = Path("~/data/graph_test.html").expanduser()
    fig.write_html(output_path)

    # fig.show()


if __name__ == "__main__":
    frame = cv2.imread(Path("~/data/graph_test.png").expanduser())

    if frame is not None:
        create_hsv_graph(frame)
