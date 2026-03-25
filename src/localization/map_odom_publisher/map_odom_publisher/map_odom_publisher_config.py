from dataclasses import dataclass


@dataclass(frozen=True)
class MapOdomPublisherConfig:
    """Config for MapOdomPublisher.

    Attributes:
        map_frame_id: TF frame ID for the map frame.
        odom_frame_id: TF frame ID for the odom frame.
        base_frame_id: TF frame ID for the robot base frame.
        publish_period_s: Timer period for broadcasting the TF
    """

    map_frame_id: str = "map"
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"
    publish_period_s: float = 0.1
