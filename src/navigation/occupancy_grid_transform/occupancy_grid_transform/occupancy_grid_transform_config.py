from dataclasses import dataclass


@dataclass(frozen=True)
class InflationParams:
    """Parameters controlling obstacle inflation for an occupancy grid.

    Inflation is performed in grid (cell) space and consists of a fully inflated core region surrounded by an optional
    falloff region where obstacle influence decays with distance.

    Attributes:
        inflation_radius_cells: Radius (in grid cells) of the fully inflated obstacle core. All cells within this
            distance of an occupied cell are treated as maximally inflated, preventing planners from routing paths
            too close to obstacles.
        inflation_falloff_extent_cells: Extent (in grid cells) of the falloff region applied beyond the hard core.
            Obstacle influence decays with distance across this region. The total radius of obstacle influence is
            `inflation_radius_cells + inflation_falloff_extent_cells`. Set to 0 to disable the falloff region.
        inflation_decay_factor: Decay factor applied to obstacle influence in the falloff region. Values closer to
            1.0 result in slower decay and more conservative paths; values closer to 0.0 decay more aggressively.
    """

    inflation_radius_cells: int
    inflation_falloff_extent_cells: int
    inflation_decay_factor: float

    def __post_init__(self) -> None:
        if self.inflation_radius_cells < 0:
            raise ValueError("InflationParams: inflation_radius_cells must be >= 0")
        if self.inflation_falloff_extent_cells < 0:
            raise ValueError("InflationParams: inflation_falloff_extent_cells must be >= 0")
        if not (0 < self.inflation_decay_factor < 1):
            raise ValueError("InflationParams: inflation_decay_factor must be between 0 and 1")


@dataclass(frozen=True)
class OccupancyGridTransformConfig:
    """Configuration for transforming a robot-centric occupancy grid into a world-aligned ROS `nav_msgs/msg/OccupancyGrid`.

    Attributes:
        frame_id: TF frame the transformed grids are published in.
        inflation_params: Parameters controlling obstacle inflation applied to the grid prior to publishing.
        no_mans_land_inflation_params: Inflation parameters used instead of `inflation_params` while the mission
            state has `in_no_mans_land` set. Defaults to a larger fully inflated core so the robot keeps more
            distance from obstacles where lane markings are absent.
    """

    frame_id: str
    inflation_params: InflationParams = InflationParams(
        inflation_radius_cells=12,
        inflation_falloff_extent_cells=0,
        inflation_decay_factor=0.9,
    )
    no_mans_land_inflation_params: InflationParams = InflationParams(
        inflation_radius_cells=22,
        inflation_falloff_extent_cells=0,
        inflation_decay_factor=0.9,
    )
