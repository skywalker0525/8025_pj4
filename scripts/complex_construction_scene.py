from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence, Tuple


MAP_ORIGIN = (-10.0, -8.0)
MAP_SIZE = (20.0, 16.0)
MAP_RESOLUTION = 0.05

START = (-8.6, -6.2, 0.0)
GOAL = (8.5, 6.0, 1.2)

WAYPOINTS: Sequence[Tuple[str, float, float, float]] = (
    ("A", -8.6, -6.2, 0.0),
    ("P1", -7.4, -2.0, 0.0),
    ("P2", -5.1, -2.0, 1.57),
    ("P3", -5.1, 1.0, 0.0),
    ("P4", -2.2, 1.0, -1.57),
    ("P5", -2.2, -2.5, 0.0),
    ("P6", 0.6, -2.5, 1.57),
    ("P7", 0.6, 2.6, 0.0),
    ("P8", 3.2, 2.6, -1.57),
    ("P9", 3.2, -4.0, 0.0),
    ("P10", 5.8, -4.0, 1.57),
    ("P11", 5.8, 3.8, 0.0),
    ("P12", 8.4, 3.8, 1.2),
    ("B", 8.5, 6.0, 1.2),
)

MISSION_SEQUENCE = [name for name, *_ in WAYPOINTS if name != "A"]


@dataclass(frozen=True)
class BoxObstacle:
    name: str
    cx: float
    cy: float
    sx: float
    sy: float
    height: float = 1.2
    yaw: float = 0.0
    color: Tuple[float, float, float] = (0.55, 0.56, 0.58)


def _wall_segment(
    name: str,
    x: float,
    y0: float,
    y1: float,
    width: float = 0.16,
    color: Tuple[float, float, float] = (0.72, 0.74, 0.76),
) -> BoxObstacle:
    return BoxObstacle(name, x, 0.5 * (y0 + y1), width, abs(y1 - y0), 1.55, 0.0, color)


def _h_segment(
    name: str,
    x0: float,
    x1: float,
    y: float,
    width: float = 0.16,
    color: Tuple[float, float, float] = (0.72, 0.74, 0.76),
) -> BoxObstacle:
    return BoxObstacle(name, 0.5 * (x0 + x1), y, abs(x1 - x0), width, 1.55, 0.0, color)


def boundary_obstacles() -> List[BoxObstacle]:
    return [
        BoxObstacle("north_wall", 0.0, 8.05, 20.4, 0.16, 2.0, 0.0, (0.78, 0.79, 0.80)),
        BoxObstacle("south_wall", 0.0, -8.05, 20.4, 0.16, 2.0, 0.0, (0.78, 0.79, 0.80)),
        BoxObstacle("east_wall", 10.05, 0.0, 0.16, 16.4, 2.0, 0.0, (0.78, 0.79, 0.80)),
        BoxObstacle("west_wall", -10.05, 0.0, 0.16, 16.4, 2.0, 0.0, (0.78, 0.79, 0.80)),
    ]


def internal_obstacles() -> List[BoxObstacle]:
    walls = [
        _wall_segment("maze_wall_1_south", -6.2, -7.9, -3.0),
        _wall_segment("maze_wall_1_north", -6.2, -1.0, 7.9),
        _wall_segment("maze_wall_2_south", -3.5, -7.9, 0.0),
        _wall_segment("maze_wall_2_north", -3.5, 2.0, 7.9),
        _wall_segment("maze_wall_3_south", -0.8, -7.9, -3.5),
        _wall_segment("maze_wall_3_north", -0.8, -1.5, 7.9),
        _wall_segment("maze_wall_4_south", 1.9, -7.9, 1.6),
        _wall_segment("maze_wall_4_north", 1.9, 3.6, 7.9),
        _wall_segment("maze_wall_5_south", 4.6, -7.9, -5.0),
        _wall_segment("maze_wall_5_north", 4.6, -3.0, 7.9),
        _wall_segment("maze_wall_6_south", 7.3, -7.9, 2.8),
        _wall_segment("maze_wall_6_north", 7.3, 4.8, 7.9),
        _h_segment("service_bay_partition_west_a", -9.2, -8.25, -4.65),
        _h_segment("service_bay_partition_west_b", -6.95, -6.2, -4.65),
        _h_segment("service_bay_partition_mid", -3.5, -0.8, 4.95),
        _h_segment("inspection_partition_east_a", 1.9, 2.4, 0.55),
        _h_segment("inspection_partition_east_b", 4.0, 4.6, 0.55),
    ]

    clutter = [
        BoxObstacle("material_stack_a", -8.15, -0.35, 0.95, 1.10, 0.85, 0.18, (0.55, 0.45, 0.31)),
        BoxObstacle("material_stack_b", -7.85, 5.45, 1.25, 0.75, 0.95, -0.28, (0.62, 0.52, 0.38)),
        BoxObstacle("tool_chest_west", -4.55, -5.85, 0.85, 0.55, 0.70, -0.20, (0.20, 0.42, 0.58)),
        BoxObstacle("pipe_bundle_1", -4.95, -0.40, 0.38, 1.60, 0.45, 0.05, (0.36, 0.37, 0.39)),
        BoxObstacle("rebar_cage_1", -2.10, 4.30, 1.10, 0.55, 1.10, 0.40, (0.28, 0.30, 0.33)),
        BoxObstacle("temporary_fence_1", -2.05, -5.35, 1.55, 0.12, 1.00, 0.28, (0.95, 0.58, 0.14)),
        BoxObstacle("concrete_pallet_1", 0.20, 0.30, 0.90, 0.75, 0.55, -0.35, (0.60, 0.62, 0.64)),
        BoxObstacle("scaffold_base_1", 0.55, 5.70, 1.35, 0.45, 1.20, 0.20, (0.28, 0.33, 0.37)),
        BoxObstacle("lift_platform", 2.80, -6.10, 1.10, 0.75, 0.70, 0.10, (0.18, 0.46, 0.46)),
        BoxObstacle("barrier_diagonal_1", 2.95, -0.95, 1.40, 0.12, 0.95, 0.72, (0.95, 0.45, 0.16)),
        BoxObstacle("material_stack_c", 5.85, -1.20, 0.95, 1.05, 0.85, -0.15, (0.55, 0.45, 0.31)),
        BoxObstacle("temporary_fence_2", 5.85, 5.70, 1.55, 0.12, 1.00, -0.18, (0.95, 0.58, 0.14)),
        BoxObstacle("tool_chest_east", 8.45, -1.35, 0.80, 0.55, 0.70, 0.28, (0.20, 0.42, 0.58)),
        BoxObstacle("finish_feature_panel", 9.20, 6.50, 0.18, 1.20, 1.40, 0.0, (0.18, 0.44, 0.72)),
    ]
    return walls + clutter


def all_obstacles(include_boundary: bool = True) -> List[BoxObstacle]:
    obstacles = internal_obstacles()
    if include_boundary:
        obstacles = boundary_obstacles() + obstacles
    return obstacles


def point_in_box(x: float, y: float, box: BoxObstacle, margin: float = 0.0) -> bool:
    dx = x - box.cx
    dy = y - box.cy
    c = math.cos(box.yaw)
    s = math.sin(box.yaw)
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return abs(lx) <= 0.5 * box.sx + margin and abs(ly) <= 0.5 * box.sy + margin


def point_in_any_box(x: float, y: float, boxes: Iterable[BoxObstacle], margin: float = 0.0) -> bool:
    return any(point_in_box(x, y, box, margin=margin) for box in boxes)


def distance_to_box(x: float, y: float, box: BoxObstacle) -> float:
    dx = x - box.cx
    dy = y - box.cy
    c = math.cos(box.yaw)
    s = math.sin(box.yaw)
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    ox = abs(lx) - 0.5 * box.sx
    oy = abs(ly) - 0.5 * box.sy
    if ox <= 0.0 and oy <= 0.0:
        return max(ox, oy)
    return math.hypot(max(ox, 0.0), max(oy, 0.0))


def distance_to_obstacles(x: float, y: float, boxes: Iterable[BoxObstacle]) -> float:
    return min(distance_to_box(x, y, box) for box in boxes)
