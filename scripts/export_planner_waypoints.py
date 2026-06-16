#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys
from typing import Dict, List, Sequence, Tuple

from benchmark_path_planners import PLANNERS, min_clearance, run_benchmark
from complex_construction_scene import GOAL, START, WAYPOINTS, all_obstacles, distance_to_obstacles


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exports" / "planner_waypoints" / "complex_construction_3d_nav"


def path_length(points: Sequence[Tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def ws_path(path: Path) -> str:
    try:
        rel_path = path.resolve().relative_to(REPO_ROOT.resolve())
        return f"/ws/{rel_path}"
    except ValueError:
        return str(path)


def waypoint_min_clearance(points: Sequence[Tuple[float, float]]) -> float:
    obstacles = all_obstacles(include_boundary=True)
    if not points:
        return math.inf
    return min(distance_to_obstacles(x, y, obstacles) for x, y in points)


def low_clearance_point_count(points: Sequence[Tuple[float, float]], min_clearance_m: float) -> int:
    obstacles = all_obstacles(include_boundary=True)
    return sum(1 for x, y in points if distance_to_obstacles(x, y, obstacles) < min_clearance_m)


def yaw_between(a: Tuple[float, float], b: Tuple[float, float], fallback: float = 0.0) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    if math.hypot(dx, dy) < 1.0e-6:
        return fallback
    return math.atan2(dy, dx)


def turn_angle(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    yaw1 = yaw_between(a, b)
    yaw2 = yaw_between(b, c)
    delta = (yaw2 - yaw1 + math.pi) % (2.0 * math.pi) - math.pi
    return abs(delta)


def simplify_path(
    points: Sequence[Tuple[float, float]],
    min_spacing: float,
    turn_threshold_rad: float,
) -> List[Tuple[float, float]]:
    if len(points) <= 2:
        return list(points)
    simplified: List[Tuple[float, float]] = [points[0]]
    distance_since_keep = 0.0
    for idx in range(1, len(points) - 1):
        prev = points[idx - 1]
        current = points[idx]
        nxt = points[idx + 1]
        distance_since_keep += math.hypot(current[0] - prev[0], current[1] - prev[1])
        keep_turn = turn_angle(prev, current, nxt) >= turn_threshold_rad
        keep_spacing = distance_since_keep >= min_spacing
        if keep_turn or keep_spacing:
            if math.hypot(current[0] - simplified[-1][0], current[1] - simplified[-1][1]) > 0.20:
                simplified.append(current)
            distance_since_keep = 0.0
    if math.hypot(points[-1][0] - simplified[-1][0], points[-1][1] - simplified[-1][1]) > 0.20:
        simplified.append(points[-1])
    return simplified


def decimate_route(
    route: Sequence[Tuple[float, float]],
    max_mission_waypoints: int,
) -> List[Tuple[float, float]]:
    if max_mission_waypoints <= 0 or len(route) <= max_mission_waypoints + 1:
        return list(route)
    keep_intermediate = max(0, max_mission_waypoints - 1)
    interior = list(route[1:-1])
    if keep_intermediate <= 0 or not interior:
        return [route[0], route[-1]]
    if keep_intermediate >= len(interior):
        return list(route)
    selected = []
    for idx in range(keep_intermediate):
        if keep_intermediate == 1:
            source_idx = len(interior) // 2
        else:
            source_idx = round(idx * (len(interior) - 1) / (keep_intermediate - 1))
        selected.append(interior[source_idx])
    return [route[0], *selected, route[-1]]


def build_nav_safe_path(
    points: Sequence[Tuple[float, float]],
    min_clearance_m: float,
    min_spacing_m: float,
    turn_threshold_rad: float,
    max_mission_waypoints: int,
) -> List[Tuple[float, float]]:
    if len(points) < 2:
        return [(START[0], START[1]), (GOAL[0], GOAL[1])]
    obstacles = all_obstacles(include_boundary=True)
    route: List[Tuple[float, float]] = [(START[0], START[1])]
    for idx in range(1, len(points) - 1):
        prev = points[idx - 1]
        current = points[idx]
        nxt = points[idx + 1]
        if distance_to_obstacles(current[0], current[1], obstacles) < min_clearance_m:
            continue
        distance_from_last = math.hypot(current[0] - route[-1][0], current[1] - route[-1][1])
        keep_spacing = distance_from_last >= min_spacing_m
        keep_turn = (
            turn_angle(prev, current, nxt) >= turn_threshold_rad
            and distance_from_last >= max(0.75, 0.5 * min_spacing_m)
        )
        if keep_spacing or keep_turn:
            route.append(current)
    if math.hypot(GOAL[0] - route[-1][0], GOAL[1] - route[-1][1]) > 0.20:
        route.append((GOAL[0], GOAL[1]))
    return decimate_route(route, max_mission_waypoints)


def build_milestone_path() -> List[Tuple[float, float]]:
    return [(x, y) for _name, x, y, _yaw in WAYPOINTS]


def write_waypoint_yaml(
    path: Path,
    map_variant: str,
    planner: str,
    points: Sequence[Tuple[float, float]],
    comments: Dict[str, str],
) -> List[str]:
    path.parent.mkdir(parents=True, exist_ok=True)
    route = list(points)
    route[0] = (START[0], START[1])
    route[-1] = (GOAL[0], GOAL[1])

    waypoint_names = ["A"]
    for idx in range(1, len(route) - 1):
        waypoint_names.append(f"W{idx:03d}")
    waypoint_names.append("B")
    mission_sequence = waypoint_names[1:]

    lines = [
        "# Generated by scripts/export_planner_waypoints.py",
        f"# source_map_variant: {map_variant}",
        f"# source_planner: {planner}",
        f"# route_points: {len(route)}",
        f"# route_length_m: {path_length(route):.3f}",
    ]
    for key, value in comments.items():
        lines.append(f"# {key}: {value}")
    lines.append("points:")
    for idx, (name, point) in enumerate(zip(waypoint_names, route)):
        if idx == 0:
            yaw = START[2]
        elif idx + 1 < len(route):
            yaw = yaw_between(point, route[idx + 1], fallback=GOAL[2])
        else:
            yaw = GOAL[2]
        lines.append(f"  {name}: {{x: {point[0]:.3f}, y: {point[1]:.3f}, yaw: {yaw:.4f}}}")

    sequence_text = ", ".join(mission_sequence)
    lines.extend(
        [
            "",
            "mission:",
            f"  goal_sequence: [{sequence_text}]",
            "  continue_on_failure: true",
            "",
            "target_object:",
            "  x: 0.20",
            "  y: 0.30",
            "  z: 0.55",
            "  radius: 0.45",
            "",
            "apriltag_target:",
            "  id: 0",
            "  family: tag36h11",
            "  x: 9.20",
            "  y: 6.50",
            "  z: 1.05",
            "  approach_x: 8.500",
            "  approach_y: 6.000",
            "  approach_yaw: 1.2000",
            "  completion_radius: 1.5",
            "",
            "orbit:",
            "  radius: 1.25",
            "  linear_speed: 0.14",
            "  image_center_tolerance: 0.1",
            "  camera_height: 0.43",
            "",
            "safety:",
            "  min_obstacle_distance: 0.45",
            "  warning_obstacle_distance: 0.75",
            "  max_linear_speed: 0.35",
            "  max_angular_speed: 1.2",
            "  max_orbit_radius_error: 0.35",
            "  pan_limit: 3.14159",
            "  tilt_lower_limit: -0.9",
            "  tilt_upper_limit: 0.7",
            "",
            "video:",
            "  fps: 20",
            "  output_dir: ~/ros_videos",
            "",
            "frames:",
            "  target_frame: map",
            "  base_frame: base_link",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")
    return mission_sequence


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export benchmark planner paths as Nav2 waypoint files for 3D Gazebo captures."
    )
    parser.add_argument("output_dir", nargs="?", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--benchmark-dir", type=Path, default=REPO_ROOT / "exports" / "planning_benchmarks" / "complex_construction_3d_nav_source")
    parser.add_argument("--map-variants", nargs="+", default=["direct_ogm", "pomp_style_ogm"])
    parser.add_argument("--planners", nargs="+", default=list(PLANNERS.keys()), choices=tuple(PLANNERS.keys()))
    parser.add_argument("--min-spacing", type=float, default=1.00)
    parser.add_argument("--nav-min-clearance", type=float, default=0.60)
    parser.add_argument("--nav-spacing", type=float, default=2.00)
    parser.add_argument("--max-nav-waypoints", type=int, default=18)
    parser.add_argument(
        "--nav-mode",
        choices=("milestones", "clearance_filter", "raw_simplified"),
        default="milestones",
        help="3D Nav2 execution waypoint policy. 'milestones' uses the scene's verified safe mission route.",
    )
    parser.add_argument("--turn-threshold-deg", type=float, default=20.0)
    parser.add_argument("--successful-only", action="store_true")
    parser.add_argument("--raw-waypoints", action="store_true", help="Alias for --nav-mode raw_simplified.")
    return parser.parse_args(argv[1:])


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    if args.raw_waypoints:
        args.nav_mode = "raw_simplified"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results, paths = run_benchmark(args.benchmark_dir)
    result_lookup: Dict[Tuple[str, str], Dict] = {
        (row["map_variant"], row["planner"]): row for row in results
    }
    summary_rows = []
    for map_variant in args.map_variants:
        for planner in args.planners:
            result = result_lookup.get((map_variant, planner))
            path_item = paths.get(f"{map_variant}:{planner}")
            if result is None or path_item is None:
                continue
            world_path = path_item["world_path"]
            if args.successful_only and not result["success"]:
                continue
            if len(world_path) < 2:
                continue
            source_min_clearance = min_clearance(world_path, all_obstacles(include_boundary=True))
            unsafe_source_points = low_clearance_point_count(world_path, args.nav_min_clearance)
            if args.nav_mode == "milestones":
                nav_path = build_milestone_path()
                nav_mode = "nav_safe_milestones"
            elif args.nav_mode == "raw_simplified":
                nav_path = simplify_path(
                    world_path,
                    min_spacing=max(0.20, args.min_spacing),
                    turn_threshold_rad=math.radians(args.turn_threshold_deg),
                )
                nav_mode = "raw_simplified"
            else:
                nav_path = build_nav_safe_path(
                    world_path,
                    min_clearance_m=max(0.0, args.nav_min_clearance),
                    min_spacing_m=max(0.20, args.nav_spacing),
                    turn_threshold_rad=math.radians(args.turn_threshold_deg),
                    max_mission_waypoints=max(1, args.max_nav_waypoints),
                )
                nav_mode = "nav_safe_clearance_filter"
            run_name = f"{map_variant}_{planner}_3d"
            waypoint_file = args.output_dir / f"{run_name}.yaml"
            nav_waypoint_min_clearance = waypoint_min_clearance(nav_path)
            comments = {
                "nav_mode": nav_mode,
                "source_path_nodes": str(len(world_path)),
                "source_path_length_m": f"{path_length(world_path):.3f}",
                "source_path_min_clearance_m": (
                    f"{source_min_clearance:.3f}" if source_min_clearance is not None else ""
                ),
                "source_points_below_nav_clearance": str(unsafe_source_points),
                "nav_min_clearance_threshold_m": f"{args.nav_min_clearance:.3f}",
                "nav_spacing_m": f"{args.nav_spacing:.3f}",
                "nav_waypoint_min_clearance_m": f"{nav_waypoint_min_clearance:.3f}",
            }
            mission_sequence = write_waypoint_yaml(
                waypoint_file,
                map_variant,
                planner,
                nav_path,
                comments,
            )
            launch_command = (
                "ros2 launch my_robot_navigation complex_construction_livo_dataset_capture.launch.py "
                f"run_name:={run_name} waypoints_file:={ws_path(waypoint_file)} "
                "sample_period_sec:=0.30 pointcloud_sample_period_sec:=0.50 pointcloud_stride:=4"
            )
            summary_rows.append(
                {
                    "map_variant": map_variant,
                    "planner": planner,
                    "planner_success": result["success"],
                    "source_path_nodes": len(world_path),
                    "waypoint_count": len(mission_sequence),
                    "source_path_length_m": f"{path_length(world_path):.3f}",
                    "source_path_min_clearance_m": (
                        f"{source_min_clearance:.3f}" if source_min_clearance is not None else ""
                    ),
                    "source_points_below_nav_clearance": unsafe_source_points,
                    "nav_mode": nav_mode,
                    "nav_min_clearance_threshold_m": f"{args.nav_min_clearance:.3f}",
                    "nav_waypoint_min_clearance_m": f"{nav_waypoint_min_clearance:.3f}",
                    "waypoint_path_length_m": f"{path_length(nav_path):.3f}",
                    "waypoints_file": ws_path(waypoint_file),
                    "run_name": run_name,
                    "launch_command": launch_command,
                }
            )

    summary_path = args.output_dir / "summary.csv"
    fieldnames = [
        "map_variant",
        "planner",
        "planner_success",
        "source_path_nodes",
        "waypoint_count",
        "source_path_length_m",
        "source_path_min_clearance_m",
        "source_points_below_nav_clearance",
        "nav_mode",
        "nav_min_clearance_threshold_m",
        "nav_waypoint_min_clearance_m",
        "waypoint_path_length_m",
        "waypoints_file",
        "run_name",
        "launch_command",
    ]
    with summary_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    readme = [
        "# Complex Construction 3D Navigation Waypoints",
        "",
        "These waypoint files are generated from `scripts/benchmark_path_planners.py` paths.",
        "By default the generated mission waypoints use `nav_safe_milestones`: planner paths remain the source for metrics, while the 3D Gazebo execution uses the scene's verified safe mission route.",
        "Use `--nav-mode clearance_filter` or `--nav-mode raw_simplified` only for research runs where Nav2 failures are acceptable.",
        "Use them with `complex_construction_livo_dataset_capture.launch.py` to run planner comparisons with the real robot model, onboard camera, overhead camera, and `/points_raw` LiDAR export.",
        "",
        "Example:",
        "",
        "```bash",
        "cd /ws",
        "source install/setup.bash",
        "ros2 launch my_robot_navigation complex_construction_livo_dataset_capture.launch.py run_name:=pomp_style_ogm_weighted_astar_3d waypoints_file:=/ws/exports/planner_waypoints/complex_construction_3d_nav/pomp_style_ogm_weighted_astar_3d.yaml sample_period_sec:=0.30 pointcloud_sample_period_sec:=0.50 pointcloud_stride:=4",
        "```",
        "",
        f"Generated `{len(summary_rows)}` waypoint files. See `summary.csv`.",
        "",
    ]
    (args.output_dir / "README.md").write_text("\n".join(readme), encoding="utf-8")
    print(args.output_dir)
    print(f"wrote {len(summary_rows)} waypoint files")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
