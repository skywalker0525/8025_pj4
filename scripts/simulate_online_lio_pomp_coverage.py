#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
import csv
import heapq
import json
import math
from pathlib import Path
import sys
import time
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from complex_construction_scene import (
    GOAL,
    MAP_ORIGIN,
    MAP_SIZE,
    START,
    all_obstacles,
    point_in_any_box,
)


UNKNOWN = -1
FREE = 0
OCCUPIED = 1

FINE_RESOLUTION = 0.05
PLAN_RESOLUTION = 0.20
POMP_SUBDIVISIONS = int(round(PLAN_RESOLUTION / FINE_RESOLUTION))
POMP_OCCUPANCY_THRESHOLD = 0.55

LIDAR_RANGE_M = 4.5
LIDAR_BEAMS = 144
LIDAR_DRAW_STRIDE = 3
LIDAR_Z_M = 0.280
CAMERA_RANGE_M = 7.0
CAMERA_FOV_RAD = math.radians(72.0)
CAMERA_BEAMS = 96
CAMERA_Z_M = 0.430

MAX_STEPS = 1300
MAX_REPLANS = 220
MAX_GOAL_REPLANS = 80
TARGET_FREE_COVERAGE = 0.60
TARGET_SURFACE_COVERAGE = 0.58
GOAL_REACHED_RADIUS_M = 0.45
FRONTIER_CANDIDATES = 14
VISIBLE_GAIN_PREFILTER = 64
VISIBLE_GAIN_BEAMS = 72
MIN_INFORMATION_GAIN = 18
SAMPLE_DISTANCE_M = 0.75
VIDEO_STRIDE = 2
VIDEO_FPS = 10.0
UNKNOWN_TRAVERSAL_PENALTY = 2.4
DEFAULT_PLANNER_TIMEOUT_SEC = 5.0
DEFAULT_THETA_TIMEOUT_SEC = 5.0
DEFAULT_MAX_EXPANDED_NODES = 50000

RUNTIME_PLANNER_TIMEOUT_SEC = DEFAULT_PLANNER_TIMEOUT_SEC
RUNTIME_THETA_TIMEOUT_SEC = DEFAULT_THETA_TIMEOUT_SEC
RUNTIME_MAX_EXPANDED_NODES = DEFAULT_MAX_EXPANDED_NODES
RUNTIME_LOG_PLANS = True

OVERHEAD_SCALE = 2
OVERHEAD_SIZE = (
    int(round(MAP_SIZE[0] / FINE_RESOLUTION)) * OVERHEAD_SCALE,
    int(round(MAP_SIZE[1] / FINE_RESOLUTION)) * OVERHEAD_SCALE,
)
ONBOARD_SIZE = (640, 360)

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exports" / "online_lio_pomp_coverage" / "complex_construction_goalcheck_pointcloud_20260616"

DIRS_8 = [
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (-1, -1, math.sqrt(2.0)),
]


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class PlanGrid:
    state: np.ndarray
    name: str
    resolution: float = PLAN_RESOLUTION
    origin: Tuple[float, float] = MAP_ORIGIN

    @property
    def rows(self) -> int:
        return int(self.state.shape[0])

    @property
    def cols(self) -> int:
        return int(self.state.shape[1])

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.cols and 0 <= y < self.rows

    def is_free(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and int(self.state[y, x]) == FREE

    def is_traversable(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and int(self.state[y, x]) != OCCUPIED

    def traversal_multiplier(self, x: int, y: int) -> float:
        if not self.in_bounds(x, y):
            return math.inf
        return UNKNOWN_TRAVERSAL_PENALTY if int(self.state[y, x]) == UNKNOWN else 1.0

    def flatten(self, x: int, y: int) -> int:
        return x + y * self.cols

    def unflatten(self, idx: int) -> Tuple[int, int]:
        return idx % self.cols, idx // self.cols

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(math.floor((x - self.origin[0]) / self.resolution)),
            int(math.floor((y - self.origin[1]) / self.resolution)),
        )

    def grid_to_world(self, x: int, y: int) -> Tuple[float, float]:
        return (
            self.origin[0] + (x + 0.5) * self.resolution,
            self.origin[1] + (y + 0.5) * self.resolution,
        )


@dataclass
class PlannerResult:
    ok: bool
    path: List[Tuple[int, int]]
    expanded: int
    runtime_ms: float
    reason: str = ""
    timed_out: bool = False
    fallback_used: bool = False
    raw_path_nodes: int = 0
    smoothed_path_nodes: int = 0
    planner_label: str = ""


@dataclass
class RunMetrics:
    algorithm: str
    planner: str = ""
    map_variant: str = ""
    success: bool = False
    reason: str = ""
    coverage_success: bool = False
    goal_reached: bool = False
    goal_distance_m: float = math.inf
    final_goal_attempts: int = 0
    steps: int = 0
    replans: int = 0
    path_length_m: float = 0.0
    free_coverage: float = 0.0
    surface_coverage: float = 0.0
    reconstruction_samples: int = 0
    observed_free_cells: int = 0
    observed_surface_cells: int = 0
    total_expanded_nodes: int = 0
    planning_runtime_ms: float = 0.0
    mean_plan_runtime_ms: float = 0.0
    failed_plans: int = 0
    planner_timeouts: int = 0
    planner_fallbacks: int = 0
    route_png: str = ""
    overhead_lidar_video: str = ""
    onboard_video: str = ""
    samples_dir: str = ""
    poses_csv: str = ""
    transforms_json: str = ""
    poses_colmap_w2c: str = ""
    camera_centers_world: str = ""
    image_name_mapping: str = ""
    lidar_points_csv: str = ""
    lidar_points_ply: str = ""
    occupied_points_ply: str = ""
    lidar_scans: int = 0
    lidar_points: int = 0


def fine_dims() -> Tuple[int, int]:
    return (
        int(round(MAP_SIZE[0] / FINE_RESOLUTION)),
        int(round(MAP_SIZE[1] / FINE_RESOLUTION)),
    )


def fine_to_world(ix: int, iy: int) -> Tuple[float, float]:
    return (
        MAP_ORIGIN[0] + (ix + 0.5) * FINE_RESOLUTION,
        MAP_ORIGIN[1] + (iy + 0.5) * FINE_RESOLUTION,
    )


def world_to_fine(x: float, y: float) -> Tuple[int, int]:
    return (
        int(math.floor((x - MAP_ORIGIN[0]) / FINE_RESOLUTION)),
        int(math.floor((y - MAP_ORIGIN[1]) / FINE_RESOLUTION)),
    )


def in_fine_bounds(ix: int, iy: int, cols: int, rows: int) -> bool:
    return 0 <= ix < cols and 0 <= iy < rows


def yaw_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)


def build_ground_truth() -> np.ndarray:
    cols, rows = fine_dims()
    obstacles = all_obstacles(include_boundary=True)
    occ = np.zeros((rows, cols), dtype=np.bool_)
    for iy in range(rows):
        y = MAP_ORIGIN[1] + (iy + 0.5) * FINE_RESOLUTION
        for ix in range(cols):
            x = MAP_ORIGIN[0] + (ix + 0.5) * FINE_RESOLUTION
            occ[iy, ix] = point_in_any_box(x, y, obstacles)
    return occ


def build_surface_mask(gt_occ: np.ndarray) -> np.ndarray:
    gt_free = ~gt_occ
    surface = np.zeros_like(gt_occ, dtype=np.bool_)
    rows, cols = gt_occ.shape
    for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        shifted_free = np.zeros_like(gt_free, dtype=np.bool_)
        y0 = max(0, dy)
        y1 = rows + min(0, dy)
        x0 = max(0, dx)
        x1 = cols + min(0, dx)
        shifted_free[y0:y1, x0:x1] = gt_free[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
        surface |= gt_occ & shifted_free
    return surface


def cast_ray(gt_occ: np.ndarray, pose: Pose2D, angle: float, max_range: float) -> Tuple[float, float, float, bool]:
    rows, cols = gt_occ.shape
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    steps = int(math.ceil(max_range / FINE_RESOLUTION))
    last_x = pose.x
    last_y = pose.y
    for step in range(1, steps + 1):
        dist = step * FINE_RESOLUTION
        x = pose.x + cos_a * dist
        y = pose.y + sin_a * dist
        ix, iy = world_to_fine(x, y)
        if not in_fine_bounds(ix, iy, cols, rows):
            return last_x, last_y, dist, False
        last_x, last_y = x, y
        if gt_occ[iy, ix]:
            return x, y, dist, True
    return last_x, last_y, max_range, False


def update_lidar_map(
    gt_occ: np.ndarray,
    known: np.ndarray,
    pose: Pose2D,
) -> Tuple[int, int, List[Tuple[float, float, bool]]]:
    rows, cols = gt_occ.shape
    rays: List[Tuple[float, float, bool]] = []
    new_free = 0
    new_occ = 0
    sx, sy = world_to_fine(pose.x, pose.y)
    if in_fine_bounds(sx, sy, cols, rows) and known[sy, sx] == UNKNOWN:
        known[sy, sx] = FREE
        new_free += 1

    for beam in range(LIDAR_BEAMS):
        angle = pose.yaw + (2.0 * math.pi * beam / LIDAR_BEAMS)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        hit = False
        end_x = pose.x
        end_y = pose.y
        steps = int(math.ceil(LIDAR_RANGE_M / FINE_RESOLUTION))
        for step in range(1, steps + 1):
            dist = step * FINE_RESOLUTION
            x = pose.x + cos_a * dist
            y = pose.y + sin_a * dist
            ix, iy = world_to_fine(x, y)
            if not in_fine_bounds(ix, iy, cols, rows):
                break
            end_x, end_y = x, y
            if gt_occ[iy, ix]:
                if known[iy, ix] != OCCUPIED:
                    new_occ += 1
                known[iy, ix] = OCCUPIED
                hit = True
                break
            if known[iy, ix] == UNKNOWN:
                new_free += 1
            if known[iy, ix] != OCCUPIED:
                known[iy, ix] = FREE
        rays.append((end_x, end_y, hit))
    return new_free, new_occ, rays


def mark_known_obstacle_patch(known: np.ndarray, ix: int, iy: int, radius_cells: int = 2) -> None:
    rows, cols = known.shape
    y0 = max(0, iy - radius_cells)
    y1 = min(rows, iy + radius_cells + 1)
    x0 = max(0, ix - radius_cells)
    x1 = min(cols, ix + radius_cells + 1)
    known[y0:y1, x0:x1] = OCCUPIED


def build_pomp_plan_grid(known: np.ndarray) -> PlanGrid:
    factor = POMP_SUBDIVISIONS
    rows = known.shape[0] // factor
    cols = known.shape[1] // factor
    state = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    total = factor * factor
    for y in range(rows):
        for x in range(cols):
            if x == 0 or y == 0 or x == cols - 1 or y == rows - 1:
                state[y, x] = OCCUPIED
                continue
            block = known[y * factor:(y + 1) * factor, x * factor:(x + 1) * factor]
            occ_points = np.argwhere(block == OCCUPIED)
            occ_count = int(occ_points.shape[0])
            free_count = int(np.count_nonzero(block == FREE))
            if occ_count == 0:
                state[y, x] = FREE if free_count >= max(1, total // 4) else UNKNOWN
                continue

            ratio = occ_count / float(total)
            if ratio >= POMP_OCCUPANCY_THRESHOLD:
                state[y, x] = OCCUPIED
                continue

            center = 0.5 * (factor - 1)
            unsafe_lim = 0.5 * factor * 0.78
            xs = occ_points[:, 1].astype(float)
            ys = occ_points[:, 0].astype(float)
            center_blocked = bool(np.any((np.abs(xs - center) <= 0.75) & (np.abs(ys - center) <= 0.75)))
            if center_blocked:
                state[y, x] = OCCUPIED
                continue
            unsafe = bool(np.any((np.abs(xs - center) >= unsafe_lim) | (np.abs(ys - center) >= unsafe_lim)))
            spans_x = bool(np.min(xs) < center and np.max(xs) > center)
            spans_y = bool(np.min(ys) < center and np.max(ys) > center)
            if unsafe and (spans_x or spans_y):
                state[y, x] = OCCUPIED
            elif free_count > 0:
                state[y, x] = FREE
            else:
                state[y, x] = UNKNOWN
    return PlanGrid(state=state, name="pomp_style_ogm")


def build_direct_plan_grid(known: np.ndarray) -> PlanGrid:
    factor = POMP_SUBDIVISIONS
    rows = known.shape[0] // factor
    cols = known.shape[1] // factor
    state = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    total = factor * factor
    for y in range(rows):
        for x in range(cols):
            if x == 0 or y == 0 or x == cols - 1 or y == rows - 1:
                state[y, x] = OCCUPIED
                continue
            block = known[y * factor:(y + 1) * factor, x * factor:(x + 1) * factor]
            occ_count = int(np.count_nonzero(block == OCCUPIED))
            free_count = int(np.count_nonzero(block == FREE))
            if occ_count > 0:
                state[y, x] = OCCUPIED
            elif free_count >= max(1, total // 4):
                state[y, x] = FREE
            else:
                state[y, x] = UNKNOWN
    return PlanGrid(state=state, name="direct_ogm")


MAP_BUILDERS: Dict[str, Callable[[np.ndarray], PlanGrid]] = {
    "direct_ogm": build_direct_plan_grid,
    "pomp_style_ogm": build_pomp_plan_grid,
}

MAP_DESCRIPTIONS = {
    "direct_ogm": "direct coarse occupancy-grid projection: any occupied fine cell marks the 0.20 m planning cell occupied",
    "pomp_style_ogm": "POMP-style sub-cell occupancy projection from 0.05 m local map to 0.20 m planning grid",
}


def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def can_step(grid: PlanGrid, x: int, y: int, dx: int, dy: int) -> bool:
    nx, ny = x + dx, y + dy
    if not grid.is_traversable(nx, ny):
        return False
    if dx != 0 and dy != 0:
        return grid.is_traversable(x + dx, y) and grid.is_traversable(x, y + dy)
    return True


def reconstruct(came_from: Dict[int, int], current: int, grid: PlanGrid) -> List[Tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return [grid.unflatten(idx) for idx in path]


def nearest_free(grid: PlanGrid, idx: Tuple[int, int], max_radius: int = 10) -> Optional[Tuple[int, int]]:
    if grid.is_traversable(*idx):
        return idx
    x0, y0 = idx
    for radius in range(1, max_radius + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                cand = (x0 + dx, y0 + dy)
                if grid.is_traversable(*cand):
                    return cand
    return None


def astar_like(
    grid: PlanGrid,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    weight: float,
    label: str,
    timeout_sec: Optional[float] = None,
    max_expanded_nodes: Optional[int] = None,
) -> PlannerResult:
    started = time.perf_counter()
    timeout_sec = RUNTIME_PLANNER_TIMEOUT_SEC if timeout_sec is None else timeout_sec
    max_expanded_nodes = RUNTIME_MAX_EXPANDED_NODES if max_expanded_nodes is None else max_expanded_nodes
    if not grid.is_traversable(*start) or not grid.is_traversable(*goal):
        return PlannerResult(
            False,
            [],
            0,
            (time.perf_counter() - started) * 1000.0,
            "start_or_goal_not_traversable",
            planner_label=label,
        )

    start_id = grid.flatten(*start)
    goal_id = grid.flatten(*goal)
    open_heap = [(0.0, 0, start_id)]
    came_from: Dict[int, int] = {}
    gscore = {start_id: 0.0}
    closed = set()
    counter = 0
    expanded = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        if expanded >= max_expanded_nodes:
            runtime = (time.perf_counter() - started) * 1000.0
            return PlannerResult(
                False,
                [],
                expanded,
                runtime,
                "max_expanded_nodes",
                planner_label=label,
            )
        if expanded % 256 == 0 and time.perf_counter() - started > timeout_sec:
            runtime = (time.perf_counter() - started) * 1000.0
            return PlannerResult(
                False,
                [],
                expanded,
                runtime,
                "timeout",
                timed_out=True,
                planner_label=label,
            )
        if current == goal_id:
            runtime = (time.perf_counter() - started) * 1000.0
            path = reconstruct(came_from, current, grid)
            return PlannerResult(
                True,
                path,
                expanded,
                runtime,
                raw_path_nodes=len(path),
                smoothed_path_nodes=len(path),
                planner_label=label,
            )
        x, y = grid.unflatten(current)
        for dx, dy, step_cost in DIRS_8:
            if not can_step(grid, x, y, dx, dy):
                continue
            nx, ny = x + dx, y + dy
            nid = grid.flatten(nx, ny)
            tentative = gscore[current] + step_cost * grid.traversal_multiplier(nx, ny)
            if tentative < gscore.get(nid, math.inf):
                came_from[nid] = current
                gscore[nid] = tentative
                counter += 1
                h = weight * heuristic((nx, ny), goal)
                heapq.heappush(open_heap, (tentative + h, counter, nid))
    runtime = (time.perf_counter() - started) * 1000.0
    return PlannerResult(False, [], expanded, runtime, "no_path", planner_label=label)


def line_of_sight(
    grid: PlanGrid,
    a: Tuple[int, int],
    b: Tuple[int, int],
    cache: Optional[Dict[Tuple[Tuple[int, int], Tuple[int, int]], bool]] = None,
) -> bool:
    key = (a, b) if a <= b else (b, a)
    if cache is not None and key in cache:
        return cache[key]
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    result = True
    while True:
        if not grid.is_traversable(x, y):
            result = False
            break
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        old_x, old_y = x, y
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
        if x != old_x and y != old_y:
            if not grid.is_traversable(x, old_y) or not grid.is_traversable(old_x, y):
                result = False
                break
    if cache is not None:
        cache[key] = result
    return result


def theta_style_shortcut_smoothing(
    grid: PlanGrid,
    path: Sequence[Tuple[int, int]],
    timeout_sec: float,
) -> Tuple[List[Tuple[int, int]], bool]:
    if len(path) <= 2:
        return list(path), False
    started = time.perf_counter()
    los_cache: Dict[Tuple[Tuple[int, int], Tuple[int, int]], bool] = {}
    smoothed = [path[0]]
    i = 0
    timed_out = False
    while i < len(path) - 1:
        if time.perf_counter() - started > timeout_sec:
            timed_out = True
            break
        best_j = i + 1
        for j in range(len(path) - 1, i, -1):
            if time.perf_counter() - started > timeout_sec:
                timed_out = True
                break
            if line_of_sight(grid, path[i], path[j], los_cache):
                best_j = j
                break
        smoothed.append(path[best_j])
        i = best_j
        if timed_out:
            break
    if timed_out:
        return list(path), True
    return smoothed, False


def theta_star_smoothed_weighted_astar(grid: PlanGrid, start: Tuple[int, int], goal: Tuple[int, int]) -> PlannerResult:
    started = time.perf_counter()
    raw = astar_like(
        grid,
        start,
        goal,
        weight=1.4,
        label="theta_star/raw_weighted_astar",
        timeout_sec=RUNTIME_PLANNER_TIMEOUT_SEC,
        max_expanded_nodes=RUNTIME_MAX_EXPANDED_NODES,
    )
    if not raw.ok:
        raw.planner_label = "theta_star/raw_weighted_astar"
        return raw

    remaining = max(0.001, RUNTIME_THETA_TIMEOUT_SEC - (time.perf_counter() - started))
    smoothed_path, smoothing_timed_out = theta_style_shortcut_smoothing(grid, raw.path, remaining)
    runtime = (time.perf_counter() - started) * 1000.0
    return PlannerResult(
        True,
        smoothed_path,
        raw.expanded,
        runtime,
        reason="shortcut_timeout_fallback_to_weighted_astar" if smoothing_timed_out else "",
        timed_out=smoothing_timed_out,
        fallback_used=smoothing_timed_out,
        raw_path_nodes=len(raw.path),
        smoothed_path_nodes=len(smoothed_path),
        planner_label="theta_star_shortcut",
    )


PLANNERS: Dict[str, Callable[[PlanGrid, Tuple[int, int], Tuple[int, int]], PlannerResult]] = {
    "dijkstra": lambda grid, start, goal: astar_like(grid, start, goal, 0.0, "dijkstra"),
    "astar": lambda grid, start, goal: astar_like(grid, start, goal, 1.0, "astar"),
    "weighted_astar": lambda grid, start, goal: astar_like(grid, start, goal, 1.4, "weighted_astar"),
    "theta_star": theta_star_smoothed_weighted_astar,
}


def integral_image(mask: np.ndarray) -> np.ndarray:
    return np.pad(mask.astype(np.int32).cumsum(axis=0).cumsum(axis=1), ((1, 0), (1, 0)))


def box_sum(integral: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> int:
    return int(integral[y1, x1] - integral[y0, x1] - integral[y1, x0] + integral[y0, x0])


def frontier_cells(grid: PlanGrid) -> List[Tuple[int, int]]:
    state = grid.state
    rows, cols = state.shape
    cells: List[Tuple[int, int]] = []
    for y in range(1, rows - 1):
        for x in range(1, cols - 1):
            if state[y, x] != FREE:
                continue
            neighborhood = state[y - 1:y + 2, x - 1:x + 2]
            if np.any(neighborhood == UNKNOWN):
                cells.append((x, y))
    return cells


def information_gain(unknown_integral: np.ndarray, wx: float, wy: float, radius_m: float = LIDAR_RANGE_M) -> int:
    cols, rows = fine_dims()
    cx, cy = world_to_fine(wx, wy)
    radius = int(math.ceil(radius_m / FINE_RESOLUTION))
    x0 = max(0, cx - radius)
    x1 = min(cols, cx + radius + 1)
    y0 = max(0, cy - radius)
    y1 = min(rows, cy + radius + 1)
    return box_sum(unknown_integral, x0, y0, x1, y1)


def visible_unknown_gain(known: np.ndarray, wx: float, wy: float) -> int:
    rows, cols = known.shape
    visible = set()
    steps = int(math.ceil(LIDAR_RANGE_M / FINE_RESOLUTION))
    for beam in range(VISIBLE_GAIN_BEAMS):
        angle = 2.0 * math.pi * beam / VISIBLE_GAIN_BEAMS
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        for step in range(1, steps + 1):
            x = wx + cos_a * step * FINE_RESOLUTION
            y = wy + sin_a * step * FINE_RESOLUTION
            ix, iy = world_to_fine(x, y)
            if not in_fine_bounds(ix, iy, cols, rows):
                break
            state = int(known[iy, ix])
            if state == OCCUPIED:
                break
            if state == UNKNOWN:
                visible.add((ix, iy))
    return len(visible)


def path_length_cells(grid: PlanGrid, path: Sequence[Tuple[int, int]]) -> float:
    if len(path) < 2:
        return 0.0
    points = [grid.grid_to_world(x, y) for x, y in path]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def select_frontier_goal(
    algorithm: str,
    grid: PlanGrid,
    known: np.ndarray,
    pose: Pose2D,
) -> Tuple[Optional[Tuple[int, int]], Optional[PlannerResult], int]:
    start = nearest_free(grid, grid.world_to_grid(pose.x, pose.y))
    if start is None:
        return None, None, 0

    unknown_integral = integral_image(known == UNKNOWN)
    rough_candidates = []
    for cell in frontier_cells(grid):
        wx, wy = grid.grid_to_world(*cell)
        gain = information_gain(unknown_integral, wx, wy)
        if gain < MIN_INFORMATION_GAIN:
            continue
        euclidean = math.hypot(wx - pose.x, wy - pose.y)
        rough_candidates.append((gain / (euclidean + 1.0), gain, euclidean, cell))
    rough_candidates.sort(reverse=True)

    candidates = []
    for _, _, euclidean, cell in rough_candidates[:VISIBLE_GAIN_PREFILTER]:
        wx, wy = grid.grid_to_world(*cell)
        gain = visible_unknown_gain(known, wx, wy)
        if gain >= MIN_INFORMATION_GAIN:
            candidates.append((gain / (euclidean + 1.0), gain, euclidean, cell))
    candidates.sort(reverse=True)

    best_score = -math.inf
    best_goal = None
    best_plan = None
    failed = 0
    for _, gain, _, cell in candidates[:FRONTIER_CANDIDATES]:
        if cell == start:
            continue
        result = PLANNERS[algorithm](grid, start, cell)
        if not result.ok or len(result.path) < 2:
            failed += 1
            continue
        length = max(path_length_cells(grid, result.path), PLAN_RESOLUTION)
        score = gain / (length + 0.75) - 0.006 * result.expanded
        if score > best_score:
            best_score = score
            best_goal = cell
            best_plan = result
    return best_goal, best_plan, failed


def log_plan(
    algorithm: str,
    replan: int,
    plan: PlannerResult,
    path_length_m: float,
    failed_candidates: int,
) -> None:
    if not RUNTIME_LOG_PLANS:
        return
    timeout = "yes" if plan.timed_out else "no"
    fallback = "yes" if plan.fallback_used else "no"
    print(
        f"  {algorithm}: replan={replan:03d} planner={plan.planner_label or algorithm} "
        f"time_ms={plan.runtime_ms:.2f} expanded={plan.expanded} "
        f"path_m={path_length_m:.2f} nodes={len(plan.path)} "
        f"raw_nodes={plan.raw_path_nodes or len(plan.path)} "
        f"smoothed_nodes={plan.smoothed_path_nodes or len(plan.path)} "
        f"timeout={timeout} fallback={fallback} failed_candidates={failed_candidates}",
        flush=True,
    )


def world_to_overhead_px(x: float, y: float) -> Tuple[int, int]:
    cols, rows = fine_dims()
    ix, iy = world_to_fine(x, y)
    return (
        int(np.clip(ix * OVERHEAD_SCALE, 0, cols * OVERHEAD_SCALE - 1)),
        int(np.clip((rows - 1 - iy) * OVERHEAD_SCALE, 0, rows * OVERHEAD_SCALE - 1)),
    )


def render_overhead(
    algorithm: str,
    known: np.ndarray,
    pose: Pose2D,
    path: Sequence[Tuple[float, float]],
    rays: Sequence[Tuple[float, float, bool]],
    metrics: RunMetrics,
    goal: Optional[Tuple[float, float]],
) -> np.ndarray:
    rows, cols = known.shape
    colors = np.zeros((rows, cols, 3), dtype=np.uint8)
    colors[known == UNKNOWN] = (74, 76, 78)
    colors[known == FREE] = (232, 232, 224)
    colors[known == OCCUPIED] = (25, 27, 30)
    img = cv2.resize(np.flipud(colors), OVERHEAD_SIZE, interpolation=cv2.INTER_NEAREST)

    start_px = world_to_overhead_px(START[0], START[1])
    goal_px = world_to_overhead_px(GOAL[0], GOAL[1])
    cv2.circle(img, start_px, 6, (60, 170, 85), -1)
    cv2.circle(img, goal_px, 6, (60, 70, 210), -1)

    robot_px = world_to_overhead_px(pose.x, pose.y)
    for end_x, end_y, hit in rays[::LIDAR_DRAW_STRIDE]:
        end_px = world_to_overhead_px(end_x, end_y)
        cv2.line(img, robot_px, end_px, (155, 200, 235), 1, cv2.LINE_AA)
        if hit:
            cv2.circle(img, end_px, 2, (45, 45, 230), -1)

    if len(path) > 1:
        pts = np.array([world_to_overhead_px(x, y) for x, y in path], dtype=np.int32)
        cv2.polylines(img, [pts], False, (30, 120, 245), 2, cv2.LINE_AA)

    if goal is not None:
        cv2.circle(img, world_to_overhead_px(goal[0], goal[1]), 5, (0, 190, 255), 2)

    cv2.circle(img, robot_px, 8, (0, 150, 255), -1)
    heading = (int(robot_px[0] + 18 * math.cos(pose.yaw)), int(robot_px[1] - 18 * math.sin(pose.yaw)))
    cv2.arrowedLine(img, robot_px, heading, (0, 70, 255), 2, tipLength=0.35)

    hud = [
        f"{algorithm} | online LIO/POMP coverage",
        f"coverage free={metrics.free_coverage:.3f} surface={metrics.surface_coverage:.3f}",
        f"goal_dist={metrics.goal_distance_m:.2f}m reached={'yes' if metrics.goal_reached else 'no'}",
        f"path={metrics.path_length_m:.1f}m samples={metrics.reconstruction_samples} replans={metrics.replans}",
    ]
    y = 22
    for line in hud:
        cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (10, 10, 10), 3, cv2.LINE_AA)
        cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 245, 245), 1, cv2.LINE_AA)
        y += 22
    return img


def render_onboard(
    algorithm: str,
    gt_occ: np.ndarray,
    pose: Pose2D,
    metrics: RunMetrics,
    include_hud: bool = True,
) -> np.ndarray:
    width, height = ONBOARD_SIZE
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[: height // 2, :] = (185, 196, 205)
    img[height // 2:, :] = (92, 96, 91)
    cv2.line(img, (0, height // 2), (width, height // 2), (130, 138, 142), 2)

    for i in range(CAMERA_BEAMS):
        rel = -0.5 * CAMERA_FOV_RAD + CAMERA_FOV_RAD * i / max(1, CAMERA_BEAMS - 1)
        angle = pose.yaw + rel
        _, _, dist, hit = cast_ray(gt_occ, pose, angle, CAMERA_RANGE_M)
        if not hit:
            continue
        x0 = int(i * width / CAMERA_BEAMS)
        x1 = int((i + 1) * width / CAMERA_BEAMS) + 1
        bar_h = int(min(height * 0.88, height * 1.35 / max(0.35, dist)))
        y0 = max(0, height // 2 - bar_h // 2)
        y1 = min(height - 1, height // 2 + bar_h // 2)
        shade = int(np.clip(230 - 24 * dist, 70, 225))
        color = (shade - 20, shade - 12, shade)
        cv2.rectangle(img, (x0, y0), (x1, y1), color, -1)
        cv2.line(img, (x0, y0), (x0, y1), (45, 48, 55), 1)

    cv2.rectangle(img, (0, height - 54), (width, height), (25, 28, 30), -1)
    cv2.putText(img, "simulated onboard RGB view", (12, height - 31), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 235, 235), 1, cv2.LINE_AA)
    cv2.putText(img, f"pose=({pose.x:.2f},{pose.y:.2f}) yaw={pose.yaw:.2f}", (12, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (210, 220, 230), 1, cv2.LINE_AA)
    if include_hud:
        cv2.putText(img, algorithm, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 3, cv2.LINE_AA)
        cv2.putText(img, algorithm, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (250, 250, 250), 1, cv2.LINE_AA)
        cv2.putText(img, f"free {metrics.free_coverage:.3f} | surface {metrics.surface_coverage:.3f}", (12, 51), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (20, 20, 20), 3, cv2.LINE_AA)
        cv2.putText(img, f"free {metrics.free_coverage:.3f} | surface {metrics.surface_coverage:.3f}", (12, 51), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (250, 250, 250), 1, cv2.LINE_AA)
    return img


def write_sample(
    algorithm_dir: Path,
    algorithm: str,
    gt_occ: np.ndarray,
    pose: Pose2D,
    metrics: RunMetrics,
    sample_rows: List[Dict[str, str]],
) -> None:
    images_dir = algorithm_dir / "samples" / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    image_name = f"{len(sample_rows) + 1:06d}.png"
    frame = render_onboard(algorithm, gt_occ, pose, metrics, include_hud=False)
    cv2.imwrite(str(images_dir / image_name), frame)
    qw, qx, qy, qz = yaw_quaternion(pose.yaw)
    sample_rows.append(
        {
            "image_name": image_name,
            "sample_index": str(len(sample_rows) + 1),
            "x": f"{pose.x:.6f}",
            "y": f"{pose.y:.6f}",
            "z": f"{CAMERA_Z_M:.6f}",
            "yaw": f"{pose.yaw:.6f}",
            "qw": f"{qw:.9f}",
            "qx": f"{qx:.9f}",
            "qy": f"{qy:.9f}",
            "qz": f"{qz:.9f}",
            "free_coverage": f"{metrics.free_coverage:.6f}",
            "surface_coverage": f"{metrics.surface_coverage:.6f}",
        }
    )
    metrics.reconstruction_samples = len(sample_rows)


def camera_to_world_matrix(row: Dict[str, str]) -> List[List[float]]:
    x = float(row["x"])
    y = float(row["y"])
    z = float(row["z"])
    yaw = float(row["yaw"])
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [
        [c, -s, 0.0, x],
        [s, c, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def write_pose_sidecars(samples_dir: Path, sample_rows: Sequence[Dict[str, str]]) -> Dict[str, Path]:
    width, height = ONBOARD_SIZE
    focal = 0.5 * width / math.tan(0.5 * CAMERA_FOV_RAD)
    mapping_path = samples_dir / "image_name_mapping.csv"
    centers_path = samples_dir / "camera_centers_world.txt"
    colmap_path = samples_dir / "poses_colmap_w2c.txt"
    transforms_path = samples_dir / "transforms.json"

    with mapping_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["image_name", "relative_path"])
        writer.writeheader()
        for row in sample_rows:
            writer.writerow({"image_name": row["image_name"], "relative_path": f"images/{row['image_name']}"})

    with centers_path.open("w", encoding="utf-8") as handle:
        handle.write("# image_name Cx Cy Cz\n")
        for row in sample_rows:
            handle.write(f"{row['image_name']} {row['x']} {row['y']} {row['z']}\n")

    with colmap_path.open("w", encoding="utf-8") as handle:
        handle.write("# image_name qw qx qy qz tx ty tz\n")
        for row in sample_rows:
            yaw = float(row["yaw"])
            x = float(row["x"])
            y = float(row["y"])
            z = float(row["z"])
            c = math.cos(yaw)
            s = math.sin(yaw)
            qw, qx, qy, qz = yaw_quaternion(-yaw)
            tx = -c * x - s * y
            ty = s * x - c * y
            tz = -z
            handle.write(
                f"{row['image_name']} {qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f} "
                f"{tx:.9f} {ty:.9f} {tz:.9f}\n"
            )

    frames = []
    for row in sample_rows:
        frames.append(
            {
                "file_path": f"images/{row['image_name']}",
                "transform_matrix": camera_to_world_matrix(row),
                "free_coverage": float(row["free_coverage"]),
                "surface_coverage": float(row["surface_coverage"]),
            }
        )
    transforms = {
        "camera_model": "SIMPLE_PINHOLE",
        "w": width,
        "h": height,
        "fl_x": focal,
        "fl_y": focal,
        "cx": width / 2.0,
        "cy": height / 2.0,
        "frames": frames,
    }
    transforms_path.write_text(json.dumps(transforms, indent=2), encoding="utf-8")

    return {
        "image_name_mapping": mapping_path,
        "camera_centers_world": centers_path,
        "poses_colmap_w2c": colmap_path,
        "transforms_json": transforms_path,
    }


def append_lidar_scan_points(
    lidar_points: List[Tuple[int, int, float, float, float, float, float, float, float, bool]],
    scan_index: int,
    pose: Pose2D,
    rays: Sequence[Tuple[float, float, bool]],
) -> None:
    for beam_index, (x, y, hit) in enumerate(rays):
        lidar_points.append(
            (
                scan_index,
                beam_index,
                pose.x,
                pose.y,
                pose.yaw,
                x,
                y,
                LIDAR_Z_M,
                math.hypot(x - pose.x, y - pose.y),
                hit,
            )
        )


def write_lidar_point_cloud(
    point_cloud_dir: Path,
    lidar_points: Sequence[Tuple[int, int, float, float, float, float, float, float, float, bool]],
) -> Dict[str, Path]:
    point_cloud_dir.mkdir(parents=True, exist_ok=True)
    csv_path = point_cloud_dir / "lidar_points_world.csv"
    ply_path = point_cloud_dir / "lidar_points_world.ply"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["scan_index", "beam_index", "robot_x", "robot_y", "robot_yaw", "x", "y", "z", "range_m", "hit"])
        for scan_index, beam_index, robot_x, robot_y, robot_yaw, x, y, z, range_m, hit in lidar_points:
            writer.writerow(
                [
                    scan_index,
                    beam_index,
                    f"{robot_x:.6f}",
                    f"{robot_y:.6f}",
                    f"{robot_yaw:.6f}",
                    f"{x:.6f}",
                    f"{y:.6f}",
                    f"{z:.6f}",
                    f"{range_m:.6f}",
                    int(hit),
                ]
            )
    with ply_path.open("w", encoding="utf-8") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {len(lidar_points)}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property uchar red\n")
        handle.write("property uchar green\n")
        handle.write("property uchar blue\n")
        handle.write("end_header\n")
        for _, _, _, _, _, x, y, z, _, hit in lidar_points:
            r, g, b = (230, 60, 45) if hit else (80, 170, 235)
            handle.write(f"{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n")
    return {"lidar_points_csv": csv_path, "lidar_points_ply": ply_path}


def write_occupied_map_point_cloud(point_cloud_dir: Path, known: np.ndarray) -> Path:
    point_cloud_dir.mkdir(parents=True, exist_ok=True)
    occupied = np.argwhere(known == OCCUPIED)
    ply_path = point_cloud_dir / "final_known_occupied_points_world.ply"
    with ply_path.open("w", encoding="utf-8") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {int(occupied.shape[0])}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property uchar red\n")
        handle.write("property uchar green\n")
        handle.write("property uchar blue\n")
        handle.write("end_header\n")
        for iy, ix in occupied:
            x, y = fine_to_world(int(ix), int(iy))
            handle.write(f"{x:.6f} {y:.6f} 0.000000 35 37 40\n")
    return ply_path


def update_coverage_metrics(metrics: RunMetrics, known: np.ndarray, gt_occ: np.ndarray, surface_mask: np.ndarray) -> None:
    gt_free = ~gt_occ
    observed_free = (known == FREE) & gt_free
    observed_surface = (known == OCCUPIED) & surface_mask
    metrics.observed_free_cells = int(np.count_nonzero(observed_free))
    metrics.observed_surface_cells = int(np.count_nonzero(observed_surface))
    metrics.free_coverage = metrics.observed_free_cells / max(1, int(np.count_nonzero(gt_free)))
    metrics.surface_coverage = metrics.observed_surface_cells / max(1, int(np.count_nonzero(surface_mask)))


def goal_distance(pose: Pose2D) -> float:
    return math.hypot(pose.x - GOAL[0], pose.y - GOAL[1])


def select_final_goal_plan(
    planner: str,
    grid: PlanGrid,
    pose: Pose2D,
) -> Tuple[Optional[Tuple[int, int]], Optional[PlannerResult], int]:
    start = nearest_free(grid, grid.world_to_grid(pose.x, pose.y), max_radius=20)
    goal = nearest_free(grid, grid.world_to_grid(GOAL[0], GOAL[1]), max_radius=28)
    if start is None or goal is None:
        return None, None, 1

    plan = PLANNERS[planner](grid, start, goal)
    failed = 0
    if not plan.ok or len(plan.path) < 2:
        failed += 1
        if planner != "weighted_astar":
            fallback = PLANNERS["weighted_astar"](grid, start, goal)
            fallback.fallback_used = True
            fallback.planner_label = f"{planner}/goal_fallback_weighted_astar"
            plan = fallback
    if not plan.ok or len(plan.path) < 2:
        return goal, plan, failed
    return goal, plan, failed


def run_algorithm(
    output_dir: Path,
    map_variant: str,
    planner: str,
    gt_occ: np.ndarray,
    surface_mask: np.ndarray,
) -> RunMetrics:
    run_label = f"{map_variant}_{planner}"
    algorithm_dir = output_dir / run_label
    algorithm_dir.mkdir(parents=True, exist_ok=True)
    map_builder = MAP_BUILDERS[map_variant]
    known = np.full_like(gt_occ, UNKNOWN, dtype=np.int8)
    pose = Pose2D(START[0], START[1], START[2])
    path_world: List[Tuple[float, float]] = [(pose.x, pose.y)]
    metrics = RunMetrics(algorithm=run_label, planner=planner, map_variant=map_variant)
    sample_rows: List[Dict[str, str]] = []
    lidar_points: List[Tuple[int, int, float, float, float, float, float, float, float, bool]] = []
    scan_index = 0

    overhead_path = algorithm_dir / f"{run_label}_overhead_lidar.mp4"
    onboard_path = algorithm_dir / f"{run_label}_onboard.mp4"
    overhead_writer = cv2.VideoWriter(str(overhead_path), cv2.VideoWriter_fourcc(*"mp4v"), VIDEO_FPS, OVERHEAD_SIZE)
    onboard_writer = cv2.VideoWriter(str(onboard_path), cv2.VideoWriter_fourcc(*"mp4v"), VIDEO_FPS, ONBOARD_SIZE)
    if not overhead_writer.isOpened() or not onboard_writer.isOpened():
        raise RuntimeError("Failed to open OpenCV video writers.")

    _, _, rays = update_lidar_map(gt_occ, known, pose)
    append_lidar_scan_points(lidar_points, scan_index, pose, rays)
    update_coverage_metrics(metrics, known, gt_occ, surface_mask)
    metrics.goal_distance_m = goal_distance(pose)
    write_sample(algorithm_dir, run_label, gt_occ, pose, metrics, sample_rows)
    distance_since_sample = 0.0
    current_goal_world: Optional[Tuple[float, float]] = None

    for replan in range(MAX_REPLANS):
        if replan > 0 and replan % 25 == 0:
            print(
                f"  {run_label}: coverage replan={replan} steps={metrics.steps} "
                f"free={metrics.free_coverage:.3f} surface={metrics.surface_coverage:.3f} "
                f"goal_dist={metrics.goal_distance_m:.2f}",
                flush=True,
            )
        if metrics.steps >= MAX_STEPS:
            metrics.reason = "max_steps"
            break
        if metrics.free_coverage >= TARGET_FREE_COVERAGE and metrics.surface_coverage >= TARGET_SURFACE_COVERAGE:
            metrics.coverage_success = True
            metrics.reason = "target_coverage_reached"
            break

        grid = map_builder(known)
        goal_cell, plan, failed_plans = select_frontier_goal(planner, grid, known, pose)
        metrics.failed_plans += failed_plans
        metrics.replans += 1
        if plan is None or goal_cell is None:
            metrics.reason = "no_reachable_frontier"
            break

        metrics.total_expanded_nodes += plan.expanded
        metrics.planning_runtime_ms += plan.runtime_ms
        if plan.timed_out:
            metrics.planner_timeouts += 1
        if plan.fallback_used:
            metrics.planner_fallbacks += 1
        current_goal_world = grid.grid_to_world(*goal_cell)
        path_points = [grid.grid_to_world(x, y) for x, y in plan.path]
        if len(path_points) < 2:
            metrics.reason = "degenerate_plan"
            break
        log_plan(f"{run_label}/coverage", replan, plan, path_length_cells(grid, plan.path), failed_plans)

        for target_x, target_y in path_points[1:]:
            if metrics.steps >= MAX_STEPS:
                break
            fx, fy = world_to_fine(target_x, target_y)
            if in_fine_bounds(fx, fy, gt_occ.shape[1], gt_occ.shape[0]) and gt_occ[fy, fx]:
                mark_known_obstacle_patch(known, fx, fy)
                metrics.failed_plans += 1
                break

            dx = target_x - pose.x
            dy = target_y - pose.y
            step_dist = math.hypot(dx, dy)
            if step_dist > 1.0e-6:
                pose = Pose2D(target_x, target_y, math.atan2(dy, dx))
            else:
                pose = Pose2D(target_x, target_y, pose.yaw)
            metrics.path_length_m += step_dist
            distance_since_sample += step_dist
            metrics.steps += 1
            path_world.append((pose.x, pose.y))

            scan_index += 1
            new_free, new_occ, rays = update_lidar_map(gt_occ, known, pose)
            append_lidar_scan_points(lidar_points, scan_index, pose, rays)
            update_coverage_metrics(metrics, known, gt_occ, surface_mask)
            metrics.goal_distance_m = goal_distance(pose)
            metrics.goal_reached = metrics.goal_distance_m <= GOAL_REACHED_RADIUS_M

            if distance_since_sample >= SAMPLE_DISTANCE_M and (new_free + 2 * new_occ) >= 4:
                write_sample(algorithm_dir, run_label, gt_occ, pose, metrics, sample_rows)
                distance_since_sample = 0.0

            if metrics.steps % VIDEO_STRIDE == 0:
                overhead_writer.write(render_overhead(run_label, known, pose, path_world, rays, metrics, current_goal_world))
                onboard_writer.write(render_onboard(run_label, gt_occ, pose, metrics))

            if metrics.free_coverage >= TARGET_FREE_COVERAGE and metrics.surface_coverage >= TARGET_SURFACE_COVERAGE:
                metrics.coverage_success = True
                metrics.reason = "target_coverage_reached"
                break

        if metrics.coverage_success:
            break

    coverage_reason = metrics.reason or "coverage_loop_completed"
    if metrics.goal_reached:
        metrics.success = True
        metrics.reason = f"{coverage_reason}; goal_already_reached"
    elif metrics.steps >= MAX_STEPS:
        metrics.success = False
        metrics.reason = f"{coverage_reason}; goal_not_attempted_max_steps"
    else:
        for goal_replan in range(MAX_GOAL_REPLANS):
            if metrics.steps >= MAX_STEPS:
                metrics.reason = f"{coverage_reason}; goal_failed_max_steps"
                break
            metrics.goal_distance_m = goal_distance(pose)
            metrics.goal_reached = metrics.goal_distance_m <= GOAL_REACHED_RADIUS_M
            if metrics.goal_reached:
                metrics.success = True
                metrics.reason = f"{coverage_reason}; goal_reached"
                break

            grid = map_builder(known)
            plan_grid = grid
            goal_cell, plan, failed_plans = select_final_goal_plan(planner, grid, pose)
            metrics.failed_plans += failed_plans
            if (plan is None or not plan.ok or len(plan.path) < 2) and map_variant != "pomp_style_ogm":
                fallback_grid = build_pomp_plan_grid(known)
                fallback_goal, fallback_plan, fallback_failed = select_final_goal_plan("weighted_astar", fallback_grid, pose)
                metrics.failed_plans += fallback_failed
                if fallback_plan is not None:
                    fallback_plan.fallback_used = True
                    fallback_plan.planner_label = f"{planner}/goal_fallback_pomp_weighted_astar"
                    plan_grid = fallback_grid
                    goal_cell = fallback_goal
                    plan = fallback_plan
            metrics.replans += 1
            metrics.final_goal_attempts += 1
            if plan is None or goal_cell is None:
                metrics.reason = f"{coverage_reason}; goal_no_plan"
                break
            metrics.total_expanded_nodes += plan.expanded
            metrics.planning_runtime_ms += plan.runtime_ms
            if plan.timed_out:
                metrics.planner_timeouts += 1
            if plan.fallback_used:
                metrics.planner_fallbacks += 1
            if not plan.ok or len(plan.path) < 2:
                metrics.reason = f"{coverage_reason}; goal_plan_failed:{plan.reason}"
                break

            current_goal_world = GOAL
            path_points = [plan_grid.grid_to_world(x, y) for x, y in plan.path]
            log_plan(f"{run_label}/final_goal", goal_replan, plan, path_length_cells(plan_grid, plan.path), failed_plans)

            for target_x, target_y in path_points[1:]:
                if metrics.steps >= MAX_STEPS:
                    break
                fx, fy = world_to_fine(target_x, target_y)
                if in_fine_bounds(fx, fy, gt_occ.shape[1], gt_occ.shape[0]) and gt_occ[fy, fx]:
                    mark_known_obstacle_patch(known, fx, fy)
                    metrics.failed_plans += 1
                    break

                dx = target_x - pose.x
                dy = target_y - pose.y
                step_dist = math.hypot(dx, dy)
                if step_dist > 1.0e-6:
                    pose = Pose2D(target_x, target_y, math.atan2(dy, dx))
                else:
                    pose = Pose2D(target_x, target_y, pose.yaw)
                metrics.path_length_m += step_dist
                distance_since_sample += step_dist
                metrics.steps += 1
                path_world.append((pose.x, pose.y))

                scan_index += 1
                new_free, new_occ, rays = update_lidar_map(gt_occ, known, pose)
                append_lidar_scan_points(lidar_points, scan_index, pose, rays)
                update_coverage_metrics(metrics, known, gt_occ, surface_mask)
                metrics.goal_distance_m = goal_distance(pose)
                metrics.goal_reached = metrics.goal_distance_m <= GOAL_REACHED_RADIUS_M

                if distance_since_sample >= SAMPLE_DISTANCE_M:
                    write_sample(algorithm_dir, run_label, gt_occ, pose, metrics, sample_rows)
                    distance_since_sample = 0.0

                if metrics.steps % VIDEO_STRIDE == 0:
                    overhead_writer.write(render_overhead(run_label, known, pose, path_world, rays, metrics, current_goal_world))
                    onboard_writer.write(render_onboard(run_label, gt_occ, pose, metrics))

                if metrics.goal_reached:
                    metrics.success = True
                    metrics.reason = f"{coverage_reason}; goal_reached"
                    break

            if metrics.goal_reached:
                break

        if not metrics.goal_reached and "goal_" not in metrics.reason:
            metrics.reason = f"{coverage_reason}; goal_not_reached"

    if not metrics.reason:
        metrics.reason = "completed" if metrics.goal_reached else "stopped"
    metrics.success = bool(metrics.goal_reached)
    metrics.coverage_success = (
        metrics.free_coverage >= TARGET_FREE_COVERAGE and metrics.surface_coverage >= TARGET_SURFACE_COVERAGE
    )
    if metrics.replans:
        metrics.mean_plan_runtime_ms = metrics.planning_runtime_ms / metrics.replans

    overhead_writer.write(render_overhead(run_label, known, pose, path_world, rays, metrics, current_goal_world))
    onboard_writer.write(render_onboard(run_label, gt_occ, pose, metrics))
    overhead_writer.release()
    onboard_writer.release()

    route_png = algorithm_dir / f"{run_label}_trajectory.png"
    render_final_route(route_png, run_label, known, path_world, metrics)
    poses_csv = algorithm_dir / "samples" / "poses.csv"
    poses_csv.parent.mkdir(parents=True, exist_ok=True)
    with poses_csv.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "image_name",
            "sample_index",
            "x",
            "y",
            "z",
            "yaw",
            "qw",
            "qx",
            "qy",
            "qz",
            "free_coverage",
            "surface_coverage",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(sample_rows)

    sidecars = write_pose_sidecars(poses_csv.parent, sample_rows)
    point_cloud_dir = algorithm_dir / "point_cloud"
    lidar_files = write_lidar_point_cloud(point_cloud_dir, lidar_points)
    occupied_ply = write_occupied_map_point_cloud(point_cloud_dir, known)

    metrics.route_png = str(route_png.relative_to(output_dir))
    metrics.overhead_lidar_video = str(overhead_path.relative_to(output_dir))
    metrics.onboard_video = str(onboard_path.relative_to(output_dir))
    metrics.samples_dir = str((algorithm_dir / "samples").relative_to(output_dir))
    metrics.poses_csv = str(poses_csv.relative_to(output_dir))
    metrics.transforms_json = str(sidecars["transforms_json"].relative_to(output_dir))
    metrics.poses_colmap_w2c = str(sidecars["poses_colmap_w2c"].relative_to(output_dir))
    metrics.camera_centers_world = str(sidecars["camera_centers_world"].relative_to(output_dir))
    metrics.image_name_mapping = str(sidecars["image_name_mapping"].relative_to(output_dir))
    metrics.lidar_points_csv = str(lidar_files["lidar_points_csv"].relative_to(output_dir))
    metrics.lidar_points_ply = str(lidar_files["lidar_points_ply"].relative_to(output_dir))
    metrics.occupied_points_ply = str(occupied_ply.relative_to(output_dir))
    metrics.lidar_scans = scan_index + 1
    metrics.lidar_points = len(lidar_points)
    write_algorithm_metadata(algorithm_dir, run_label, metrics, path_world)
    return metrics


def render_final_route(path: Path, algorithm: str, known: np.ndarray, route: Sequence[Tuple[float, float]], metrics: RunMetrics) -> None:
    frame = render_overhead(algorithm, known, Pose2D(route[-1][0], route[-1][1], 0.0), route, [], metrics, None)
    cv2.imwrite(str(path), frame)


def write_algorithm_metadata(
    algorithm_dir: Path,
    algorithm: str,
    metrics: RunMetrics,
    route: Sequence[Tuple[float, float]],
) -> None:
    data = {
        "algorithm": algorithm,
        "map_variant": metrics.map_variant,
        "planner": metrics.planner,
        "map_update": "simulated 2D lidar ray casting from an initially unknown map",
        "pose_source": "simulated LIO pose: ground-truth pose is used as drift-free LiDAR-inertial odometry",
        "map_representation": MAP_DESCRIPTIONS.get(metrics.map_variant, metrics.map_variant),
        "coverage_goal": {
            "target_free_coverage": TARGET_FREE_COVERAGE,
            "target_surface_coverage": TARGET_SURFACE_COVERAGE,
            "success": metrics.coverage_success,
        },
        "final_goal": {
            "x": GOAL[0],
            "y": GOAL[1],
            "reached_radius_m": GOAL_REACHED_RADIUS_M,
            "reached": metrics.goal_reached,
            "distance_m": metrics.goal_distance_m,
        },
        "point_cloud_outputs": {
            "lidar_points_csv": metrics.lidar_points_csv,
            "lidar_points_ply": metrics.lidar_points_ply,
            "occupied_points_ply": metrics.occupied_points_ply,
        },
        "metrics": metrics.__dict__,
        "route": [{"x": x, "y": y} for x, y in route],
    }
    (algorithm_dir / "metadata.json").write_text(json.dumps(data, indent=2), encoding="utf-8")


def render_comparison_route(output_dir: Path, gt_occ: np.ndarray, routes: Dict[str, Sequence[Tuple[float, float]]]) -> None:
    rows, cols = gt_occ.shape
    colors = np.zeros((rows, cols, 3), dtype=np.uint8)
    colors[~gt_occ] = (238, 238, 232)
    colors[gt_occ] = (35, 37, 40)
    img = cv2.resize(np.flipud(colors), OVERHEAD_SIZE, interpolation=cv2.INTER_NEAREST)
    palette = {
        "dijkstra": (120, 85, 210),
        "astar": (225, 125, 35),
        "weighted_astar": (40, 150, 230),
        "theta_star": (45, 175, 95),
    }
    y = 24
    for algorithm, route in routes.items():
        planner = algorithm
        for name in sorted(PLANNERS, key=len, reverse=True):
            if algorithm.endswith(name):
                planner = name
                break
        color = palette.get(planner, (0, 0, 255))
        if len(route) > 1:
            pts = np.array([world_to_overhead_px(x, yy) for x, yy in route], dtype=np.int32)
            cv2.polylines(img, [pts], False, color, 2, cv2.LINE_AA)
        cv2.putText(img, algorithm, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
        y += 24
    cv2.circle(img, world_to_overhead_px(START[0], START[1]), 7, (50, 170, 80), -1)
    cv2.circle(img, world_to_overhead_px(GOAL[0], GOAL[1]), 7, (50, 70, 220), -1)
    cv2.imwrite(str(output_dir / "route_comparison.png"), img)


def write_metrics(outputs: Path, metrics: Sequence[RunMetrics]) -> None:
    fieldnames = list(RunMetrics.__dataclass_fields__.keys())
    with (outputs / "metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in metrics:
            data = row.__dict__.copy()
            for key in [
                "goal_distance_m",
                "path_length_m",
                "free_coverage",
                "surface_coverage",
                "planning_runtime_ms",
                "mean_plan_runtime_ms",
            ]:
                data[key] = f"{float(data[key]):.6f}"
            writer.writerow(data)
    (outputs / "metrics.json").write_text(json.dumps([m.__dict__ for m in metrics], indent=2), encoding="utf-8")
    render_metrics_table(outputs / "metrics_table.png", metrics)


def render_metrics_table(path: Path, metrics: Sequence[RunMetrics]) -> None:
    headers = ["run", "coverage", "goal", "goal m", "free", "surface", "path m", "samples", "lidar pts"]
    rows = []
    for m in metrics:
        rows.append(
            [
                m.algorithm,
                "yes" if m.coverage_success else "no",
                "yes" if m.goal_reached else "no",
                f"{m.goal_distance_m:.2f}",
                f"{m.free_coverage:.3f}",
                f"{m.surface_coverage:.3f}",
                f"{m.path_length_m:.1f}",
                str(m.reconstruction_samples),
                str(m.lidar_points),
            ]
        )
    cell_w = [230, 90, 70, 80, 80, 90, 90, 90, 100]
    row_h = 34
    width = sum(cell_w) + 2
    height = row_h * (len(rows) + 1) + 2
    img = Image.new("RGB", (width, height), (250, 250, 248))
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 15)
        bold = ImageFont.truetype("DejaVuSans-Bold.ttf", 15)
    except OSError:
        font = ImageFont.load_default()
        bold = font
    x = 1
    for i, header in enumerate(headers):
        draw.rectangle((x, 1, x + cell_w[i], row_h), fill=(38, 45, 52))
        draw.text((x + 8, 9), header, fill=(255, 255, 255), font=bold)
        x += cell_w[i]
    for r, row in enumerate(rows):
        y = row_h * (r + 1) + 1
        x = 1
        fill = (242, 244, 243) if r % 2 == 0 else (255, 255, 255)
        for i, value in enumerate(row):
            draw.rectangle((x, y, x + cell_w[i], y + row_h), fill=fill, outline=(210, 213, 214))
            draw.text((x + 8, y + 9), value, fill=(25, 27, 30), font=font)
            x += cell_w[i]
    img.save(path)


def write_readme(output_dir: Path, metrics: Sequence[RunMetrics]) -> None:
    lines = [
        "# Online LIO Coverage + Goal-Reach Simulation",
        "",
        "This experiment starts from an unknown local map. A simulated 2D LiDAR scan updates free/occupied cells, while the pose is treated as drift-free LIO output. Each run first performs frontier coverage for reconstruction sampling, then explicitly replans to the final B goal and records whether the robot reaches it.",
        "",
        "Generated artifacts:",
        "- `metrics.csv`, `metrics.json`, `metrics_table.png`",
        "- `route_comparison.png`",
        "- Per run: `*_trajectory.png`, `*_overhead_lidar.mp4`, `*_onboard.mp4`",
        "- Per run camera export: `samples/images/*.png`, `samples/poses.csv`, `samples/transforms.json`, `samples/poses_colmap_w2c.txt`, `samples/camera_centers_world.txt`, `samples/image_name_mapping.csv`",
        "- Per run LiDAR point cloud: `point_cloud/lidar_points_world.csv`, `point_cloud/lidar_points_world.ply`, `point_cloud/final_known_occupied_points_world.ply`",
        "",
        "Runs:",
    ]
    for metric in metrics:
        lines.append(
            f"- `{metric.algorithm}`: coverage_success={metric.coverage_success}, "
            f"goal_reached={metric.goal_reached}, goal_distance={metric.goal_distance_m:.2f} m, "
            f"free={metric.free_coverage:.3f}, surface={metric.surface_coverage:.3f}, "
            f"path={metric.path_length_m:.1f} m, samples={metric.reconstruction_samples}, "
            f"lidar_points={metric.lidar_points}"
        )
    lines.extend(
        [
            "",
            "Notes:",
            "- This is an online coverage simulation, not a full FAST-LIVO2 graph-optimization integration.",
            "- Ground-truth geometry is used only by the simulator to synthesize LiDAR/camera observations; the planner starts with an unknown map.",
            "- `direct_ogm` uses a conservative direct coarse occupancy-grid projection.",
            "- `pomp_style_ogm` uses the POMP-style sub-cell projection.",
            "- `theta_star` is implemented as bounded weighted A* followed by cached Theta-style line-of-sight shortcut smoothing; the online loop does not run full Theta* global search.",
        ]
    )
    (output_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulate online LiDAR/LIO mapping with POMP-style frontier coverage planning."
    )
    parser.add_argument(
        "output_dir",
        nargs="?",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for metrics, videos, route plots, and reconstruction samples.",
    )
    parser.add_argument(
        "--algorithms",
        nargs="+",
        choices=tuple(PLANNERS.keys()),
        default=list(PLANNERS.keys()),
        help="Planner algorithms to run.",
    )
    parser.add_argument(
        "--map-variants",
        nargs="+",
        choices=tuple(MAP_BUILDERS.keys()),
        default=list(MAP_BUILDERS.keys()),
        help="Map representations to compare: direct coarse OGM and/or POMP-style sub-cell OGM.",
    )
    parser.add_argument(
        "--skip-theta-star",
        action="store_true",
        help="Skip the theta_star comparison entirely. Useful for quick or low-load runs.",
    )
    parser.add_argument(
        "--theta-timeout",
        type=float,
        default=DEFAULT_THETA_TIMEOUT_SEC,
        help="Seconds allowed for theta-style shortcut smoothing before falling back to raw weighted A*.",
    )
    parser.add_argument(
        "--planner-timeout",
        type=float,
        default=DEFAULT_PLANNER_TIMEOUT_SEC,
        help="Seconds allowed for each graph-search call before it is treated as timed out.",
    )
    parser.add_argument(
        "--max-expanded-nodes",
        type=int,
        default=DEFAULT_MAX_EXPANDED_NODES,
        help="Maximum expanded nodes for each graph-search call.",
    )
    parser.add_argument(
        "--quiet-planner-log",
        action="store_true",
        help="Only print coarse progress instead of per-replan planner timing lines.",
    )
    return parser.parse_args(argv[1:])


def main(argv: Sequence[str]) -> int:
    global RUNTIME_MAX_EXPANDED_NODES
    global RUNTIME_PLANNER_TIMEOUT_SEC
    global RUNTIME_THETA_TIMEOUT_SEC
    global RUNTIME_LOG_PLANS

    args = parse_args(argv)
    RUNTIME_MAX_EXPANDED_NODES = max(1, int(args.max_expanded_nodes))
    RUNTIME_PLANNER_TIMEOUT_SEC = max(0.001, float(args.planner_timeout))
    RUNTIME_THETA_TIMEOUT_SEC = max(0.001, float(args.theta_timeout))
    RUNTIME_LOG_PLANS = not bool(args.quiet_planner_log)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    gt_occ = build_ground_truth()
    surface_mask = build_surface_mask(gt_occ)
    metrics: List[RunMetrics] = []
    routes: Dict[str, Sequence[Tuple[float, float]]] = {}
    algorithms = [name for name in args.algorithms if not (args.skip_theta_star and name == "theta_star")]
    map_variants = list(args.map_variants)
    print(
        f"planner safety: timeout={RUNTIME_PLANNER_TIMEOUT_SEC:.2f}s "
        f"theta_timeout={RUNTIME_THETA_TIMEOUT_SEC:.2f}s "
        f"max_expanded={RUNTIME_MAX_EXPANDED_NODES} "
        f"map_variants={','.join(map_variants)} "
        f"algorithms={','.join(algorithms)}",
        flush=True,
    )

    for map_variant in map_variants:
        for algorithm in algorithms:
            run_label = f"{map_variant}_{algorithm}"
            print(f"running {run_label}...", flush=True)
            metric = run_algorithm(output_dir, map_variant, algorithm, gt_occ, surface_mask)
            metrics.append(metric)
            metadata = json.loads((output_dir / run_label / "metadata.json").read_text(encoding="utf-8"))
            routes[run_label] = [(pt["x"], pt["y"]) for pt in metadata["route"]]
            print(
                f"{run_label}: coverage_success={metric.coverage_success} "
                f"goal_reached={metric.goal_reached} goal_dist={metric.goal_distance_m:.2f} "
                f"free={metric.free_coverage:.3f} surface={metric.surface_coverage:.3f} "
                f"path={metric.path_length_m:.1f} samples={metric.reconstruction_samples} "
                f"lidar_points={metric.lidar_points}",
                flush=True,
            )

    write_metrics(output_dir, metrics)
    render_comparison_route(output_dir, gt_occ, routes)
    write_readme(output_dir, metrics)
    print(output_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
