#!/usr/bin/env python3
from dataclasses import dataclass
import csv
import heapq
import json
import math
from pathlib import Path
import sys
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont

from complex_construction_scene import (
    MAP_ORIGIN,
    MAP_SIZE,
    START,
    GOAL,
    BoxObstacle,
    all_obstacles,
    distance_to_obstacles,
    point_in_any_box,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exports" / "planning_benchmarks" / "complex_construction_benchmark"
BENCHMARK_RESOLUTION = 0.20
POMP_SUBDIVISIONS = 4
POMP_OCCUPANCY_THRESHOLD = 0.55
EPS = 1.0e-9


@dataclass
class Grid:
    name: str
    cols: int
    rows: int
    resolution: float
    origin: Tuple[float, float]
    cells: List[int]
    edge_obstacles: Sequence[BoxObstacle]

    def flatten(self, x: int, y: int) -> int:
        return x + y * self.cols

    def unflatten(self, idx: int) -> Tuple[int, int]:
        return idx % self.cols, idx // self.cols

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.cols and 0 <= y < self.rows

    def is_free(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and self.cells[self.flatten(x, y)] == 0

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(math.floor((x - self.origin[0]) / self.resolution))
        gy = int(math.floor((y - self.origin[1]) / self.resolution))
        return gx, gy

    def grid_to_world(self, x: int, y: int) -> Tuple[float, float]:
        return (
            self.origin[0] + (x + 0.5) * self.resolution,
            self.origin[1] + (y + 0.5) * self.resolution,
        )

    def count_cells(self) -> Dict[str, int]:
        free = self.cells.count(0)
        occupied = self.cells.count(1)
        boundary = self.cells.count(2)
        return {"free": free, "occupied": occupied, "boundary": boundary}

    def edge_is_clear(self, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
        ax, ay = self.grid_to_world(*a)
        bx, by = self.grid_to_world(*b)
        dist = math.hypot(bx - ax, by - ay)
        steps = max(2, int(math.ceil(dist / (0.25 * self.resolution))))
        for i in range(steps + 1):
            t = i / steps
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            if point_in_any_box(x, y, self.edge_obstacles):
                return False
        return True


def cell_is_boundary(x: int, y: int, cols: int, rows: int) -> bool:
    return x == 0 or y == 0 or x == cols - 1 or y == rows - 1


def cell_samples(
    x: int,
    y: int,
    resolution: float,
    origin: Tuple[float, float],
    subdivisions: int,
) -> Iterable[Tuple[float, float]]:
    base_x = origin[0] + x * resolution
    base_y = origin[1] + y * resolution
    step = resolution / subdivisions
    for sy in range(subdivisions):
        for sx in range(subdivisions):
            yield base_x + (sx + 0.5) * step, base_y + (sy + 0.5) * step


def build_direct_grid(obstacles: Sequence[BoxObstacle]) -> Grid:
    cols = int(round(MAP_SIZE[0] / BENCHMARK_RESOLUTION))
    rows = int(round(MAP_SIZE[1] / BENCHMARK_RESOLUTION))
    cells: List[int] = []
    for y in range(rows):
        for x in range(cols):
            if cell_is_boundary(x, y, cols, rows):
                cells.append(2)
                continue
            wx, wy = (
                MAP_ORIGIN[0] + (x + 0.5) * BENCHMARK_RESOLUTION,
                MAP_ORIGIN[1] + (y + 0.5) * BENCHMARK_RESOLUTION,
            )
            corners = [
                (wx, wy),
                (wx - 0.5 * BENCHMARK_RESOLUTION, wy - 0.5 * BENCHMARK_RESOLUTION),
                (wx + 0.5 * BENCHMARK_RESOLUTION, wy - 0.5 * BENCHMARK_RESOLUTION),
                (wx - 0.5 * BENCHMARK_RESOLUTION, wy + 0.5 * BENCHMARK_RESOLUTION),
                (wx + 0.5 * BENCHMARK_RESOLUTION, wy + 0.5 * BENCHMARK_RESOLUTION),
            ]
            cells.append(1 if any(point_in_any_box(px, py, obstacles) for px, py in corners) else 0)
    return Grid("direct_ogm", cols, rows, BENCHMARK_RESOLUTION, MAP_ORIGIN, cells, obstacles)


def build_pomp_style_grid(obstacles: Sequence[BoxObstacle]) -> Grid:
    cols = int(round(MAP_SIZE[0] / BENCHMARK_RESOLUTION))
    rows = int(round(MAP_SIZE[1] / BENCHMARK_RESOLUTION))
    cells: List[int] = []
    for y in range(rows):
        for x in range(cols):
            if cell_is_boundary(x, y, cols, rows):
                cells.append(2)
                continue
            occupied_samples = []
            for sx, sy in cell_samples(x, y, BENCHMARK_RESOLUTION, MAP_ORIGIN, POMP_SUBDIVISIONS):
                if point_in_any_box(sx, sy, obstacles):
                    occupied_samples.append((sx, sy))
            if not occupied_samples:
                cells.append(0)
                continue

            ratio = len(occupied_samples) / float(POMP_SUBDIVISIONS * POMP_SUBDIVISIONS)
            if ratio >= POMP_OCCUPANCY_THRESHOLD:
                cells.append(1)
                continue

            center_x, center_y = (
                MAP_ORIGIN[0] + (x + 0.5) * BENCHMARK_RESOLUTION,
                MAP_ORIGIN[1] + (y + 0.5) * BENCHMARK_RESOLUTION,
            )
            unsafe_lim = 0.5 * BENCHMARK_RESOLUTION * 0.78
            unsafe = any(
                abs(px - center_x) >= unsafe_lim or abs(py - center_y) >= unsafe_lim
                for px, py in occupied_samples
            )
            spans_x = (
                min(px for px, _ in occupied_samples) < center_x
                and max(px for px, _ in occupied_samples) > center_x
            )
            spans_y = (
                min(py for _, py in occupied_samples) < center_y
                and max(py for _, py in occupied_samples) > center_y
            )
            cells.append(1 if unsafe and (spans_x or spans_y) else 0)
    return Grid("pomp_style_ogm", cols, rows, BENCHMARK_RESOLUTION, MAP_ORIGIN, cells, obstacles)


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


def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def can_step(grid: Grid, x: int, y: int, dx: int, dy: int) -> bool:
    nx, ny = x + dx, y + dy
    if not grid.is_free(nx, ny):
        return False
    if dx != 0 and dy != 0:
        if not (grid.is_free(x + dx, y) and grid.is_free(x, y + dy)):
            return False
    return True


def reconstruct(came_from: Dict[int, int], current: int, grid: Grid) -> List[Tuple[int, int]]:
    path = [current]
    while current in came_from:
        parent = came_from[current]
        if parent == current:
            break
        current = parent
        path.append(current)
    path.reverse()
    return [grid.unflatten(idx) for idx in path]


def astar_like(
    grid: Grid,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    heuristic_weight: float,
) -> Dict:
    if not grid.is_free(*start) or not grid.is_free(*goal):
        return {"ok": False, "reason": "start_or_goal_not_free", "expanded": 0, "path": []}

    start_id = grid.flatten(*start)
    goal_id = grid.flatten(*goal)
    open_heap = [(0.0, 0, start_id)]
    came_from: Dict[int, int] = {}
    g = {start_id: 0.0}
    closed = set()
    expanded_order = []
    counter = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expanded_order.append(current)
        if current == goal_id:
            return {
                "ok": True,
                "path": reconstruct(came_from, current, grid),
                "expanded": len(expanded_order),
                "expanded_order": expanded_order,
            }

        x, y = grid.unflatten(current)
        for dx, dy, step_cost in DIRS_8:
            if not can_step(grid, x, y, dx, dy):
                continue
            nx, ny = x + dx, y + dy
            nid = grid.flatten(nx, ny)
            tentative = g[current] + step_cost
            if tentative + EPS < g.get(nid, math.inf):
                came_from[nid] = current
                g[nid] = tentative
                h = heuristic_weight * heuristic((nx, ny), goal)
                counter += 1
                heapq.heappush(open_heap, (tentative + h, counter, nid))
    return {"ok": False, "reason": "no_path", "expanded": len(expanded_order), "path": []}


def line_of_sight(grid: Grid, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        if not grid.is_free(x, y):
            return False
        if x == x1 and y == y1:
            return True
        e2 = 2 * err
        old_x, old_y = x, y
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
        if x != old_x and y != old_y:
            if not grid.is_free(x, old_y) or not grid.is_free(old_x, y):
                return False


def theta_star(grid: Grid, start: Tuple[int, int], goal: Tuple[int, int]) -> Dict:
    if not grid.is_free(*start) or not grid.is_free(*goal):
        return {"ok": False, "reason": "start_or_goal_not_free", "expanded": 0, "path": []}
    start_id = grid.flatten(*start)
    goal_id = grid.flatten(*goal)
    parent = {start_id: start_id}
    g = {start_id: 0.0}
    open_heap = [(heuristic(start, goal), 0, start_id)]
    closed = set()
    expanded_order = []
    counter = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expanded_order.append(current)
        if current == goal_id:
            return {
                "ok": True,
                "path": reconstruct(parent, current, grid),
                "expanded": len(expanded_order),
                "expanded_order": expanded_order,
            }
        x, y = grid.unflatten(current)
        p_id = parent[current]
        p_xy = grid.unflatten(p_id)
        for dx, dy, step_cost in DIRS_8:
            if not can_step(grid, x, y, dx, dy):
                continue
            nx, ny = x + dx, y + dy
            nid = grid.flatten(nx, ny)
            if line_of_sight(grid, p_xy, (nx, ny)):
                candidate_parent = p_id
                candidate_g = g[p_id] + heuristic(p_xy, (nx, ny))
            else:
                candidate_parent = current
                candidate_g = g[current] + step_cost
            if candidate_g + EPS < g.get(nid, math.inf):
                parent[nid] = candidate_parent
                g[nid] = candidate_g
                counter += 1
                heapq.heappush(open_heap, (candidate_g + heuristic((nx, ny), goal), counter, nid))
    return {"ok": False, "reason": "no_path", "expanded": len(expanded_order), "path": []}


def norm_dir(value: int) -> int:
    return 0 if value == 0 else (1 if value > 0 else -1)


def jps_successors(grid: Grid, node: Tuple[int, int], parent: Optional[Tuple[int, int]]) -> List[Tuple[int, int]]:
    if parent is None:
        return [(dx, dy) for dx, dy, _ in DIRS_8 if can_step(grid, node[0], node[1], dx, dy)]
    x, y = node
    px, py = parent
    dx = norm_dir(x - px)
    dy = norm_dir(y - py)
    dirs = []
    if dx != 0 and dy != 0:
        for cand in [(dx, dy), (dx, 0), (0, dy)]:
            if can_step(grid, x, y, cand[0], cand[1]):
                dirs.append(cand)
        if not grid.is_free(x - dx, y) and can_step(grid, x, y, -dx, dy):
            dirs.append((-dx, dy))
        if not grid.is_free(x, y - dy) and can_step(grid, x, y, dx, -dy):
            dirs.append((dx, -dy))
    elif dx != 0:
        if can_step(grid, x, y, dx, 0):
            dirs.append((dx, 0))
        if not grid.is_free(x, y + 1) and can_step(grid, x, y, dx, 1):
            dirs.append((dx, 1))
        if not grid.is_free(x, y - 1) and can_step(grid, x, y, dx, -1):
            dirs.append((dx, -1))
    else:
        if can_step(grid, x, y, 0, dy):
            dirs.append((0, dy))
        if not grid.is_free(x + 1, y) and can_step(grid, x, y, 1, dy):
            dirs.append((1, dy))
        if not grid.is_free(x - 1, y) and can_step(grid, x, y, -1, dy):
            dirs.append((-1, dy))
    return list(dict.fromkeys(dirs))


def jump(grid: Grid, x: int, y: int, dx: int, dy: int, goal: Tuple[int, int]) -> Optional[Tuple[int, int]]:
    nx, ny = x + dx, y + dy
    if not can_step(grid, x, y, dx, dy):
        return None
    if (nx, ny) == goal:
        return nx, ny

    if dx != 0 and dy != 0:
        if (not grid.is_free(nx - dx, ny) and can_step(grid, nx, ny, -dx, dy)) or (
            not grid.is_free(nx, ny - dy) and can_step(grid, nx, ny, dx, -dy)
        ):
            return nx, ny
        if jump(grid, nx, ny, dx, 0, goal) is not None or jump(grid, nx, ny, 0, dy, goal) is not None:
            return nx, ny
    elif dx != 0:
        if (not grid.is_free(nx, ny + 1) and can_step(grid, nx, ny, dx, 1)) or (
            not grid.is_free(nx, ny - 1) and can_step(grid, nx, ny, dx, -1)
        ):
            return nx, ny
    else:
        if (not grid.is_free(nx + 1, ny) and can_step(grid, nx, ny, 1, dy)) or (
            not grid.is_free(nx - 1, ny) and can_step(grid, nx, ny, -1, dy)
        ):
            return nx, ny
    return jump(grid, nx, ny, dx, dy, goal)


def jps(grid: Grid, start: Tuple[int, int], goal: Tuple[int, int]) -> Dict:
    if not grid.is_free(*start) or not grid.is_free(*goal):
        return {"ok": False, "reason": "start_or_goal_not_free", "expanded": 0, "path": []}
    start_id = grid.flatten(*start)
    goal_id = grid.flatten(*goal)
    open_heap = [(heuristic(start, goal), 0, start_id)]
    g = {start_id: 0.0}
    came_from: Dict[int, int] = {}
    closed = set()
    expanded_order = []
    counter = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expanded_order.append(current)
        if current == goal_id:
            return {
                "ok": True,
                "path": reconstruct(came_from, current, grid),
                "expanded": len(expanded_order),
                "expanded_order": expanded_order,
            }
        node = grid.unflatten(current)
        parent = grid.unflatten(came_from[current]) if current in came_from else None
        for dx, dy in jps_successors(grid, node, parent):
            jp = jump(grid, node[0], node[1], dx, dy, goal)
            if jp is None:
                continue
            jid = grid.flatten(*jp)
            if jid in closed:
                continue
            tentative = g[current] + heuristic(node, jp)
            if tentative + EPS < g.get(jid, math.inf):
                g[jid] = tentative
                came_from[jid] = current
                counter += 1
                heapq.heappush(open_heap, (tentative + heuristic(jp, goal), counter, jid))
    return {"ok": False, "reason": "no_path", "expanded": len(expanded_order), "path": []}


PLANNERS = {
    "dijkstra": lambda grid, start, goal: astar_like(grid, start, goal, heuristic_weight=0.0),
    "astar": lambda grid, start, goal: astar_like(grid, start, goal, heuristic_weight=1.0),
    "weighted_astar": lambda grid, start, goal: astar_like(grid, start, goal, heuristic_weight=1.4),
    "theta_star": theta_star,
}


def path_to_world(grid: Grid, path: Sequence[Tuple[int, int]]) -> List[Tuple[float, float]]:
    return [grid.grid_to_world(x, y) for x, y in path]


def path_length(points: Sequence[Tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def turn_count(points: Sequence[Tuple[float, float]]) -> int:
    turns = 0
    last_angle = None
    for a, b in zip(points, points[1:]):
        angle = math.atan2(b[1] - a[1], b[0] - a[0])
        if last_angle is not None and abs(math.atan2(math.sin(angle - last_angle), math.cos(angle - last_angle))) > 0.2:
            turns += 1
        last_angle = angle
    return turns


def min_clearance(points: Sequence[Tuple[float, float]], obstacles: Sequence[BoxObstacle]) -> Optional[float]:
    if not points:
        return None
    samples: List[Tuple[float, float]] = []
    for a, b in zip(points, points[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(dist / 0.10)))
        for i in range(steps):
            t = i / steps
            samples.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    samples.append(points[-1])
    return min(distance_to_obstacles(x, y, obstacles) for x, y in samples)


def min_grid_clearance(points: Sequence[Tuple[float, float]], grid: Grid) -> Optional[float]:
    if not points:
        return None
    occupied = [
        grid.grid_to_world(x, y)
        for y in range(grid.rows)
        for x in range(grid.cols)
        if grid.cells[grid.flatten(x, y)] != 0
    ]
    if not occupied:
        return None
    samples: List[Tuple[float, float]] = []
    for a, b in zip(points, points[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(dist / 0.10)))
        for i in range(steps):
            t = i / steps
            samples.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    samples.append(points[-1])
    clearance = math.inf
    for sx, sy in samples:
        nearest = min(math.hypot(sx - ox, sy - oy) for ox, oy in occupied)
        clearance = min(clearance, max(0.0, nearest - 0.5 * grid.resolution))
    return clearance


def nearest_free(grid: Grid, idx: Tuple[int, int], max_radius: int = 20) -> Tuple[int, int]:
    if grid.is_free(*idx):
        return idx
    x0, y0 = idx
    for radius in range(1, max_radius + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                cand = (x0 + dx, y0 + dy)
                if grid.is_free(*cand):
                    return cand
    raise RuntimeError(f"No free cell found near {idx} on {grid.name}.")


def run_benchmark(output_dir: Path) -> Tuple[List[Dict], Dict[str, Dict]]:
    obstacles = all_obstacles(include_boundary=True)
    grids = [build_direct_grid(obstacles), build_pomp_style_grid(obstacles)]
    output_dir.mkdir(parents=True, exist_ok=True)
    results: List[Dict] = []
    paths: Dict[str, Dict] = {}
    for grid in grids:
        start = nearest_free(grid, grid.world_to_grid(START[0], START[1]))
        goal = nearest_free(grid, grid.world_to_grid(GOAL[0], GOAL[1]))
        counts = grid.count_cells()
        for planner_name, planner in PLANNERS.items():
            started = time.perf_counter()
            result = planner(grid, start, goal)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            world_path = path_to_world(grid, result.get("path", [])) if result.get("ok") else []
            key = f"{grid.name}:{planner_name}"
            paths[key] = {"grid": grid, "planner": planner_name, "result": result, "world_path": world_path}
            results.append(
                {
                    "map_variant": grid.name,
                    "planner": planner_name,
                    "success": bool(result.get("ok")),
                    "path_nodes": len(world_path),
                    "path_length_m": f"{path_length(world_path):.4f}" if world_path else "",
                    "turn_count": turn_count(world_path) if world_path else "",
                    "min_grid_clearance_m": f"{min_grid_clearance(world_path, grid):.4f}" if world_path else "",
                    "expanded_nodes": int(result.get("expanded", 0)),
                    "runtime_ms": f"{elapsed_ms:.4f}",
                    "free_cells": counts["free"],
                    "occupied_cells": counts["occupied"],
                    "boundary_cells": counts["boundary"],
                    "reason": result.get("reason", ""),
                }
            )
    return results, paths


def world_to_pixel(grid: Grid, point: Tuple[float, float], scale: int, margin: int) -> Tuple[int, int]:
    gx, gy = grid.world_to_grid(point[0], point[1])
    px = margin + gx * scale + scale // 2
    py = margin + (grid.rows - gy - 1) * scale + scale // 2
    return px, py


def render_grid(
    grid: Grid,
    paths: Sequence[Tuple[str, Sequence[Tuple[float, float]], Tuple[int, int, int]]],
    title: str,
    output_path: Optional[Path],
    size: Tuple[int, int] = (900, 760),
) -> Image.Image:
    margin = 38
    scale = max(3, min((size[0] - 2 * margin) // grid.cols, (size[1] - 2 * margin) // grid.rows))
    width = margin * 2 + grid.cols * scale
    height = margin * 2 + grid.rows * scale
    image = Image.new("RGB", (width, height), (245, 246, 244))
    draw = ImageDraw.Draw(image)
    for y in range(grid.rows):
        for x in range(grid.cols):
            cell = grid.cells[grid.flatten(x, y)]
            if cell == 0:
                color = (245, 246, 244)
            elif cell == 1:
                color = (58, 62, 66)
            else:
                color = (20, 22, 24)
            px = margin + x * scale
            py = margin + (grid.rows - y - 1) * scale
            draw.rectangle((px, py, px + scale - 1, py + scale - 1), fill=color)
    draw.rectangle((margin, margin, margin + grid.cols * scale, margin + grid.rows * scale), outline=(30, 30, 30), width=2)
    for label, world_path, color in paths:
        if len(world_path) < 2:
            continue
        pts = [world_to_pixel(grid, pt, scale, margin) for pt in world_path]
        draw.line(pts, fill=color, width=max(2, scale // 2), joint="curve")
        draw.text((pts[-1][0] + 4, pts[-1][1] - 4), label, fill=color)
    start_px = world_to_pixel(grid, (START[0], START[1]), scale, margin)
    goal_px = world_to_pixel(grid, (GOAL[0], GOAL[1]), scale, margin)
    r = max(4, scale)
    draw.ellipse((start_px[0] - r, start_px[1] - r, start_px[0] + r, start_px[1] + r), fill=(39, 132, 67))
    draw.ellipse((goal_px[0] - r, goal_px[1] - r, goal_px[0] + r, goal_px[1] + r), fill=(196, 57, 48))
    draw.text((margin, 10), title, fill=(20, 20, 20))
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        image.save(output_path)
    return image


def render_outputs(output_dir: Path, results: List[Dict], paths: Dict[str, Dict]) -> None:
    colors = {
        "dijkstra": (109, 84, 191),
        "astar": (27, 110, 190),
        "weighted_astar": (190, 62, 119),
        "theta_star": (217, 119, 6),
    }
    for map_variant in ["direct_ogm", "pomp_style_ogm"]:
        selected = []
        grid = None
        for planner_name in PLANNERS:
            item = paths[f"{map_variant}:{planner_name}"]
            grid = item["grid"]
            selected.append((planner_name, item["world_path"], colors[planner_name]))
        assert grid is not None
        render_grid(
            grid,
            selected,
            f"{map_variant} planner trajectories",
            output_dir / f"{map_variant}_trajectories.png",
        )

    direct = Image.open(output_dir / "direct_ogm_trajectories.png")
    pomp = Image.open(output_dir / "pomp_style_ogm_trajectories.png")
    summary = Image.new("RGB", (direct.width + pomp.width, max(direct.height, pomp.height)), (255, 255, 255))
    summary.paste(direct, (0, 0))
    summary.paste(pomp, (direct.width, 0))
    summary.save(output_dir / "trajectory_summary.png")
    write_animation(output_dir, paths, colors)


def animation_frames(paths: Dict[str, Dict], colors: Dict[str, Tuple[int, int, int]]) -> List[Image.Image]:
    frames: List[Image.Image] = []
    map_variant = "pomp_style_ogm"
    grid = paths[f"{map_variant}:astar"]["grid"]
    completed: List[Tuple[str, Sequence[Tuple[float, float]], Tuple[int, int, int]]] = []
    for planner_name in PLANNERS:
        full_path = paths[f"{map_variant}:{planner_name}"]["world_path"]
        if len(full_path) < 2:
            continue
        for count in range(2, len(full_path) + 1, max(1, len(full_path) // 40)):
            frame_path = full_path[:count]
            image = render_grid(
                grid,
                completed + [(planner_name, frame_path, colors[planner_name])],
                f"POMP-style OGM animation: {planner_name}",
                None,
                size=(900, 760),
            )
            frames.append(image)
        completed.append((planner_name, full_path, colors[planner_name]))
    return frames


def write_animation(output_dir: Path, paths: Dict[str, Dict], colors: Dict[str, Tuple[int, int, int]]) -> None:
    frames = animation_frames(paths, colors)
    if not frames:
        return
    try:
        import cv2
        import numpy as np

        video_path = output_dir / "planner_animation.mp4"
        first = np.array(frames[0].convert("RGB"))
        height, width = first.shape[:2]
        writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), 8.0, (width, height))
        for frame in frames:
            rgb = np.array(frame.convert("RGB"))
            writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        writer.release()
    except Exception:
        gif_path = output_dir / "planner_animation.gif"
        frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=125, loop=0)


def write_metrics(output_dir: Path, results: List[Dict]) -> None:
    csv_path = output_dir / "metrics.csv"
    fieldnames = [
        "map_variant",
        "planner",
        "success",
        "path_nodes",
        "path_length_m",
        "turn_count",
        "min_grid_clearance_m",
        "expanded_nodes",
        "runtime_ms",
        "free_cells",
        "occupied_cells",
        "boundary_cells",
        "reason",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    (output_dir / "metrics.json").write_text(json.dumps(results, indent=2), encoding="utf-8")

    lines = [
        "# Complex Construction Path Planning Benchmark",
        "",
        "Map variants:",
        "- `direct_ogm`: conservative fixed-cell occupancy.",
        "- `pomp_style_ogm`: sub-cell occupancy projection inspired by POMP and the browser demo.",
        "",
        "Planners: Dijkstra, A*, Weighted A*, Theta*.",
        "",
        "Key files:",
        "- `metrics.csv` / `metrics.json`",
        "- `trajectory_summary.png`",
        "- `direct_ogm_trajectories.png`",
        "- `pomp_style_ogm_trajectories.png`",
        "- `planner_animation.mp4` if OpenCV is available, otherwise `planner_animation.gif`",
        "",
    ]
    (output_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main(argv: Sequence[str]) -> int:
    output_dir = Path(argv[1]) if len(argv) > 1 else DEFAULT_OUTPUT_DIR
    results, paths = run_benchmark(output_dir)
    write_metrics(output_dir, results)
    render_outputs(output_dir, results, paths)
    print(output_dir)
    for row in results:
        print(
            f"{row['map_variant']:16s} {row['planner']:10s} "
            f"success={row['success']} length={row['path_length_m'] or '-'} "
            f"expanded={row['expanded_nodes']} runtime_ms={row['runtime_ms']}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
