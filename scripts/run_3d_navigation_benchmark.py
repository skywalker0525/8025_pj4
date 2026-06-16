#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import select
import shlex
import signal
import subprocess
import sys
import time
from typing import Dict, List, Optional, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WAYPOINT_SUMMARY = (
    REPO_ROOT / "exports" / "planner_waypoints" / "complex_construction_3d_nav" / "summary.csv"
)
DEFAULT_BENCHMARK_DIR = (
    REPO_ROOT / "exports" / "navigation_benchmarks" / "complex_construction_3d_nav"
)


DONE_MARKERS = (
    "Mission state -> DONE",
    "NAVIGATION_SUCCEEDED",
    "AUTO_SEQUENCE_COMPLETED_WITH_FAILURES:",
)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run planner-labelled 3D Gazebo/Nav2 captures and summarize navigation metrics. "
            "Run inside the ROS container from /ws after building the workspace."
        )
    )
    parser.add_argument("--waypoint-summary", type=Path, default=DEFAULT_WAYPOINT_SUMMARY)
    parser.add_argument("--benchmark-dir", type=Path, default=DEFAULT_BENCHMARK_DIR)
    parser.add_argument("--capture-output-dir", default="/ws/exports/livo_image_pose")
    parser.add_argument("--run-prefix", default="")
    parser.add_argument("--map-variants", nargs="*", default=[])
    parser.add_argument("--planners", nargs="*", default=[])
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--timeout-sec", type=float, default=600.0)
    parser.add_argument("--post-done-grace-sec", type=float, default=8.0)
    parser.add_argument("--sample-period-sec", type=float, default=0.30)
    parser.add_argument("--video-fps", type=float, default=10.0)
    parser.add_argument("--pointcloud-sample-period-sec", type=float, default=0.50)
    parser.add_argument("--pointcloud-stride", type=int, default=4)
    parser.add_argument("--max-pointcloud-scans", type=int, default=250)
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args(argv[1:])


def maybe_container_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        rel = resolved.relative_to(REPO_ROOT.resolve())
        return f"/ws/{rel}"
    except ValueError:
        return str(path)


def load_rows(path: Path, map_variants: Sequence[str], planners: Sequence[str]) -> List[Dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(f"Waypoint summary not found: {path}")
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if map_variants:
        rows = [row for row in rows if row.get("map_variant") in map_variants]
    if planners:
        rows = [row for row in rows if row.get("planner") in planners]
    return rows


def output_run_dir(capture_output_dir: str, run_name: str) -> Path:
    if capture_output_dir.startswith("/ws/"):
        return REPO_ROOT / capture_output_dir.removeprefix("/ws/") / run_name
    return Path(capture_output_dir) / run_name


def build_launch_command(row: Dict[str, str], args: argparse.Namespace, run_name: str) -> List[str]:
    waypoints_file = row["waypoints_file"]
    return [
        "xvfb-run",
        "-a",
        "env",
        "LIBGL_ALWAYS_SOFTWARE=1",
        "ros2",
        "launch",
        "my_robot_navigation",
        "complex_construction_livo_dataset_capture.launch.py",
        f"run_name:={run_name}",
        f"waypoints_file:={waypoints_file}",
        f"output_dir:={args.capture_output_dir}",
        f"sample_period_sec:={args.sample_period_sec:.3f}",
        f"video_fps:={args.video_fps:.3f}",
        f"pointcloud_sample_period_sec:={args.pointcloud_sample_period_sec:.3f}",
        f"pointcloud_stride:={int(args.pointcloud_stride)}",
        f"max_pointcloud_scans:={int(args.max_pointcloud_scans)}",
    ]


def run_one(
    row: Dict[str, str],
    args: argparse.Namespace,
    logs_dir: Path,
    run_name: str,
) -> Dict[str, object]:
    command = build_launch_command(row, args, run_name)
    shell_command = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        f"exec {shlex.join(command)}"
    )
    log_path = logs_dir / f"{run_name}.log"
    capture_dir = output_run_dir(args.capture_output_dir, run_name)
    started = time.time()
    done_detected = False
    timeout = False
    return_code: Optional[int] = None
    done_at: Optional[float] = None

    if args.dry_run:
        print(shlex.join(["bash", "-lc", shell_command]))
        return {
            "run_name": run_name,
            "map_variant": row["map_variant"],
            "planner": row["planner"],
            "status": "dry_run",
            "command": shell_command,
            "log_path": str(log_path),
            "capture_dir": str(capture_dir),
        }

    with log_path.open("w", encoding="utf-8") as log_handle:
        proc = subprocess.Popen(
            ["bash", "-lc", shell_command],
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        assert proc.stdout is not None
        try:
            while True:
                ready, _, _ = select.select([proc.stdout], [], [], 0.25)
                if ready:
                    line = proc.stdout.readline()
                    log_handle.write(line)
                    log_handle.flush()
                    if not args.quiet:
                        print(line, end="")
                    if any(marker in line for marker in DONE_MARKERS):
                        done_detected = True
                        done_at = done_at or time.time()
                else:
                    return_code = proc.poll()
                    if return_code is not None:
                        break

                now = time.time()
                if done_detected and done_at is not None and now - done_at >= args.post_done_grace_sec:
                    os_killpg(proc.pid, signal.SIGINT)
                if now - started >= args.timeout_sec:
                    timeout = True
                    os_killpg(proc.pid, signal.SIGINT)
                    break

            try:
                return_code = proc.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                os_killpg(proc.pid, signal.SIGTERM)
                try:
                    return_code = proc.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    os_killpg(proc.pid, signal.SIGKILL)
                    return_code = proc.wait(timeout=5.0)
        finally:
            if proc.poll() is None:
                os_killpg(proc.pid, signal.SIGINT)

    nav_summary = load_json_if_exists(capture_dir / "navigation_summary.json")
    camera_metadata = load_json_if_exists(capture_dir / "camera_metadata.json")
    pointcloud_metadata = load_json_if_exists(capture_dir / "point_cloud" / "lidar_scan_metadata.json")
    livo_metadata = load_json_if_exists(capture_dir / "metadata.json")
    status = "timeout" if timeout else ("done" if done_detected else "exited")
    return {
        "run_name": run_name,
        "map_variant": row["map_variant"],
        "planner": row["planner"],
        "nav_mode": row.get("nav_mode", ""),
        "source_path_length_m": row.get("source_path_length_m", ""),
        "source_path_min_clearance_m": row.get("source_path_min_clearance_m", ""),
        "waypoint_path_length_m": row.get("waypoint_path_length_m", ""),
        "status": status,
        "return_code": return_code,
        "duration_wall_sec": time.time() - started,
        "done_detected": done_detected,
        "timeout": timeout,
        "log_path": str(log_path),
        "capture_dir": str(capture_dir),
        "navigation_success": nav_summary.get("navigation_success"),
        "completed_with_failures": nav_summary.get("completed_with_failures"),
        "reached_waypoint_count": nav_summary.get("reached_waypoint_count"),
        "failed_waypoint_count": nav_summary.get("failed_waypoint_count"),
        "expected_waypoint_count": nav_summary.get("expected_waypoint_count"),
        "actual_path_length_m": nav_summary.get("path_length_m"),
        "runtime_ros_sec": nav_summary.get("runtime_ros_sec"),
        "image_pose_pairs": (
            livo_metadata.get("saved_count")
            or (nav_summary.get("livo_export") or {}).get("image_pose_pairs")
        ),
        "camera_frames": camera_metadata.get("frame_count"),
        "pointcloud_scans": pointcloud_metadata.get("scan_count"),
        "pointcloud_points": pointcloud_metadata.get("point_count"),
    }


def os_killpg(pid: int, sig: signal.Signals) -> None:
    try:
        subprocess.run(["kill", f"-{sig.value}", f"-{pid}"], check=False)
    except Exception:
        pass


def load_json_if_exists(path: Path) -> Dict:
    if not path.exists():
        return {}
    try:
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    except (json.JSONDecodeError, OSError):
        return {}


def write_summary(path: Path, rows: List[Dict[str, object]]) -> None:
    json_path = path / "navigation_benchmark_summary.json"
    csv_path = path / "navigation_benchmark_summary.csv"
    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(rows, handle, indent=2)
    fieldnames = [
        "run_name",
        "map_variant",
        "planner",
        "nav_mode",
        "source_path_length_m",
        "waypoint_path_length_m",
        "status",
        "return_code",
        "duration_wall_sec",
        "navigation_success",
        "completed_with_failures",
        "expected_waypoint_count",
        "reached_waypoint_count",
        "failed_waypoint_count",
        "actual_path_length_m",
        "runtime_ros_sec",
        "image_pose_pairs",
        "camera_frames",
        "pointcloud_scans",
        "pointcloud_points",
        "capture_dir",
        "log_path",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    rows = load_rows(args.waypoint_summary, args.map_variants, args.planners)
    if args.limit > 0:
        rows = rows[: args.limit]
    if not rows:
        print("No runs selected.", file=sys.stderr)
        return 2

    if not args.run_prefix:
        args.run_prefix = datetime.now().strftime("nav3d_%Y%m%d_%H%M%S")
    args.benchmark_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = args.benchmark_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    results: List[Dict[str, object]] = []
    for index, row in enumerate(rows, start=1):
        base_run_name = row.get("run_name") or f"{row['map_variant']}_{row['planner']}_3d"
        run_name = f"{args.run_prefix}_{base_run_name}"
        capture_dir = output_run_dir(args.capture_output_dir, run_name)
        if args.skip_existing and (capture_dir / "navigation_summary.json").exists():
            print(f"[{index}/{len(rows)}] skip existing {run_name}")
            results.append({
                "run_name": run_name,
                "map_variant": row["map_variant"],
                "planner": row["planner"],
                "status": "skipped_existing",
                "capture_dir": str(capture_dir),
            })
            continue
        print(f"[{index}/{len(rows)}] running {run_name}")
        result = run_one(row, args, logs_dir, run_name)
        results.append(result)
        write_summary(args.benchmark_dir, results)

    write_summary(args.benchmark_dir, results)
    print(args.benchmark_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
