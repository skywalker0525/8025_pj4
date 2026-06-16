# CIVIL 8025 Project 4: Construction Robot Digital Twin

This workspace implements a ROS2 Humble + Gazebo Classic + Nav2 + local Web dashboard demo for a construction inspection robot.

The current demo is deliberately stable for presentation: a four-wheel mobile robot starts at point A, uses Nav2 to drive to point B in front of a wall-mounted AprilTag, and marks the task complete when the robot reaches the tag approach zone. The camera stream, camera gimbal, manual driving, telemetry, event log, SLAM workflow, collision objects, and safety warnings remain available.

## Project 4 Fit

- **Robot twin scope:** four-wheel differential mobile base, pan/tilt camera gimbal, LiDAR, RGB camera, odometry, and TF.
- **Site-like scene:** Gazebo room with walls, barriers, material stacks, a rebar cage, a tool chest, a simple inspection column, and a wall-mounted AprilTag.
- **Executable twin:** Nav2 drives from A to the AprilTag approach point B; mission logic publishes readiness, success, fallback, stop, and reset events.
- **Browser dashboard:** local FastAPI page for `Start Auto`, `Stop`, `Reset`, manual driving, camera gimbal control, live camera stream, telemetry, warnings, and event logs.
- **Physics and constraints:** URDF collision/inertia/joint limits, Gazebo collision objects, Nav2 costmap inflation, velocity limits, safety distance thresholds, and gimbal joint-limit monitoring.

## Workspace Layout

```text
8025_pj4/
├── README.md
├── docker/
├── docker-compose.yml
├── src/
│   ├── my_robot_description/   # URDF/Xacro robot, gimbal, ros2_control config
│   ├── my_robot_mission/       # mission, manual control, telemetry, markers
│   ├── my_robot_navigation/    # Nav2 launch, SLAM launch, map, waypoints, params, RViz
│   └── my_robot_sim/           # Gazebo world, simulation launch, local model/tag assets
└── web/
    ├── backend/                # FastAPI + rclpy ROS bridge
    └── static/                 # local HTML/CSS/JS dashboard served by FastAPI
```

## Main Interfaces

| Purpose | Interface |
| --- | --- |
| Start/stop/reset mission | `/mission/command` (`std_msgs/String`: `START_AUTO`, `STOP`, `RESET`) |
| Mission state | `/mission/state` |
| Event log | `/mission/events` |
| Telemetry JSON | `/mission/telemetry` |
| Web manual drive input | `/mission/manual_cmd_vel` |
| Robot velocity command | `/cmd_vel` |
| Web manual camera input | `/mission/manual_gimbal` |
| Camera gimbal controller | `/camera_gimbal_position_controller/commands` |
| AMCL initial pose | `/initialpose` |
| Local camera stream | `http://localhost:8000/camera/stream` |
| Nav2 goal action | `navigate_to_pose` |
| Sensors | `/scan`, `/camera/image_raw`, `/odom`, `/joint_states`, `/tf` |

Legacy orbit/video nodes remain in the package for reference, but the default `nav_demo.launch.py` now runs the simpler AprilTag arrival task.

## Docker Workflow

Host preparation for Gazebo/RViz GUI apps on Linux:

```bash
xhost +local:root
```

Build and start the persistent container:

```bash
cd /home/luke/Documents/8025_pj4
docker compose build
docker compose up -d ros_humble_sim
docker exec -it ros_humble_orbit_demo bash
```

Inside the container:

```bash
cd /ws
colcon build
source install/setup.bash
```

The Docker image contains ROS, Gazebo, Nav2, SLAM Toolbox, FastAPI, OpenCV, and the local dashboard assets. The web interface does not need an external React/Node server.

## Run The Demo

Recommended tmux workflow inside the Docker container:

```bash
docker exec -it ros_humble_orbit_demo bash
cd /ws
colcon build
source install/setup.bash
tmux-dev
```

tmux window 1: start Gazebo, Nav2, RViz, mission nodes, manual control, telemetry, and markers.

```bash
cd /ws
source install/setup.bash
ros2 launch my_robot_navigation nav_demo.launch.py
```

For headless checks:

```bash
xvfb-run -a env LIBGL_ALWAYS_SOFTWARE=1 ros2 launch my_robot_navigation nav_demo.launch.py gui:=false rviz:=false
```

tmux window 2: start the local dashboard and ROS/Web bridge.

```bash
cd /ws
source install/setup.bash
uvicorn web.backend.ros_bridge:app --host 0.0.0.0 --port 8000
```

Open:

```text
http://localhost:8000
```

## Export Image And Camera Poses

Use the LIO-style capture launch when you need a reconstruction dataset made of RGB frames and the camera pose at each image timestamp. This branch adds a simulated IMU topic `/imu/data`, a 3D LiDAR PointCloud2 topic `/points_raw`, and an exporter that waits for both before saving images. The robot pose chain is LiDAR/IMU based: AMCL uses `/scan` for `map -> odom`, `robot_localization` fuses `/odom` and `/imu/data` for `odom -> base_link`, and the exporter looks up `map -> camera_optical_frame` at each image timestamp. The launch records during the automatic `NAVIGATING` mission state by default.

Inside the Docker container:

```bash
cd /ws
colcon build
source install/setup.bash
ros2 launch my_robot_navigation livo_dataset_capture.launch.py gui:=false rviz:=false auto_start:=true
```

By default `max_frames:=0`, so the exporter records the full automatic navigation sequence and stops when the mission reaches `DONE` or `STOPPED`. Use a positive `max_frames` value only for quick smoke tests.

The output is written under:

```text
/ws/exports/livo_image_pose/<capture_timestamp>/
```

Main files:

- `images/*.png`: exported camera frames.
- `poses.csv`: `T_parent_camera` pose for each image, looked up at the image timestamp.
- `transforms.json`: camera-to-parent matrices and intrinsics for 3DGS-style tools.
- `sparse/0/cameras.txt` and `sparse/0/images.txt`: COLMAP-style camera intrinsics and world-to-camera poses.
- `poses_colmap_w2c.txt`: desktop-compatible `name qw qx qy qz tx ty tz` world-to-camera poses.
- `camera_centers_world.txt`: desktop-compatible camera centers `name Cx Cy Cz` in the parent frame.
- `image_name_mapping.csv`: identity mapping for exported image names.
- `camera.mp4` and `overhead.mp4`: onboard camera video and overhead camera video recorded during `NAVIGATING`.

By default the parent frame is `map`. If a FAST-LIVO2-style LIO backend publishes a different world frame, launch with `pose_parent_frame:=<lio_world_frame>` and keep `camera_frame:=camera_optical_frame`.

### Complex Small House Capture

The complex indoor capture uses the official AWS RoboMaker Small House World. Fetch it into the ignored `.external_worlds/` directory before launching:

```bash
cd /ws
./scripts/fetch_aws_small_house_world.sh
colcon build
source install/setup.bash
```

Run the full top-left to bottom-right navigation sequence:

```bash
xvfb-run -a env LIBGL_ALWAYS_SOFTWARE=1 ros2 launch my_robot_navigation small_house_livo_dataset_capture.launch.py run_name:=small_house_full_video_20260615 sample_period_sec:=0.25
```

This branch was verified with two exported datasets:

- `exports/livo_image_pose/old_room_full_video_20260615`: 134 image/pose pairs, onboard video, overhead video, and desktop-compatible pose files.
- `exports/livo_image_pose/small_house_full_video_20260615`: 450 image/pose pairs, onboard video, overhead video, and desktop-compatible pose files.

Both folders were also copied to:

```text
/media/luke/Extreme Pro/old_room_full_video_20260615
/media/luke/Extreme Pro/small_house_full_video_20260615
```

### Complex Construction POMP Benchmark

This branch also adds a larger generated construction-site map for path-planning experiments. The scene is defined in `scripts/complex_construction_scene.py` and generated into Gazebo, Nav2 map, waypoint, and parameter assets by:

```bash
cd /ws
python3 scripts/generate_complex_construction_assets.py
colcon build
source install/setup.bash
```

The generated assets are:

- `src/my_robot_sim/worlds/complex_construction_site.world`
- `src/my_robot_navigation/maps/complex_construction_map.yaml`
- `src/my_robot_navigation/maps/complex_construction_map.pgm`
- `src/my_robot_navigation/config/waypoints_complex_construction.yaml`
- `src/my_robot_navigation/config/nav2_params_complex_construction.yaml`

Run the POMP-style occupancy-grid benchmark against common planners:

```bash
cd /ws
python3 scripts/benchmark_path_planners.py /ws/exports/planning_benchmarks/complex_construction_benchmark_20260616
```

The benchmark compares direct coarse OGM projection with a POMP-style subcell OGM projection, then runs Dijkstra, A*, Weighted A*, and Theta*. Verified local outputs:

```text
exports/planning_benchmarks/complex_construction_benchmark_20260616/metrics.csv
exports/planning_benchmarks/complex_construction_benchmark_20260616/metrics.json
exports/planning_benchmarks/complex_construction_benchmark_20260616/direct_ogm_trajectories.png
exports/planning_benchmarks/complex_construction_benchmark_20260616/pomp_style_ogm_trajectories.png
exports/planning_benchmarks/complex_construction_benchmark_20260616/trajectory_summary.png
exports/planning_benchmarks/complex_construction_benchmark_20260616/planner_animation.mp4
```

Final benchmark summary:

| Map variant | Planner | Success | Length m | Expanded nodes | Runtime ms |
| --- | --- | ---: | ---: | ---: | ---: |
| direct OGM | Dijkstra | yes | 34.6818 | 5888 | 25.6304 |
| direct OGM | A* | yes | 34.6818 | 4904 | 22.7830 |
| direct OGM | Weighted A* | yes | 34.7990 | 4207 | 20.0415 |
| direct OGM | Theta* | yes | 32.8685 | 4483 | 162.7424 |
| POMP-style OGM | Dijkstra | yes | 23.3078 | 6319 | 30.2212 |
| POMP-style OGM | A* | yes | 23.3078 | 2263 | 10.9218 |
| POMP-style OGM | Weighted A* | yes | 24.3907 | 234 | 1.1889 |
| POMP-style OGM | Theta* | yes | 21.8283 | 1228 | 70.7606 |

Run the full complex-map image/pose capture:

```bash
cd /ws
source install/setup.bash
xvfb-run -a env LIBGL_ALWAYS_SOFTWARE=1 ros2 launch my_robot_navigation complex_construction_livo_dataset_capture.launch.py run_name:=complex_construction_full_20260616_v3 sample_period_sec:=0.30
```

The verified full-run dataset is local only and was not moved to an external drive:

```text
exports/livo_image_pose/complex_construction_full_20260616_v3/
```

It contains 1023 RGB images, 1023 timestamped camera poses in `poses.csv`, COLMAP-style `sparse/0/`, `poses_colmap_w2c.txt`, `camera_centers_world.txt`, `transforms.json`, `camera.mp4`, and `overhead.mp4`. The mission reached every waypoint from `P1` through `B` and finished with `NAVIGATION_SUCCEEDED`.

### Online LIO/POMP Coverage Simulation

For unknown or semi-known scene experiments, run the online coverage simulator. It starts from an unknown local grid, synthesizes LiDAR observations against the complex construction scene, treats the current pose as drift-free LIO output, projects the discovered local map into a POMP-style planning grid, and plans coverage/frontier goals for reconstruction sampling.

Run the safe full comparison:

```bash
cd /ws
python3 scripts/simulate_online_lio_pomp_coverage.py /ws/exports/online_lio_pomp_coverage/complex_construction_safe_20260616 --planner-timeout 5 --theta-timeout 5 --max-expanded-nodes 50000 --quiet-planner-log
```

The online comparison includes Dijkstra, A*, Weighted A*, and `theta_star`. In this simulator, `theta_star` intentionally does not run full Theta* global search in the online loop; it uses bounded Weighted A* as the main planner and then applies cached Theta-style line-of-sight shortcut smoothing to the returned path. This prevents high CPU load on complex maps while preserving an any-angle path-smoothing comparison.

Useful safety options:

```bash
python3 scripts/simulate_online_lio_pomp_coverage.py /ws/exports/online_lio_pomp_coverage/no_theta --skip-theta-star --quiet-planner-log
python3 scripts/simulate_online_lio_pomp_coverage.py /ws/exports/online_lio_pomp_coverage/debug_theta --algorithms theta_star --theta-timeout 5 --planner-timeout 5 --max-expanded-nodes 50000
```

Verified local output:

```text
exports/online_lio_pomp_coverage/complex_construction_safe_20260616/
```

Each planner folder contains:

- `*_overhead_lidar.mp4`: top-down LiDAR mapping and route video.
- `*_onboard.mp4`: simulated onboard camera video.
- `*_trajectory.png`: route plot for that planner.
- `samples/images/*.png` and `samples/poses.csv`: reconstruction samples and corresponding poses.
- `metadata.json`: route and run metadata.

The root output folder contains:

- `metrics.csv` and `metrics.json`: numeric comparison.
- `metrics_table.png`: rendered metric table.
- `route_comparison.png`: route comparison across planners.
- `patches/simulate_online_lio_pomp_coverage_safe_theta.diff`: full patch for the safe online simulator.

Final online coverage summary:

| Planner | Success | Free coverage | Surface coverage | Path m | Samples | Expanded nodes | Plan ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Dijkstra | yes | 0.771292 | 0.585387 | 72.755749 | 72 | 37371 | 346.103034 |
| A* | no | 0.757851 | 0.576660 | 69.942850 | 68 | 8057 | 76.494268 |
| Weighted A* | yes | 0.728656 | 0.580424 | 80.591378 | 74 | 2895 | 26.440310 |
| Theta-style shortcut | no | 0.622218 | 0.375941 | 45.106140 | 23 | 587 | 6.441236 |

## Automatic Mode

1. Open Gazebo/RViz and the Web Dashboard.
2. Wait until `/mission/events` or the dashboard event log shows `NAV2_READY`.
3. The launch file publishes the A-point AMCL initial pose automatically. If needed, use RViz `2D Pose Estimate` near point A to correct AMCL.
4. Click `Start Auto` in the dashboard.
5. The robot navigates from A `(-3.8, -3.6)` to B `(-1.5, 3.65)`.
6. B is the approach point in front of the wall AprilTag at `(-1.5, 4.93, 1.05)`.
7. On success, `/mission/events` publishes `NAVIGATION_SUCCEEDED` and `APRILTAG_REACHED`, and `/mission/state` becomes `DONE`.
8. If Nav2 reports a late failure while the robot is already close enough to the tag, the mission publishes `NAVIGATION_FALLBACK_APRILTAG_REACHED` and still finishes as `DONE`.

Equivalent command-line trigger:

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: START_AUTO}"
```

Emergency stop and reset:

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: STOP}"
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: RESET}"
```

## Manual Mode

Use the dashboard drive buttons to publish manual velocity commands:

- Forward/reverse control `/mission/manual_cmd_vel`.
- Left/right control angular velocity.
- `Stop` publishes zero velocity.
- Camera sliders publish `/mission/manual_gimbal` with pan and tilt values.
- The camera panel displays `/camera/image_raw` through `/camera/stream`.

## SLAM Mapping and Saving

Use this mode when you want RViz to show the LiDAR-built map and save a fresh static map for Nav2.

tmux window 1:

```bash
cd /ws
source install/setup.bash
ros2 launch my_robot_navigation slam_mapping.launch.py
```

Headless option:

```bash
xvfb-run -a env LIBGL_ALWAYS_SOFTWARE=1 ros2 launch my_robot_navigation slam_mapping.launch.py gui:=false rviz:=false
```

tmux window 2:

```bash
cd /ws
source install/setup.bash
uvicorn web.backend.ros_bridge:app --host 0.0.0.0 --port 8000
```

Open `http://localhost:8000`, drive the robot manually around the site, and watch `/map` grow in RViz. Save the discovered map:

```bash
cd /ws
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/my_robot_navigation/maps/room_map
```

After saving, rebuild or source the workspace if needed, then relaunch normal navigation:

```bash
colcon build
source install/setup.bash
ros2 launch my_robot_navigation nav_demo.launch.py
```

## Configuration

Primary files:

- [robot.urdf.xacro](/home/luke/Documents/8025_pj4/src/my_robot_description/urdf/robot.urdf.xacro)
- [controllers.yaml](/home/luke/Documents/8025_pj4/src/my_robot_description/config/controllers.yaml)
- [indoor_room.world](/home/luke/Documents/8025_pj4/src/my_robot_sim/worlds/indoor_room.world)
- [waypoints.yaml](/home/luke/Documents/8025_pj4/src/my_robot_navigation/config/waypoints.yaml)
- [nav2_params.yaml](/home/luke/Documents/8025_pj4/src/my_robot_navigation/config/nav2_params.yaml)
- [room_map.yaml](/home/luke/Documents/8025_pj4/src/my_robot_navigation/maps/room_map.yaml)

Important `waypoints.yaml` values:

- `points.A`: robot spawn and AMCL initial pose.
- `points.B`: automatic navigation goal in front of the wall AprilTag.
- `apriltag_target`: AprilTag ID, family, wall position, approach pose, and completion radius.
- `target_object`: simple cylinder column obstacle position and radius.
- `safety`: obstacle warning distance, velocity limits, and gimbal limits.

## Assets and Licenses

- AprilTag: AprilRobotics `apriltag-imgs`, `tag36h11` ID 0, used as a visual-only fiducial marker on the north wall.
- The active inspection column is the simple Gazebo cylinder in `indoor_room.world` for stable collision and planning.
- The local `inspection_column` model asset folder is retained because it stores the AprilTag material/texture used by the wall marker.

## Telemetry and Safety Evidence

The `telemetry_node` publishes `/mission/telemetry` as JSON for the dashboard. It includes:

- robot pose and odometry speed;
- commanded velocity;
- camera pan/tilt joint positions;
- nearest valid LiDAR obstacle range;
- AprilTag target metadata;
- recent mission events;
- warnings for low obstacle distance, velocity limit violation, and camera tilt near joint limits.

Physics and collision constraints are represented in four places:

- URDF/Xacro: link collision geometry, mass, inertia, and gimbal joint limits.
- Four-wheel base: rear left/right wheels are driven by one Gazebo diff-drive plugin that publishes `/odom`; `robot_localization` fuses `/odom` with `/imu/data` and publishes `odom -> base_link`; front wheels are passive support wheels.
- Gazebo world: collision geometry for walls, materials, barriers, rebar cage, tool chest, and the simple cylinder column.
- Nav2: robot radius, obstacle layers, and inflation layers in the local/global costmaps.
- Mission safety: runtime telemetry warnings plus stop/reset commands.

## Troubleshooting

If `Start Auto` appears to do nothing:

```bash
ros2 topic echo /mission/events
ros2 action list | grep navigate
ros2 run tf2_ros tf2_echo map base_link
```

- `NAV2_READY` means the navigation action server and `map -> base_link` TF are available.
- `NAV2_NOT_READY:ACTION_SERVER_UNAVAILABLE` means Nav2 is still starting or failed to activate.
- `NAV2_NOT_READY:TF_map_TO_base_link_UNAVAILABLE` means AMCL has not produced `map -> odom` yet.

For `Invalid frame ID "odom"`:

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic info /cmd_vel
```

If `/odom` is missing, restart `nav_demo.launch.py` after rebuilding. The current robot uses one rear diff-drive Gazebo plugin. If `/scan`, `/camera/image_raw`, and `/odom` all have no publisher, check Gazebo X/OpenGL authorization and use either `xvfb-run` or `xhost +local:root`.

For `Invalid frame ID "map"`:

1. Wait a few more seconds after Gazebo/RViz opens.
2. Confirm `INITIAL_POSE_PUBLISHED` appears on `/mission/events`.
3. Use RViz `2D Pose Estimate` at point A `(-3.8, -3.6)` if AMCL needs correction.
4. Confirm the saved map exists at `src/my_robot_navigation/maps/room_map.yaml`.
5. If the map no longer matches the world, regenerate it with `slam_mapping.launch.py` and `map_saver_cli`.

If Nav2 says `failed to create plan`:

```bash
ros2 topic echo /mission/events
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /global_costmap/costmap --once
```

Then check that B is still `(-1.5, 3.65)` and that the current `room_map.pgm` matches `indoor_room.world`.

## Useful Checks

```bash
ros2 topic list
ros2 topic echo /mission/state
ros2 topic echo /mission/telemetry
ros2 topic echo /mission/events
ros2 topic echo /odom --once
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 action list
```

Check the local camera stream:

```text
http://localhost:8000/camera/stream
```

Validate the camera controller:

```bash
ros2 topic pub --once /camera_gimbal_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.1]}"
```

## Test Checklist

- `colcon build` succeeds.
- Gazebo shows the room, simple cylinder column, wall AprilTag, and four-wheel robot at A.
- `/scan`, `/camera/image_raw`, `/odom`, `/joint_states`, and `/tf` publish.
- `ros2 run tf2_ros tf2_echo odom base_link` returns transforms.
- `ros2 run tf2_ros tf2_echo map base_link` returns transforms after AMCL initializes.
- `/mission/events` shows `NAV2_READY` before pressing `Start Auto`.
- Dashboard connects to the FastAPI bridge and shows the local camera stream.
- `Start Auto` sends the robot from A to the AprilTag approach point B.
- On arrival, `/mission/events` shows `APRILTAG_REACHED` and `/mission/state` is `DONE`.
- Manual drive and gimbal controls work from the dashboard.
- SLAM mode can build `/map`, and `map_saver_cli` can save `room_map.yaml/pgm`.
