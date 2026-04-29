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
- Four-wheel base: rear left/right wheels are driven by one Gazebo diff-drive plugin that publishes `/odom` and `odom -> base_link`; front wheels are passive support wheels.
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
