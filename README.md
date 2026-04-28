# CIVIL 8025 Project 4: Construction Robot Digital Twin

This workspace implements a ROS2 Humble + Gazebo Classic + Nav2 + Web digital twin demo for a construction inspection robot.

The robot starts at point A, navigates automatically to point B near a column, then performs a full 360-degree orbit while a pan/tilt camera gimbal tracks the column and records video. A local browser dashboard provides automatic mission control, manual driving, camera control, live camera streaming, telemetry, event logs, and safety/constraint monitoring.

## Project 4 Fit

- **Robot twin scope:** four-wheel differential mobile base + pan/tilt camera arm/gimbal + LiDAR + RGB camera.
- **Site-like scene:** enlarged Gazebo construction room with walls, barriers, material stacks, a rebar cage, a tool chest, and an inspection column.
- **Executable twin:** Nav2 drives A to B, custom mission nodes trigger orbit capture, and `ros2_control` commands the camera gimbal.
- **Telemetry:** dashboard-visible pose, velocity, gimbal joints, nearest obstacle distance, orbit progress, live camera stream, video status, warnings, and events.
- **Physics and constraints:** URDF collision/inertia/joint limits, Gazebo collision objects, Nav2 costmap inflation, velocity limits, safety distance thresholds, and gimbal joint-limit monitoring.

## Workspace Layout

```text
8025_pj4/
├── README.md
├── docker/
├── docker-compose.yml
├── src/
│   ├── my_robot_description/   # URDF/Xacro robot, gimbal, ros2_control config
│   ├── my_robot_mission/       # mission, orbit, manual control, telemetry, video nodes
│   ├── my_robot_navigation/    # Nav2 launch, map, waypoints, params, RViz
│   └── my_robot_sim/           # Gazebo world and simulation launch
└── web/
    ├── backend/                # FastAPI + rclpy ROS bridge
    └── static/                 # local HTML/CSS/JS dashboard served by FastAPI
```

## Main Interfaces

| Purpose | Interface |
| --- | --- |
| Start/stop/reset mission | `/mission/command` (`std_msgs/String`: `START_AUTO`, `STOP`, `RESET`) |
| Mission state | `/mission/state` |
| Web manual drive input | `/mission/manual_cmd_vel` |
| Robot velocity command | `/cmd_vel` |
| Web manual camera input | `/mission/manual_gimbal` |
| Camera gimbal controller | `/camera_gimbal_position_controller/commands` |
| Telemetry JSON | `/mission/telemetry` |
| Event log | `/mission/events` |
| Orbit status JSON | `/mission/orbit_status` |
| Video status JSON | `/mission/video_status` |
| Local web camera stream | `http://localhost:8000/camera/stream` |
| Nav2 goal action | `navigate_to_pose` |
| Sensors | `/scan`, `/camera/image_raw`, `/odom`, `/joint_states`, `/tf` |

## Docker Workflow

Host preparation for GUI apps on Linux:

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

## Native Prerequisites

Target environment:

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11
- Python 3 with FastAPI/Uvicorn

Recommended ROS packages:

```bash
sudo apt update
sudo apt install \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-cv-bridge \
  ros-humble-xacro \
  python3-opencv \
  python3-pip
pip3 install fastapi "uvicorn[standard]"
```

## Run the Digital Twin Demo

Recommended tmux workflow inside the Docker container:

```bash
docker exec -it ros_humble_orbit_demo bash
cd /ws
colcon build
source install/setup.bash
tmux-dev
```

tmux window 1: start Gazebo, Nav2, RViz, mission, telemetry, and video nodes.

```bash
cd /ws
source install/setup.bash
ros2 launch my_robot_navigation nav_demo.launch.py
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

The page, JavaScript, CSS, telemetry WebSocket, manual controls, and camera stream are all served locally by FastAPI. No second web server is required.

## Operating Modes

### Automatic Mode

1. Open Gazebo/RViz and the Web Dashboard.
2. Wait until `/mission/events` or the dashboard event log shows `NAV2_READY`.
3. If needed, use RViz `2D Pose Estimate` near point A to correct AMCL.
4. Click `Start Auto` in the dashboard.
4. The robot navigates from A to B using Nav2.
5. When navigation succeeds, the mission enters `ORBITING`.
6. The base circles the inspection column once while the pan/tilt gimbal tracks the column.
7. The video recorder writes the orbit capture to `~/ros_videos`.

Equivalent command-line trigger:

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: START_AUTO}"
```

### Manual Mode

Use the dashboard drive buttons to publish manual velocity commands:

- Forward/reverse control `/mission/manual_cmd_vel`.
- Left/right control angular velocity.
- `Stop` publishes zero velocity.
- Camera sliders publish `/mission/manual_gimbal` with pan and tilt values.
- The camera panel displays `/camera/image_raw` through the local MJPEG endpoint `/camera/stream`.

## SLAM Mapping and Saving

Use this mode when you want RViz to show the LiDAR-built map and save a fresh static map for Nav2.

tmux window 1: launch Gazebo, SLAM Toolbox, RViz, manual control, and telemetry.

```bash
cd /ws
source install/setup.bash
ros2 launch my_robot_navigation slam_mapping.launch.py
```

tmux window 2: launch the same local dashboard.

```bash
cd /ws
source install/setup.bash
uvicorn web.backend.ros_bridge:app --host 0.0.0.0 --port 8000
```

Open `http://localhost:8000`, drive the robot manually around the site, and watch the `/map` display grow in RViz. Save the discovered map:

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

Emergency stop from command line:

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: STOP}"
```

Reset mission state:

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: RESET}"
```

## Configuration

Primary configuration files:

- [robot.urdf.xacro](/home/luke/Documents/8025_pj4/src/my_robot_description/urdf/robot.urdf.xacro)
- [controllers.yaml](/home/luke/Documents/8025_pj4/src/my_robot_description/config/controllers.yaml)
- [indoor_room.world](/home/luke/Documents/8025_pj4/src/my_robot_sim/worlds/indoor_room.world)
- [waypoints.yaml](/home/luke/Documents/8025_pj4/src/my_robot_navigation/config/waypoints.yaml)
- [nav2_params.yaml](/home/luke/Documents/8025_pj4/src/my_robot_navigation/config/nav2_params.yaml)

Important `waypoints.yaml` values:

- `points.A`: robot spawn/initial point.
- `points.B`: automatic navigation goal near the column.
- `target_object`: column position and radius.
- `orbit`: orbit radius, speed, and camera height.
- `safety`: obstacle warning distance, velocity limits, orbit error threshold, and gimbal limits.

## Telemetry and Safety Evidence

The `telemetry_node` publishes `/mission/telemetry` as JSON for the dashboard. It includes:

- robot pose and odometry speed;
- commanded velocity;
- camera pan/tilt joint positions;
- nearest valid LiDAR obstacle range;
- orbit progress and radius error;
- video recording status and output path;
- recent mission events;
- warnings for low obstacle distance, velocity limit violation, large orbit radius error, and camera tilt near joint limits.

Physics and collision constraints are represented in four places:

- URDF/Xacro: link collision geometry, mass, inertia, and gimbal joint limits.
- Four-wheel base: front/rear left and front/rear right wheel joints are driven through the same `/cmd_vel` diff-drive interface for smoother contact than the old caster model.
- Gazebo world: collision geometry for walls, construction materials, barriers, rebar cage, and column.
- Nav2: robot radius, obstacle layers, and inflation layers in the local/global costmaps.
- Mission safety: runtime telemetry warnings and stop/reset commands.

## Troubleshooting

If `Start Auto` appears to do nothing, first check:

```bash
ros2 topic echo /mission/events
ros2 action list | grep navigate
ros2 run tf2_ros tf2_echo map base_link
```

- `NAV2_READY` means the navigation action server and `map -> base_link` TF are available.
- `NAV2_NOT_READY:ACTION_SERVER_UNAVAILABLE` means Nav2 is still starting or failed to activate.
- `NAV2_NOT_READY:TF_map_TO_base_link_UNAVAILABLE` means AMCL has not produced the `map -> odom` transform yet.

For `Invalid frame ID "map"` or `Timed out waiting for transform from base_link to map`:

1. Wait a few more seconds after Gazebo/RViz opens.
2. Use RViz `2D Pose Estimate` at point A `(-3.8, -3.6)`.
3. Confirm the saved map exists at `src/my_robot_navigation/maps/room_map.yaml`.
4. If the map no longer matches the world, regenerate it with `slam_mapping.launch.py` and `map_saver_cli`.

## Useful Checks

```bash
ros2 topic list
ros2 topic echo /mission/state
ros2 topic echo /mission/telemetry
ros2 topic echo /mission/events
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo map base_link
ros2 action list
```

Check the local camera stream in a browser:

```text
http://localhost:8000/camera/stream
```

Validate the camera controller:

```bash
ros2 topic pub --once /camera_gimbal_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.1]}"
```

## Test Checklist

- `colcon build` succeeds.
- Gazebo shows the enlarged construction scene and the robot at A.
- `/joint_states` includes `front_left_wheel_joint`, `rear_left_wheel_joint`, `front_right_wheel_joint`, and `rear_right_wheel_joint`.
- `/scan`, `/camera/image_raw`, `/odom`, `/joint_states`, and `/tf` publish.
- `/mission/events` shows `NAV2_READY` before pressing `Start Auto`.
- Dashboard connects to the FastAPI bridge.
- Dashboard shows the local camera stream without a second web server.
- `Start Auto` sends the robot from A to B.
- Orbit capture starts after navigation succeeds.
- Camera pan/tilt tracks the column during orbit.
- Video is saved under `~/ros_videos`.
- Manual drive and gimbal controls work from the dashboard.
- Safety warnings appear when the robot approaches obstacles or constraints.

## Notes

- The included map is an approximate static map for demonstration. For final presentation quality, regenerate the map with SLAM after changing the Gazebo world.
- The pan/tilt camera gimbal is the project’s simplified mobile-manipulator element. It is intentionally focused on inspection video capture rather than object grasping.
- RViz remains useful for engineering validation, while the local FastAPI-served dashboard is the Project 4 browser-based experience layer.
