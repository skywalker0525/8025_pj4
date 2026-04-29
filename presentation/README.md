# Presentation Speaking Notes

This file gives a slide-by-slide English script for a slower 10-minute presentation. The deck has 12 slides after removing the verification slide. Aim for about 45 to 55 seconds on most slides, then play the recorded task video after the final slide.

## Slide 1: Title

Good morning/afternoon. My name is Tan Tian. Today I am presenting my CIVIL 8025 Project 4, a construction robot digital twin. The system combines ROS2, Gazebo, Nav2, and a local web dashboard. The main demonstration is autonomous navigation from a start point to a wall-mounted AprilTag target.

## Slide 2: Presentation Roadmap

I will first explain what I built and why it fits the digital twin task. Then I will go through the robot model, the simulation environment, autonomous navigation, the web dashboard, SLAM mapping, and safety constraints. At the end, I will show a recorded video of the robot completing the task.

## Slide 3: Project Goal

The background is autonomous robot navigation in construction areas. These environments are cluttered and safety-critical, so robots can help with repeated inspection tasks. In this project, the robot starts at point A, navigates through a site-like indoor world, approaches a wall AprilTag, and reports the mission as complete. The important point is that the robot is not just visual; it has physics, sensors, navigation, and operator feedback.

## Slide 4: Digital Twin Architecture

The system is organized into several layers. Gazebo provides the physical world, collision objects, and simulated sensors. The robot is described with URDF and Xacro. Nav2 handles localization, costmaps, and path planning. My mission node checks whether Nav2 is ready, sends the target goal, and publishes mission events. The local FastAPI dashboard connects to the ROS layer and gives the user a browser-based control interface.

## Slide 5: Robot and Environment Model

The robot is a four-wheel differential-style mobile platform. The rear wheels are driven by one Gazebo diff-drive plugin, while the front wheels act as passive support wheels. It also has a pan-and-tilt camera gimbal, LiDAR, camera, odometry, and TF. The environment contains walls, construction materials, barriers, a tool chest, a simple cylinder column, and a wall AprilTag target.

## Slide 6: Autonomous Mission

This slide shows the simplified top-down map. I removed the route line so the map is easier to read. The robot starts at A and navigates to B, which is the approach point in front of the AprilTag. On the right, I show the map information: it is a 100 by 100 pixel occupancy map with 0.1 meter resolution, so it covers about 10 meters by 10 meters. Nav2 uses this map for planning and obstacle avoidance.

## Slide 7: Web Dashboard

This slide shows the actual local dashboard. The dashboard provides mission controls like Start Auto, Stop, and Reset. It also supports manual driving and camera gimbal control. The live feedback area shows mission state, robot pose, velocity, nearest LiDAR obstacle distance, target information, and safety warnings. This makes the demo easier to operate than using only terminal commands.

## Slide 8: SLAM and Map Workflow

This image shows the mapping and LiDAR scanning view. In mapping mode, I launch Gazebo with SLAM Toolbox and manually drive the robot from the dashboard. RViz shows the map growing from LiDAR observations. After saving the map, the normal navigation mode loads the static room map, AMCL estimates the robot pose, and Nav2 plans through the global and local costmaps.

## Slide 9: Safety and Physical Constraints

Safety is represented at both the model level and runtime level. In the robot model, I define collision geometry, inertia, wheel contact behavior, and gimbal joint limits. In the world, walls and obstacles have collision objects. During runtime, Nav2 uses obstacle and inflation layers, while the dashboard monitors speed limits, nearest obstacle distance, and warnings. The Stop command publishes zero velocity immediately.

## Slide 10: Implementation Highlights

The code is separated into simulation, navigation, mission, and web components. The simulation package contains the Gazebo world and AprilTag target. The navigation package contains Nav2 parameters, maps, waypoints, RViz, and SLAM launch files. The mission package contains readiness checks, event publishing, telemetry, manual control, and markers. The full system can be run with Docker and tmux for reproducible demonstrations.

## Slide 11: Limitations and Future Work

The current version uses the AprilTag mainly as a visual target with known map coordinates. A natural next step is real AprilTag image detection with a bounding box overlay on the camera stream. Another future improvement is to stream the LiDAR or costmap view directly into the web dashboard. Finally, tag pose estimation could be used for more precise final approach control.

## Slide 12: Conclusion

To conclude, this project demonstrates a complete construction robot digital twin with simulation, sensors, navigation, a local dashboard, SLAM workflow, and safety constraints. The robot can complete the autonomous task from A to the wall-mounted AprilTag target. Next, I will play the recorded mission video to show the complete task execution.
