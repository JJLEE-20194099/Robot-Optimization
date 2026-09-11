# Robot-Optimization

A ROS 2 and Python-based drone path optimization project focused on motion planning, trajectory smoothing, and visualization for robotic systems.

## Overview

This repository contains a collection of scripts and ROS 2 package components for experimenting with path planning and trajectory optimization techniques, especially for drones. The project explores several approaches, including:

- RRT-based path generation
- Kinodynamic RRT* planning
- B-spline smoothing
- Bezier smoothing
- SE(3) / Lie group based trajectory update
- ROS 2 message publishing for visualization in tools such as Foxglove and RViz

The project is structured as a ROS 2 package under `src/drone_controller`, with helper scripts placed at the workspace root for standalone experimentation and reference implementations.

## Key Features

- Robot motion planning using RRT and RRT*
- 3D obstacle-aware path generation
- Path smoothing with B-spline and Bezier methods
- SE(3) pose optimization using exponential/logarithm mappings
- ROS 2 publishers for visualization of paths, obstacles, tree structures, and drone markers
- Docker-based setup for quick local execution

## Repository Structure

```text
Robot-Optimization/
├── Dockerfile
├── docker-compose.yml
├── entrypoint.sh
├── manual_setup_logsbook.txt
├── path_optimization.py
├── rrt_star_path_optimization.py
├── README.md
├── src/
│   └── drone_controller/
│       ├── package.xml
│       ├── setup.py
│       ├── launch/
│       │   └── drone_launch.py
│       ├── resource/
│       │   ├── drone_controller
│       │   └── drone.urdf
│       ├── test/
│       │   ├── test_copyright.py
│       │   ├── test_flake8.py
│       │   └── test_pep257.py
│       └── drone_controller/
│           ├── __init__.py
│           ├── bspline_rrt_path_smooth.py
│           ├── bspline_smooth_on_fix_rrt_path.py
│           ├── optimizer_path.py
│           ├── rrt_path.py
│           └── rrt_path_beizer_smooth.py
```

## Main Scripts

### Root-level scripts

- `path_optimization.py`
  - Simple SE(3) path optimization example.
  - Uses exponential maps and logarithmic error between current and target poses.
  - Produces a 3D path plot with Matplotlib.

- `rrt_star_path_optimization.py`
  - Standalone kinodynamic RRT* demonstration.
  - Generates a 3D path while avoiding obstacles and optimizing a cost function.
  - Visualization is done with Matplotlib.

### ROS 2 package scripts

- `src/drone_controller/drone_controller/rrt_path.py`
  - ROS 2 node that performs RRT* style drone path planning.
  - Publishes path, obstacle markers, tree visualization, camera-like output, and start/goal markers.

- `src/drone_controller/drone_controller/rrt_path_beizer_smooth.py`
  - Similar to `rrt_path.py`, but includes Bezier-based smoothing of the generated path.

- `src/drone_controller/drone_controller/bspline_rrt_path_smooth.py`
  - Builds an RRT path and then smooths it using a B-spline with SE(3) control representation.
  - This is the main optimization flow used by the Docker Compose setup.

- `src/drone_controller/drone_controller/bspline_smooth_on_fix_rrt_path.py`
  - Takes an existing RRT path and applies custom B-spline smoothing for visualization and trajectory refinement.

- `src/drone_controller/drone_controller/optimizer_path.py`
  - Simple ROS 2 publisher for a Lie-group-based path optimization demo.
  - Publishes a `nav_msgs/Path` message over time.

### ROS launch and package files

- `src/drone_controller/launch/drone_launch.py`
  - Launch file for launching the robot state publisher.

- `src/drone_controller/setup.py`
  - ROS Python package configuration.

- `src/drone_controller/package.xml`
  - ROS package manifest with dependencies.

## How It Works

The project combines several common planning and smoothing ideas:

1. Generate an initial path using random sampling or RRT-style expansion.
2. Evaluate collision constraints against obstacle spheres.
3. Reconstruct or smooth the path using B-spline or Bezier interpolation.
4. Publish the resulting trajectory and related markers through ROS 2 topics.
5. Visualize the drone, path, obstacles, and tree in tools such as Foxglove.

The implementation uses SE(3) transforms to represent robot poses and applies Lie algebra operations for motion updates and error calculation.

## Dependencies

The project depends on:

- ROS 2 Humble
- Python 3
- NumPy
- SciPy
- OpenCV
- cv_bridge
- Foxglove Bridge
- Matplotlib

Most of the runtime dependencies are installed automatically by the included Docker image.

## Running with Docker

This repository includes a Docker-based workflow that is convenient for simulation and visualization.

### Start all services

```bash
docker compose up --build
```

### Services included

- `foxglove_bridge`
  - Exposes a Foxglove-compatible bridge on port `8765`

- `drone_launch`
  - Launches the ROS 2 robot state publisher

- `drone_optimizer`
  - Runs the main optimizer node, currently executing `bspline_rrt_path_smooth.py`

### Configuration notes

The `docker-compose.yml` file maps the workspace source into the container and runs:

```bash
python3 -u /ros2_ws/src/drone_controller/drone_controller/bspline_rrt_path_smooth.py
```

This means the Docker setup is currently oriented around the B-spline RRT smoothing workflow.

## Manual ROS 2 Setup

If you want to build and run the project manually:

### 1. Clone the repository

```bash
git clone <repository-url>
cd Robot-Optimization
```

### 2. Build the ROS 2 workspace

```bash
source /opt/ros/humble/setup.bash
colcon build
```

### 3. Source the workspace

```bash
source install/setup.bash
```

### 4. Launch the package

```bash
ros2 launch drone_controller drone_launch.py
```

### 5. Run the optimizer node

```bash
python3 src/drone_controller/drone_controller/bspline_rrt_path_smooth.py
```

Or run another available node, for example:

```bash
python3 src/drone_controller/drone_controller/rrt_path.py
```

## Visualization

The project publishes several ROS topics that can be visualized in Foxglove or RViz:

- `/rrt_drone_path`
- `/spline_drone_path`
- `/obstacles`
- `/rrt_tree`
- `/start_goal`
- `/drone_marker`
- `/normal_ray`
- `/camera/image_raw`
- `/camera/camera_info`

These topics allow you to inspect:

- the final drone path,
- obstacle regions,
- sampling tree growth,
- start/goal markers,
- and the drone pose in motion.

## Example Usage

### Standalone Python examples

```bash
python3 path_optimization.py
python3 rrt_star_path_optimization.py
```

These scripts are useful for quickly understanding the optimization logic without running the ROS 2 stack.

## Notes

- The package metadata in `package.xml` and `setup.py` still contains placeholder values such as `TODO: Package description` and `TODO: License declaration`.
- The project currently contains multiple variations of the same planner/smoother, which can be useful for research and comparison but may need cleanup for production use.
- Some scripts appear to be experimental or transitional and may require minor adjustments depending on your environment.

## Future Improvements

Potential improvements include:

- unifying the planner implementations into a single clean API,
- adding configuration files for start/goal/obstacle parameters,
- improving package metadata and licensing,
- adding unit tests for the path optimization logic,
- refining visualization topics and message structure.

## Contributing

Contributions are welcome. If you want to improve the project, consider:

- cleaning up redundant planner variants,
- adding more robust obstacle handling,
- improving documentation,
- and expanding visualization support.
