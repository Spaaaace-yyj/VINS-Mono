# Copilot Instructions for VINS-Mono

VINS-Mono is a ROS-based monocular visual-inertial SLAM system (sliding-window VIO + loop closure + pose graph). C++11, catkin build system.

## Build / Test / Lint

This is a ROS catkin workspace package. Build from a catkin workspace, not the repo root directly:

```bash
cd ~/catkin_ws/src && git clone <this-repo> .  # or symlink
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```

- Build a single package: `catkin_make --pkg vins_estimator` (also `pose_graph`, `feature_tracker`, `camera_model`, `ar_demo`, `benchmark_publisher`, `data_generator`)
- **Ceres Solver must be 1.14.x** — compilation breaks on Ceres 2.0+. Do not bump it.
- There are **no unit tests and no lint config**. Verification is manual: launch a node, play a rosbag, inspect RViz and console output.
- Docker build alternative: `cd docker && make build && ./run.sh euroc.launch`

## High-Level Architecture

The system is three cooperating ROS nodes plus a camera model library:

1. **`feature_tracker`** (`feature_tracker_node.cpp`) — KLT optical-flow frontend. Tracks features frame-to-frame and publishes them as `sensor_msgs::PointCloud` on `/feature_tracker/feature` (plus `/feature_tracker/feature_img` for visualization and `/feature_tracker/restart`).

2. **`vins_estimator`** (`estimator_node.cpp`, `Estimator` in `estimator.{h,cpp}`) — the core VIO backend. Runs a sliding-window nonlinear optimization (Ceres) with IMU pre-integration. Key phases: estimator initialization (`initial/`, using 5-point + SFM + visual-inertial alignment), then `solveOdometry()`/`optimization()` with marginalization (`factor/marginalization_factor.cpp`). The node runs two loops: a high-frequency `predict()` (IMU forward propagation) and the `process()` thread that consumes synchronized IMU+feature pairs.

3. **`pose_graph`** (`pose_graph_node.cpp`, `pose_graph.{h,cpp}`, `keyframe.{h,cpp}`) — loop closure and 4-DoF global pose graph optimization. Uses a vendored DBoW2 (`pose_graph/src/ThirdParty/`) for place recognition and publishes loop-closure matches to `/pose_graph/match_points`, which the estimator consumes for relocalization.

Auxiliary packages: `camera_model` (camodocal-derived camera model library used by the pose graph), `ar_demo` (AR overlay demo), `benchmark_publisher` (ground-truth visualization), `data_generator` (synthetic data).

Data flow: camera + IMU topics → `feature_tracker` → `/feature_tracker/feature` → `vins_estimator` → `/vins_estimator/odometry`, `/vins_estimator/keyframe_pose`, `/vins_estimator/keyframe_point` → `pose_graph` → `/pose_graph/match_points` → back into `vins_estimator` for relocalization.

## Key Conventions

- **Config drives everything.** Each `config/<dataset>/*.yaml` is loaded by a matching launch file in `vins_estimator/launch/`. Parameters are read via `readParameters(ros::NodeHandle&)` into global `extern` variables declared in `vins_estimator/src/parameters.h` and `pose_graph/src/parameters.h`. Adding a tunable almost always means: add it to the YAML, declare an `extern`, read it in `parameters.cpp`.
- **Hardcoded constants live in `vins_estimator/src/parameters.h`** — `WINDOW_SIZE` (10), `NUM_OF_F` (1000), `NUM_OF_CAM` (1), `FOCAL_LENGTH` (460). The code is effectively mono-only despite the `NUM_OF_CAM` indirection.
- **Inter-node feature encoding.** Features are passed as a `sensor_msgs::PointCloud` where `z == 1` and the channels carry `feature_id*NUM_OF_CAM + camera_id`, `p_u`, `p_v`, velocity_x, velocity_y. See `feature_callback`/`process()` in `estimator_node.cpp` for the decode side.
- **Eigen aliases** `Vector3d`, `Matrix3d`, `Quaterniond` are defined in `vins_estimator/src/utility/utility.h`; `TicToc` (timing) lives in `utility/tic_toc.h`.
- **Naming style** mirrors the upstream HKUST code: `snake_case` files, camelCase methods, globals are SCREAMING_SNAKE. Keep style consistent rather than modernizing.
- **ROS topic names are private-relative** — nodes use `ros::NodeHandle("~")` for params, so topics are resolved under the node namespace (`/vins_estimator/...`, `/feature_tracker/...`, `/pose_graph/...`).
- **No `using namespace std`** at file scope in the core; `std::` is usually explicit, but `Eigen` types are often unqualified via the aliases above.
