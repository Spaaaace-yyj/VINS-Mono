# VINS-Mono
## A Robust and Versatile Monocular Visual-Inertial State Estimator

**11 Jan 2019**: An extension of **VINS**, which supports stereo cameras / stereo cameras + IMU / mono camera + IMU, is published at [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

**29 Dec 2017**: New features: Add map merge, pose graph reuse, online temporal calibration function, and support rolling shutter camera. Map reuse videos: 

<a href="https://www.youtube.com/embed/WDpH80nfZes" target="_blank"><img src="http://img.youtube.com/vi/WDpH80nfZes/0.jpg" 
alt="cla" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/eINyJHB34uU" target="_blank"><img src="http://img.youtube.com/vi/eINyJHB34uU/0.jpg" 
alt="icra" width="240" height="180" border="10" /></a>

VINS-Mono is a real-time SLAM framework for **Monocular Visual-Inertial Systems**. It uses an optimization-based sliding window formulation for providing high-accuracy visual-inertial odometry. It features efficient IMU pre-integration with bias correction, automatic estimator initialization, online extrinsic calibration, failure detection and recovery, loop detection, and global pose graph optimization, map merge, pose graph reuse, online temporal calibration, rolling shutter support. VINS-Mono is primarily designed for state estimation and feedback control of autonomous drones, but it is also capable of providing accurate localization for AR applications. This code runs on **Linux**, and is fully integrated with **ROS**. For **iOS** mobile implementation, please go to [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile).

**Authors:** [Tong Qin](http://www.qintonguav.com), [Peiliang Li](https://github.com/PeiliangLi), [Zhenfei Yang](https://github.com/dvorak0), and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [HKUST Aerial Robotics Group](http://uav.ust.hk/)

**Videos:**

<a href="https://www.youtube.com/embed/mv_9snb_bKs" target="_blank"><img src="http://img.youtube.com/vi/mv_9snb_bKs/0.jpg" 
alt="euroc" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/g_wN0Nt0VAU" target="_blank"><img src="http://img.youtube.com/vi/g_wN0Nt0VAU/0.jpg" 
alt="indoor_outdoor" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/I4txdvGhT6I" target="_blank"><img src="http://img.youtube.com/vi/I4txdvGhT6I/0.jpg" 
alt="AR_demo" width="240" height="180" border="10" /></a>

EuRoC dataset;                  Indoor and outdoor performance;                         AR application;

<a href="https://www.youtube.com/embed/2zE84HqT0es" target="_blank"><img src="http://img.youtube.com/vi/2zE84HqT0es/0.jpg" 
alt="MAV platform" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/CI01qbPWlYY" target="_blank"><img src="http://img.youtube.com/vi/CI01qbPWlYY/0.jpg" 
alt="Mobile platform" width="240" height="180" border="10" /></a>

 MAV application;               Mobile implementation (Video link for mainland China friends: [Video1](http://www.bilibili.com/video/av10813254/) [Video2](http://www.bilibili.com/video/av10813205/) [Video3](http://www.bilibili.com/video/av10813089/) [Video4](http://www.bilibili.com/video/av10813325/) [Video5](http://www.bilibili.com/video/av10813030/))

**Related Papers**

* **Online Temporal Calibration for Monocular Visual-Inertial Systems**, Tong Qin, Shaojie Shen, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2018), **best student paper award** [pdf](https://ieeexplore.ieee.org/abstract/document/8593603)

* **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen, IEEE Transactions on Robotics[pdf](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert) 

*If you use VINS-Mono for your academic research, please cite at least one of our related papers.*[bib](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/support_files/paper_bib.txt)

## 1. Prerequisites (ROS2 Humble)

1.1 **Ubuntu** and **ROS2**
This branch has been ported to **ROS2 Humble** on **Ubuntu 22.04 (Jammy)**. Install ROS2 Humble following the [official guide](https://docs.ros.org/en/humble/Installation.html). All nodes are written in ROS2 object-oriented style (`rclcpp::Node` subclasses); the SLAM/VIO algorithms are unchanged.

Additional ROS2 packages (normally included with `ros-humble-desktop`):
```
    sudo apt-get install ros-humble-cv-bridge ros-humble-tf2 ros-humble-tf2-ros \
                         ros-humble-visualization-msgs ros-humble-rviz2
```

1.2. **Ceres Solver**
```
    sudo apt-get install libceres-dev
```
The code compiles against **Ceres 2.0.0** (Ubuntu 22.04 default). Deprecation warnings about `LocalParameterization` are harmless.

1.3. **OpenCV / Eigen**
```
    sudo apt-get install libopencv-dev libeigen3-dev
```

## 2. Build VINS-Mono with colcon
```
    mkdir -p ~/vins_ws/src
    cd ~/vins_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
    cd ~/vins_ws
    colcon build --symlink-install
    source install/setup.bash
```
The workspace contains 7 packages: `camera_model`, `feature_tracker`, `vins_estimator`, `pose_graph`, `benchmark_publisher`, `data_generator`, `ar_demo`.

## 3. Visual-Inertial Odometry and Pose Graph Reuse on Public Datasets
Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Although it contains stereo cameras, we only use one camera. The system also works with the [ETH-asl cla dataset](http://robotics.ethz.ch/~asl-datasets/maplab/multi_session_mapping_CLA/bags/). We take EuRoC as the example.

**3.1 Visual-inertial odometry and loop closure**

Open two terminals: one launches the full pipeline (feature tracker + estimator + pose graph + RViz2), the other plays the bag.
```
    ros2 launch vins_estimator vins.launch.py
    ros2 bag play YOUR_PATH_TO_DATASET/MH_01_easy
```
Notes:
- `vins.launch.py` defaults to the EuRoC config. Use another config with:
  `ros2 launch vins_estimator vins.launch.py config_file:=/abs/path/to/config.yaml`
- Add `use_rviz:=false` to skip RViz2.
- `image_topic` and `imu_topic` are read from the config file (`config/euroc/euroc_config.yaml`).

**3.2 (Optional) Visualize ground truth**

The `benchmark_publisher` visualizes ground truth (naive alignment, for visualization only, not for quantitative comparison):
```
    ros2 run benchmark_publisher benchmark_publisher --ros-args -p data_name:=/abs/path/ground_truth.csv
```
(Green line = VINS result, red line = ground truth.)

**3.3 (Optional) Without extrinsic parameters**

Set `estimate_extrinsic: 2` in the config file and rotate the device for a few seconds at startup; camera-IMU extrinsic is calibrated online.

**3.4 Map merge**

After playing MH_01, continue playing MH_02, MH_03 ... The system merges them by loop closure.

**3.5 Map reuse**

- **Save**: set `pose_graph_save_path` in the config file, play a bag, then press `s` + `enter` in the `pose_graph` terminal. The current pose graph is saved.
- **Load**: set `load_previous_pose_graph: 1` before running. New sequences are aligned to the previous pose graph.

## 4. AR Demo
4.1 Download the [bag file](https://www.dropbox.com/s/s29oygyhwmllw9k/ar_box.bag?dl=0), which is collected from HKUST Robotic Institute. For friends in mainland China, download from [bag file](https://pan.baidu.com/s/1geEyHNl).

4.2 Run the VINS pipeline (using the `3dm` MEI-camera config), the AR node, then play the bag.
```
    ros2 launch vins_estimator vins.launch.py config_file:=$VINS_WS/install/vins_estimator/share/vins_estimator/config/3dm/3dm_config.yaml use_rviz:=false
    ros2 launch ar_demo ar_demo.launch.py config_file:=$VINS_WS/install/vins_estimator/share/vins_estimator/config/3dm/3dm_config.yaml image_topic:=/mv_25001498/image_raw
    ros2 bag play YOUR_PATH_TO_DATASET/ar_box
```
We put one 0.8m x 0.8m x 0.8m virtual box in front of your view.

## 5. Run with your device 

Suppose you are familiar with ROS and you can get a camera and an IMU with raw metric measurements in ROS topic, you can follow these steps to set up your device. For beginners, we highly recommend you to first try out [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile) if you have iOS devices since you don't need to set up anything.

5.1 Change to your topic name in the config file. The image should exceed 20Hz and IMU should exceed 100Hz. Both image and IMU should have the accurate time stamp. IMU should contain absolute acceleration values including gravity.

5.2 Camera calibration:

We support the [pinhole model](http://docs.opencv.org/2.4.8/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) and the [MEI model](http://www.robots.ox.ac.uk/~cmei/articles/single_viewpoint_calib_mei_07.pdf). You can calibrate your camera with any tools you like. Just write the parameters in the config file in the right format. If you use rolling shutter camera, please carefully calibrate your camera, making sure the reprojection error is less than 0.5 pixel.

5.3 **Camera-Imu extrinsic parameters**:

If you have seen the config files for EuRoC and AR demos, you can find that we can estimate and refine them online. If you familiar with transformation, you can figure out the rotation and position by your eyes or via hand measurements. Then write these values into config as the initial guess. Our estimator will refine extrinsic parameters online. If you don't know anything about the camera-IMU transformation, just ignore the extrinsic parameters and set the **estimate_extrinsic** to **2**, and rotate your device set at the beginning for a few seconds. When the system works successfully, we will save the calibration result. you can use these result as initial values for next time. An example of how to set the extrinsic parameters is in[extrinsic_parameter_example](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/config/extrinsic_parameter_example.pdf)

5.4 **Temporal calibration**:
Most self-made visual-inertial sensor sets are unsynchronized. You can set **estimate_td** to 1 to online estimate the time offset between your camera and IMU.  

5.5 **Rolling shutter**:
For rolling shutter camera (carefully calibrated, reprojection error under 0.5 pixel), set **rolling_shutter** to 1. Also, you should set rolling shutter readout time **rolling_shutter_tr**, which is from sensor datasheet(usually 0-0.05s, not exposure time). Don't try web camera, the web camera is so awful.

5.6 Other parameter settings: Details are included in the config file.

5.7 Performance on different devices: 

(global shutter camera + synchronized high-end IMU, e.g. VI-Sensor) > (global shutter camera + synchronized low-end IMU) > (global camera + unsync high frequency IMU) > (global camera + unsync low frequency IMU) > (rolling camera + unsync low frequency IMU). 

## 6. Node / Topic / Parameter Interface

### 6.1 Nodes

| Node | Package | Executable | Description |
|------|---------|------------|-------------|
| `feature_tracker` | feature_tracker | feature_tracker | KLT optical-flow frontend |
| `vins_estimator` | vins_estimator | vins_estimator | Sliding-window VIO backend |
| `pose_graph` | pose_graph | pose_graph | Loop closure + 4-DoF pose graph |
| `ar_demo` | ar_demo | ar_demo_node | Augmented-reality object rendering |
| `benchmark_publisher` | benchmark_publisher | benchmark_publisher | Ground-truth publisher (visualization only) |
| `data_generator` | data_generator | data_generator | Synthetic IMU/camera data generator |
| `Calibration` | camera_model | Calibration | Camera intrinsic calibration tool |

### 6.2 Topics

**feature_tracker**
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `image_topic` (from config) | sensor_msgs/msg/Image | sub | Raw camera image |
| `/feature_tracker/feature` | sensor_msgs/msg/PointCloud2 | pub | Tracked features (8 float fields: x, y, z, id, u, v, velocity_x, velocity_y) |
| `/feature_tracker/feature_img` | sensor_msgs/msg/Image | pub | Debug image with feature markers |
| `/feature_tracker/restart` | std_msgs/msg/Bool | pub | Frontend reset signal |

**vins_estimator**
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `imu_topic` (from config) | sensor_msgs/msg/Imu | sub | Raw IMU |
| `/feature_tracker/feature` | sensor_msgs/msg/PointCloud2 | sub | Features |
| `/feature_tracker/restart` | std_msgs/msg/Bool | sub | Reset |
| `/pose_graph/match_points` | sensor_msgs/msg/PointCloud2 | sub | Loop-closure match points (11 floats/point) |
| `/vins_estimator/odometry` | nav_msgs/msg/Odometry | pub | VIO odometry (consumed by pose_graph) |
| `/vins_estimator/imu_propagate` | nav_msgs/msg/Odometry | pub | IMU-propagated odometry |
| `/vins_estimator/path` | nav_msgs/msg/Path | pub | Optimized trajectory |
| `/vins_estimator/relocalization_path` | nav_msgs/msg/Path | pub | Relocalized path |
| `/vins_estimator/point_cloud` | sensor_msgs/msg/PointCloud2 | pub | 3D map points |
| `/vins_estimator/history_cloud` | sensor_msgs/msg/PointCloud2 | pub | Marginalized point cloud |
| `/vins_estimator/key_poses` | visualization_msgs/msg/Marker | pub | Keyframe poses (4-DoF graph) |
| `/vins_estimator/camera_pose` | nav_msgs/msg/Odometry | pub | Current camera pose |
| `/vins_estimator/camera_pose_visual` | visualization_msgs/msg/MarkerArray | pub | Camera pose markers |
| `/vins_estimator/keyframe_pose` | nav_msgs/msg/Odometry | pub | Keyframe pose |
| `/vins_estimator/keyframe_point` | sensor_msgs/msg/PointCloud2 | pub | Keyframe features (8 fields) |
| `/vins_estimator/extrinsic` | nav_msgs/msg/Odometry | pub | Estimated camera-IMU extrinsic |
| `/vins_estimator/relo_relative_pose` | nav_msgs/msg/Odometry | pub | Relocalization relative pose |
| `/tf` (world→body) | tf2_msgs/msg/TFMessage | pub | Body transform |

**pose_graph**
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/vins_estimator/imu_propagate` | nav_msgs/msg/Odometry | sub | IMU odometry |
| `/vins_estimator/odometry` | nav_msgs/msg/Odometry | sub | VIO odometry |
| `image_topic` (from config) | sensor_msgs/msg/Image | sub | Keyframe image for loop detection |
| `/vins_estimator/keyframe_pose` | nav_msgs/msg/Odometry | sub | Keyframe pose |
| `/vins_estimator/extrinsic` | nav_msgs/msg/Odometry | sub | Extrinsic |
| `/vins_estimator/keyframe_point` | sensor_msgs/msg/PointCloud2 | sub | Keyframe points |
| `/vins_estimator/relo_relative_pose` | nav_msgs/msg/Odometry | sub | Relo relative pose |
| `/pose_graph/match_image` | sensor_msgs/msg/Image | pub | Loop-match visualization |
| `/pose_graph/match_points` | sensor_msgs/msg/PointCloud2 | pub | Loop-match points (11 floats/point) |
| `/pose_graph/camera_pose_visual` | visualization_msgs/msg/MarkerArray | pub | Camera pose markers |
| `/pose_graph/key_odometrys` | visualization_msgs/msg/Marker | pub | Keyframe odometry markers |
| `/pose_graph/no_loop_path` | nav_msgs/msg/Path | pub | Path before pose-graph optimization |

### 6.3 Parameters

| Node | Parameter | Type | Default | Description |
|------|-----------|------|---------|-------------|
| feature_tracker | `config_file` | string | — | Path to VINS config YAML |
| feature_tracker | `vins_folder` | string | — | Root folder used to locate `config/fisheye_mask.jpg` |
| vins_estimator | `config_file` | string | — | Path to VINS config YAML |
| pose_graph | `config_file` | string | — | Path to VINS config YAML |
| pose_graph | `visualization_shift_x` | double | 0.0 | x offset of pose-graph visualization |
| pose_graph | `visualization_shift_y` | double | 0.0 | y offset of pose-graph visualization |
| pose_graph | `skip_cnt` | int | 0 | Keyframe skip count |
| pose_graph | `skip_dis` | double | 0.0 | Keyframe skip distance (m) |
| ar_demo | `calib_file` | string | — | Camera calibration YAML (VINS config) |
| ar_demo | `use_undistored_img` | bool | false | Subscribe to `image_undistored` instead of `image_raw` |
| benchmark_publisher | `data_name` | string | — | Path to ground-truth CSV |

### 6.4 Launch files

| Launch file | Package | Description |
|-------------|---------|-------------|
| `vins.launch.py` | vins_estimator | Full pipeline (feature_tracker + vins_estimator + pose_graph + rviz2). Args: `config_file`, `vins_folder`, `use_rviz` |
| `ar_demo.launch.py` | ar_demo | AR rendering node. Args: `config_file`, `image_topic` |

## 7. Docker Support

To further facilitate the building process, we add docker in our code. Docker environment is like a sandbox, thus makes our code environment-independent. To run with docker, first make sure [ros](http://wiki.ros.org/ROS/Installation) and [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) are installed on your machine. Then add your account to `docker` group by `sudo usermod -aG docker $YOUR_USER_NAME`. **Relaunch the terminal or logout and re-login if you get `Permission denied` error**, type:
```
cd ~/catkin_ws/src/VINS-Mono/docker
make build
./run.sh LAUNCH_FILE_NAME   # ./run.sh euroc.launch
```
Note that the docker building process may take a while depends on your network and machine. After VINS-Mono successfully started, open another terminal and play your bag file, then you should be able to see the result. If you need modify the code, simply run `./run.sh LAUNCH_FILE_NAME` after your changes.


## 8. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, and a generic [camera model](https://github.com/hengli/camodocal).

## 9. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong QIN <tong.qinATconnect.ust.hk> or Peiliang LI <pliapATconnect.ust.hk>.

For commercial inquiries, please contact Shaojie SHEN <eeshaojieATust.hk>
