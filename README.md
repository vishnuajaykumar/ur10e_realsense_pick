# UR10e + RealSense Vision-Guided Pick & Place

ROS2 Foxy | C++ | Colcon | Open Source

A fully open-source pipeline for autonomous object detection and pick-and-place with a **UR10e** robot arm and **Intel RealSense D4xx** depth camera.

---

## Pipeline

```
RealSense D4xx
    │
    ├─ /camera/color/image_raw          ──► YOLOv8-seg node (Python)
    │                                            │
    │                                            └─ /segmentation/masks
    │                                            └─ /segmentation/detections
    │
    ├─ /camera/aligned_depth_to_color/image_raw ─┐
    └─ /camera/color/camera_info ────────────────┤
                                                 ▼
                                    PointCloud Processor (C++)
                                    [mask → backproject → PCL filter → PCA pose]
                                                 │
                                                 └─ /perception/grasp_pose
                                                 └─ /perception/object_cloud
                                                          │
                                                          ▼
                                            MoveIt2 Pick & Place (C++)
                                            [pre-grasp → approach → grasp
                                             → retreat → place → home]
```

---

## Packages

| Package | Lang | Role |
|---|---|---|
| `realsense_bringup` | CMake/Launch | Start RealSense camera node |
| `segmentation` | Python | YOLOv8-seg inference → instance masks |
| `perception` | C++ | Masked point cloud → 3D grasp pose |
| `manipulation` | C++ | MoveIt2 pick-and-place action server |
| `bringup` | CMake/Launch | Full system launch |

---

## Dependencies

### System
```bash
# ROS2 Foxy (Ubuntu 20.04)
sudo apt install ros-foxy-cv-bridge ros-foxy-vision-msgs \
                 ros-foxy-pcl-ros ros-foxy-pcl-conversions \
                 ros-foxy-image-transport ros-foxy-tf2-ros \
                 ros-foxy-tf2-geometry-msgs ros-foxy-message-filters
```

### RealSense ROS2 wrapper
```bash
sudo apt install ros-foxy-realsense2-camera
```

### YOLOv8 (Python)
```bash
pip install ultralytics
```

### UR Robot Driver + MoveIt2
```bash
sudo apt install ros-foxy-ur-robot-driver ros-foxy-moveit
```

---

## Build

```bash
cd ~/ur10e_realsense_pick
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Run

```bash
# Full pipeline
ros2 launch bringup pick_and_place.launch.py

# With custom model and target class (COCO class 39 = bottle)
ros2 launch bringup pick_and_place.launch.py \
  model_path:=/path/to/yolov8n-seg.pt \
  target_class:=39 \
  device:=cuda:0
```

### Individual nodes
```bash
# Camera only
ros2 launch realsense_bringup realsense.launch.py

# Segmentation only (for testing with a rosbag)
ros2 launch segmentation segmentation.launch.py model_path:=yolov8n-seg.pt

# Perception only
ros2 launch perception perception.launch.py
```

---

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB stream |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth aligned to RGB |
| `/segmentation/masks` | `sensor_msgs/Image` | Instance mask (pixel = instance id) |
| `/segmentation/debug_image` | `sensor_msgs/Image` | RGB with mask overlay |
| `/perception/grasp_pose` | `geometry_msgs/PoseStamped` | 3D grasp pose in base_link |
| `/perception/object_cloud` | `sensor_msgs/PointCloud2` | Masked object point cloud |

---

## Next Steps

- [ ] Add UR10e URDF + MoveIt config (srdf, kinematics.yaml)
- [ ] Camera-to-robot hand-eye calibration
- [ ] Grasp pose refinement (approach direction from surface normals)
- [ ] Multi-object selection logic (closest / highest confidence)
- [ ] Rosbag recording for offline development
