# Sensors Bringup — F1TENTH RTAB-Map SLAM Pipeline

## Hardware

| Component | Model | Interface |
|---|---|---|
| Compute | NVIDIA Jetson Orin Nano Super (JetPack) | — |
| Stereo + Depth Camera | Intel RealSense D435i | USB 3.2 |
| 2D LiDAR | SICK TiM | Ethernet |
| Motor Controller | VESC | USB |

## Sensor Topics

### RealSense D435i

| Topic | Type | Used For |
|---|---|---|
| `/camera/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | Visual features for loop closure |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Depth data for RTAB-Map |
| `/camera/camera/infra1/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/camera/camera/imu` | `sensor_msgs/Imu` | Raw accel + gyro (fused by Madgwick filter) |

### SICK LiDAR

| Topic | Type | QoS | Used For |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | RELIABLE + TRANSIENT_LOCAL | ICP scan matching, occupancy grid |

### VESC Wheel Odometry

| Topic | Type | Used For |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Primary odometry (odom → base_link TF) |

## Architecture

```
VESC wheel odom ──► /odom ──────────────────────┐
                                                 │
RealSense D435i ──► infra1 + depth + imu ───┐    │
                                             ├──► RTAB-Map SLAM ──► map→odom correction
SICK LiDAR ──────► /scan ───────────────────┘    │       │
                                                 │       ├──► /localization_pose
Madgwick filter ──► /camera/camera/imu/filtered ─┘       ├──► /map (occupancy grid)
                                                         └──► /cloud_map (3D point cloud)
```

RTAB-Map receives wheel odometry, depth images, IR images, filtered IMU, and LiDAR scans. It performs:

1. **Neighbor link refining** — ICP scan matching between consecutive nodes to correct heading errors from wheel odom
2. **Proximity detection** — ICP matching against nearby stored scans when revisiting areas
3. **Visual loop closure** — feature matching on IR images to recognize previously visited locations
4. **Graph optimization** — globally adjusts all node poses after adding ICP/visual constraints

The corrected pose is published on `/localization_pose` in the `map` frame.

## TF Tree

```
map → odom → base_link → camera_link → camera_infra1_optical_frame
                                      → camera_depth_optical_frame
                                      → camera_accel_optical_frame
                                      → camera_gyro_optical_frame
                       → laser
```

- `odom → base_link`: published by VESC odom node
- `map → odom`: published by RTAB-Map (drift correction)
- `base_link → camera_link`: static transform (0.1, 0, 0.1)
- `base_link → laser`: static transform (0.27, 0, 0.1)
- `camera_link → *_optical_frame`: published by RealSense driver

## Known Issues & Fixes

### Jetson + RealSense D435i

**Problem:** The default `librealsense` packages from Intel do not support the Jetson's Tegra USB kernel. The RealSense node starts but IMU data is unavailable, and the camera may fail to enumerate.

**Fix:** Build `librealsense` from source with `FORCE_RSUSB_BACKEND=ON`:

```bash
cmake .. \
  -DFORCE_RSUSB_BACKEND=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false
```

Also update the D435i firmware to 5.17.x via `rs-fw-update`.

**Problem:** RealSense node exits cleanly with no output when `LD_LIBRARY_PATH` doesn't include the Humble library path on aarch64.

**Fix:** Prefix the launch command:

```bash
LD_LIBRARY_PATH=/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH \
ros2 launch realsense2_camera rs_launch.py ...
```

### RealSense IMU

**Problem:** RTAB-Map rejects IMU data with `IMU received doesn't have orientation set, it is ignored.`

**Cause:** The D435i publishes raw accel + gyro without orientation. RTAB-Map requires orientation in the IMU message.

**Fix:** Run `imu_filter_madgwick` to fuse accel + gyro into a full IMU message with orientation:

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args -p use_mag:=false -p publish_tf:=false \
  -r imu/data_raw:=/camera/camera/imu \
  -r imu/data:=/camera/camera/imu/filtered
```

### RealSense Double Namespace

**Problem:** Topics appear as `/camera/camera/infra1/...` instead of `/camera/infra1/...`.

**Cause:** The RealSense launch uses both a namespace and a `camera_name` parameter that both default to `camera`.

**Impact:** All topic remappings must use the double prefix `/camera/camera/`.

### QoS Mismatches

**Problem:** RTAB-Map doesn't receive data from sensors despite topics being published.

**Cause:** RealSense publishes with `SENSOR_DATA` QoS (best effort), SICK publishes with `RELIABLE` + `TRANSIENT_LOCAL`.

**Fix:** Set QoS per-topic in RTAB-Map:

```python
'qos_image': 1,        # SENSOR_DATA for RealSense
'qos_camera_info': 1,  # SENSOR_DATA for RealSense
'qos_scan': 0,         # RELIABLE for SICK
```

### RTAB-Map Stereo + Scan Sync

**Problem:** When using `subscribe_stereo=true` with `subscribe_scan=true`, the scan topic is never subscribed by RTAB-Map. The subscription printout shows only the 4 stereo topics.

**Cause:** In `rtabmap_ros` 0.22.1 for Humble, the stereo callback does not include scan in its approximate time sync filter. This also applies when `subscribe_odom_info=true`.

**Fix:** Use `subscribe_depth=true` instead of `subscribe_stereo=true`. Feed the left IR image as `rgb/image` and the actual depth image as `depth/image`:

```python
'subscribe_depth': True,
'subscribe_stereo': False,
'subscribe_scan': True,
```

With remappings:

```python
('rgb/image', '/camera/camera/infra1/image_rect_raw'),
('depth/image', '/camera/camera/depth/image_rect_raw'),
('rgb/camera_info', '/camera/camera/infra1/camera_info'),
('scan', '/scan'),
```

### Wheel Odometry Heading Drift

**Problem:** 90° turns appear as ~60-70° in the map. The occupancy grid has skewed corners.

**Cause:** VESC wheel odometry computes yaw from steering angle and velocity, which is inaccurate at speed.

**Mitigation:** RTAB-Map's ICP scan matching (`RGBD/NeighborLinkRefining`) corrects heading between consecutive nodes. Key parameters:

```python
'Reg/Strategy': '2',           # visual + ICP
'Reg/Force3DoF': 'true',       # constrain to 2D
'RGBD/AngularUpdate': '0.05',  # node every ~3°
'RGBD/LinearUpdate': '0.1',    # node every 10cm
'Rtabmap/DetectionRate': '5.0', # 5 Hz processing
```

## Launch Commands

### RealSense Camera

```bash
LD_LIBRARY_PATH=/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH \
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  enable_color:=false \
  enable_depth:=true \
  depth_module.depth_profile:=424x240x30 \
  depth_module.infra_profile:=424x240x30 \
  enable_infra1:=true \
  enable_infra2:=false
```

### Mapping

```bash
rm ~/.ros/rtabmap.db
ros2 launch ~/f1tenth_ws/src/rtabmap_f1tenth.launch.py
```

Drive slowly around the full track. Ctrl+C when done. Map saves to `~/.ros/rtabmap.db`.

### Save Occupancy Grid

While RTAB-Map is running:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/f1tenth_ws/track_map \
  --ros-args -p map_subscribe_transient_local:=true
```

### Record Waypoints

In localization mode, drive the desired racing line:

```bash
python3 ~/f1tenth_ws/src/record_waypoints.py
```

### Localization (Racing)

Edit `rtabmap_f1tenth.launch.py`:

```python
'Mem/IncrementalMemory': 'false',
'Mem/InitWMWithAllNodes': 'true',
```

Then launch without deleting the database:

```bash
ros2 launch ~/f1tenth_ws/src/rtabmap_f1tenth.launch.py
```

Start near where mapping began. RTAB-Map publishes corrected pose on `/localization_pose`.

## Dependencies

```bash
sudo apt install \
  ros-humble-rtabmap-ros \
  ros-humble-imu-filter-madgwick \
  ros-humble-nav2-map-server
```
