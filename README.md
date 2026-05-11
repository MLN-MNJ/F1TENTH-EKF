# Sensors Bringup

## Hardware

| Component | Model | Interface |
|---|---|---|
| Compute | NVIDIA Jetson Orin Nano Super (JetPack) | — |
| Stereo + Depth Camera | Intel RealSense D435i | USB 3.2 |
| 2D LiDAR | SICK TiM | Ethernet |
| Motor Controller | VESC | USB |

## Sensor Topics

### RealSense D435i

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/camera/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | SENSOR_DATA | Left IR image (424×240 @ 30fps) |
| `/camera/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | SENSOR_DATA | Right IR image (424×240 @ 30fps) |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | SENSOR_DATA | Depth image (424×240 @ 30fps) |
| `/camera/camera/infra1/camera_info` | `sensor_msgs/CameraInfo` | SENSOR_DATA | Camera intrinsics |
| `/camera/camera/imu` | `sensor_msgs/Imu` | SENSOR_DATA | Raw accel (63 Hz) + gyro (200 Hz) |

### SICK LiDAR

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | RELIABLE + TRANSIENT_LOCAL | 2D scan, 811 points, 15 Hz |

### VESC

| Topic | Type | Description |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Wheel odometry (~50 Hz) |

### Filtered / Fused

| Topic | Type | Source | Description |
|---|---|---|---|
| `/camera/camera/imu/filtered` | `sensor_msgs/Imu` | Madgwick filter | IMU with orientation estimate |
| `/odom_ekf` | `nav_msgs/Odometry` | robot_localization EKF | Fused wheel odom + IMU |

## TF Tree

```
odom → base_link → camera_link → camera_infra1_optical_frame
                                → camera_infra2_optical_frame
                                → camera_depth_optical_frame
                                → camera_accel_optical_frame
                                → camera_gyro_optical_frame
                 → laser
```

| Transform | Published By | Type |
|---|---|---|
| `odom → base_link` | EKF node (or VESC odom) | Dynamic |
| `base_link → camera_link` | static_transform_publisher | Static (0.1, 0, 0.1 m) |
| `base_link → laser` | static_transform_publisher | Static (0.27, 0, 0.1 m) |
| `camera_link → *_optical_frame` | RealSense driver | Static |

Adjust the static transform values to match your actual sensor mount positions on the car (x forward, y left, z up).

## Launch

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
  enable_infra2:=true
```

### IMU Filter

Fuses raw accel + gyro into orientation estimate:

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -p use_mag:=false \
  -p publish_tf:=false \
  -p world_frame:=enu \
  -r imu/data_raw:=/camera/camera/imu \
  -r imu/data:=/camera/camera/imu/filtered
```

### Static Transforms

```bash
ros2 run tf2_ros static_transform_publisher 0.1 0 0.1 0 0 0 base_link camera_link
ros2 run tf2_ros static_transform_publisher 0.27 0 0.1 0 0 0 base_link laser
```

### EKF (Wheel Odom + IMU Fusion)

Fuses VESC wheel odometry (x/y velocity) with IMU (yaw orientation + yaw rate). Disable the VESC node's TF publishing to avoid conflicts with the EKF.

```python
Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter',
    parameters=[{
        'frequency': 50.0,
        'odom_frame': 'odom',
        'base_link_frame': 'base_link',
        'world_frame': 'odom',
        'publish_tf': True,

        'odom0': '/odom',
        'odom0_config': [
            False, False, False,   # x, y, z position
            False, False, False,   # roll, pitch, yaw
            True,  True,  False,   # vx, vy, vz
            False, False, False,   # vroll, vpitch, vyaw
            False, False, False,   # ax, ay, az
        ],

        'imu0': '/camera/camera/imu/filtered',
        'imu0_config': [
            False, False, False,   # x, y, z
            False, False, True,    # roll, pitch, yaw
            False, False, False,   # vx, vy, vz
            False, False, True,    # vroll, vpitch, vyaw
            False, False, False,   # ax, ay, az
        ],
        'imu0_differential': False,
        'imu0_remove_gravitational_acceleration': True,
    }],
    remappings=[
        ('odometry/filtered', '/odom_ekf'),
    ],
)
```

## Known Issues & Fixes

### Jetson + RealSense USB Backend

**Problem:** The default `librealsense` packages from Intel do not support the Jetson's Tegra USB kernel. The RealSense node may fail to enumerate or the IMU may be unavailable.

**Fix:** Build `librealsense` from source with the RSUSB backend forced on:

```bash
cmake .. \
  -DFORCE_RSUSB_BACKEND=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false
```

Update the D435i firmware to 5.17.x via `rs-fw-update`.

### LD_LIBRARY_PATH Required on Jetson

**Problem:** RealSense node exits cleanly with no output.

**Fix:** Prefix the launch command with:

```bash
LD_LIBRARY_PATH=/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
```

### RealSense Double Namespace

**Problem:** Topics appear as `/camera/camera/...` instead of `/camera/...`.

**Cause:** The RealSense launch file uses both a ROS namespace and a `camera_name` parameter that both default to `camera`.

**Impact:** All remappings must use the double prefix `/camera/camera/`.

### Raw IMU Has No Orientation

**Problem:** Downstream nodes reject IMU data with `IMU received doesn't have orientation set`.

**Cause:** The D435i publishes raw accel + gyro without computing orientation.

**Fix:** Run `imu_filter_madgwick` to fuse accel + gyro into orientation (see Launch section above).

### QoS Mismatches Between Sensors

RealSense and SICK use different QoS profiles. Any subscriber must match:

| Sensor | Reliability | Durability |
|---|---|---|
| RealSense (images, camera_info, IMU) | BEST_EFFORT | VOLATILE |
| SICK LiDAR (`/scan`) | RELIABLE | TRANSIENT_LOCAL |

Set `qos: 1` (SENSOR_DATA) for RealSense topics, `qos: 0` (RELIABLE) for SICK.

### Wheel Odometry Heading Drift

**Problem:** VESC wheel odometry computes yaw from steering angle and velocity. At higher speeds and during sharp turns, heading accumulates significant error (90° turns appear as 60-70°). At 5 m/s with aggressive cornering, wheelslip causes the raw odom to diverge completely from the actual path.

**Fix:** Fuse with IMU via the EKF node. The IMU provides accurate yaw rate and orientation that corrects the heading drift. Disable the VESC odom node's TF publishing so only the EKF publishes `odom → base_link`.

![EKF vs raw odom at 5 m/s](Screenshot 2026-04-19 194825.png)

*Visualization at 5 m/s showing drift and wheelslip. Cyan: recorded waypoints. Purple: EKF-fused odometry (`/odom_ekf`). Green: raw wheel odometry (`/odom`). The raw odom diverges wildly off-track due to heading drift and wheelslip at speed, while the EKF-fused output stays closer to the actual driven path by incorporating IMU yaw corrections.*

### USB `control_transfer` Warnings

**Problem:** Intermittent warnings in the RealSense log: `control_transfer returned error, index: 768, error: Resource temporarily unavailable`.

**Impact:** None — these are harmless USB negotiation messages on the Jetson and can be ignored.

## Dependencies

```bash
sudo apt install \
  ros-humble-realsense2-camera \
  ros-humble-imu-filter-madgwick \
  ros-humble-robot-localization
```
