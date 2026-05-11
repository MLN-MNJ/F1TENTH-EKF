# sensors_bringup

ROS 2 package that starts the F1tenth car's full sensor and odometry stack: RealSense camera, IMU calibration, LiDAR-based ICP odometry, and an Extended Kalman Filter (EKF) that fuses everything into a single `/odom_ekf` output used by the MPC controller and RTAB-Map.

---

## What gets launched

```
RealSense D435i
  └─ /camera/camera/imu  ──► imu_covariance_override ──► /imu/data ──┐
                                                                        ├──► icp_odometry ──► /odom_icp ──► ekf_filter_node ──► /odom_ekf
/scan (LiDAR, external) ─────────────────────────────────────────────┘                                          ▲
/odom (VESC, external)  ──────────────────────────────────────────────────────────────────────────────────────┘
```

| Topic | Description |
|---|---|
| `/camera/camera/imu` | Raw IMU from RealSense |
| `/imu/data` | Bias-corrected IMU, aligned to `base_link` |
| `/odom_icp` | LiDAR odometry (ICP, Frame-to-Map) |
| `/odom` | Wheel odometry from VESC (bootstrap only) |
| `/odom_ekf` | **Final fused odometry — use this** |

---

## Prerequisites

Install these ROS 2 packages before building:

```bash
sudo apt install ros-$ROS_DISTRO-realsense2-camera \
                 ros-$ROS_DISTRO-robot-localization \
                 ros-$ROS_DISTRO-rtabmap-ros
```

Your workspace must also have a LiDAR driver publishing `/scan` (e.g. `urg_node` for the Hokuyo) and the VESC driver publishing `/odom`. Those are **not** started by this package.

---

## Build

```bash
cd ~/f1tenth_ws
colcon build --packages-select sensors_bringup
source install/setup.bash
```

---

## Launch

```bash
ros2 launch sensors_bringup sensors_bringup.launch.py
```

### Startup sequence

1. **RealSense camera** initialises (depth 424×240 @ 30 FPS, infra1/2, IMU).
2. **`imu_covariance_override`** starts and prints:
   ```
   Collecting 200 samples for gyro bias calibration...
   ```
3. **Keep the car completely still** for ~5 seconds while it collects samples.
4. Calibration finishes and prints the measured bias offsets:
   ```
   Gyro bias: x=... y=... z=...
   ```
5. **ICP odometry** begins processing `/scan` + `/imu/data`.
6. **EKF** starts fusing sources and publishing `/odom_ekf` at 45 Hz.

> **Important:** If the car moves during step 3 the gyro bias will be wrong and odometry will drift. Always wait for the calibration message before driving.

---

## Verifying it works

```bash
# All topics should be listed
ros2 topic list | grep -E "imu|odom|scan"

# Check the final fused odometry is flowing
ros2 topic echo /odom_ekf --once

# Quick sanity check on rates
ros2 topic hz /odom_ekf   # expect ~45 Hz
ros2 topic hz /odom_icp   # expect ~10–30 Hz depending on LiDAR
```

If `/odom_ekf` is not publishing, check:
- Is `/scan` publishing? The ICP node needs it to start outputting `/odom_icp`.
- Is `/odom` publishing? The EKF needs at least one odometry source to initialise.
- Did the RealSense fail to open? Run `ros2 topic hz /camera/camera/imu` to confirm.

---

## Static transforms

Two fixed transforms are published and must match your physical mounting:

| Transform | Translation (x, y, z) | Meaning |
|---|---|---|
| `base_link → laser` | 0.27 m, 0.00 m, 0.11 m | LiDAR position on chassis |
| `base_link → camera_link` | 0.10 m, 0.00 m, 0.10 m | RealSense position on chassis |

To adjust for your build, edit the `static_transform_publisher` arguments in [`launch/sensors_bringup.launch.py`](launch/sensors_bringup.launch.py).

---

## Tuning

### IMU (`src/imu_covariance_override.py`)

The node remaps the camera's IMU axes to align with `base_link`:

```
(gx, gy, gz) → (gz, -gx, -gy)
```

If your RealSense is mounted in a non-standard orientation you will need to change this rotation. The angular velocity covariance is currently set to `0.001` on the diagonal — increase it if the EKF is over-trusting the gyro.

### EKF (`config/ekf.yaml`)

Key parameters:

| Parameter | Value | Notes |
|---|---|---|
| `frequency` | 45 Hz | Output rate; tune to match your control loop |
| `two_d_mode` | true | Ignores z, roll, pitch — correct for flat floors |
| odom0 (`/odom_icp`) | x, y, yaw, vx | Primary position source |
| odom1 (`/odom`) | vx only | VESC velocity for startup bootstrap |
| imu0 (`/imu/data`) | vyaw, ax | Rotation rate + forward acceleration |

### ICP odometry (`launch/sensors_bringup.launch.py`)

Relevant parameters for racing speeds:

| Parameter | Value | Notes |
|---|---|---|
| `Icp/MaxTranslation` | 1.0 m | Max motion between frames (covers 5 m/s at 5 Hz) |
| `Icp/MaxRotation` | 1.57 rad | Max rotation between frames (90°) |
| `Icp/VoxelSize` | 0.05 m | Point cloud downsampling |
| `OdomF2M/MaxSize` | 2000 | Local map size in points |
| `Odom/ResetCountdown` | 1 | Resets immediately on failure — important for recovery |

---

## File structure

```
sensors_bringup/
├── config/
│   └── ekf.yaml                  # EKF sensor fusion config
├── launch/
│   └── sensors_bringup.launch.py # Main launch file
├── src/
│   └── imu_covariance_override.py # IMU calibration + axis remapping node
├── CMakeLists.txt
└── package.xml
```
