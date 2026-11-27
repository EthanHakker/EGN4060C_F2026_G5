# Validation and simulation test report

Date: 2026-11-04. Environment: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz sim 8),
headless server with software rendering (no GPU; ~0.12x real time). Tests use simulation time.

## 1. CAD export integrity
* 9 STL meshes exported by SOLIDWORKS from read-only part files (binary, metres, part frame,
  no translation to positive space).
* Mesh bounding boxes match CAD part boxes within 0.2 mm; mesh volumes within 0.5 % (tessellation).
* All 10 CAD files (9 parts + assembly) verified unchanged after export: identical
  modification time and SHA-256 hash.

## 2. Static validation (`scripts/validate_urdf.py`, `check_urdf`, `gz sdf -k`)
* `check_urdf`: parsed OK, single root `base_footprint`, 13 links, 12 joints (5 moving, 7 fixed).
* `gz sdf -k urdf/hsr.urdf` and `gz sdf -k worlds/hsr_test_world.sdf`: Valid.
  (sdformat prints a harmless notice that `<gz_frame_id>` is not in its schema; Gazebo reads it —
  confirmed by the frame_ids in the sensor messages below.)
* All frame origins within 0.01 mm of CAD; LiDAR scan plane Z = 225.00 mm; front beam direction
  (0.866, 0, -0.5) = 30.00° below horizontal; camera optical frame looks along +X.
* 5 moving joints: 4 continuous (+Y), camera pan revolute (+Z, ±π).
* All 12 mesh placements within 0.1 mm of the SOLIDWORKS assembly component positions;
  overall envelope 440 x 400 x 340 mm.
* All inertia tensors positive definite and satisfy the triangle inequality.

## 3. Gazebo spawn and live test (`scripts/smoke_test.py`)
```
Waiting for data (5 s sim time)...
[PASS] topic /joint_states: receiving
[PASS] topic /odom: receiving
[PASS] topic /scan: receiving
[PASS] topic /front_range/scan: receiving
[PASS] topic /camera/image_raw: receiving
[PASS] topic /camera/camera_info: receiving
[PASS] joint names: camera_pan_joint, front_left_wheel_joint, front_right_wheel_joint, rear_left_wheel_joint, rear_right_wheel_joint
[PASS] LiDAR scan: frame 'laser_frame', 360 beams, 360 valid, min 0.21 m, max 5.66 m
       rear sector (180 +/- 4 deg) hits: 8 at [0.219, 0.206, 0.221, 4.095, 4.091, 4.076, 0.227, 0.219] m (camera mast shadow, expected ~0.21 m)
[PASS] Front range sensor: frame 'front_range_beam_frame', 5 rays, min 0.316 m (flat floor at 30 deg from 0.16 m height -> 0.320 m)
[PASS] Camera: 640x480 rgb8, frame 'camera_optical_frame', fx 457.0
       saved camera frame: /home/claude/logs/hsr_camera_view_final.png
[PASS] Drive forward 0.3 m/s x 3 s: odom moved 0.85 m; wheel speeds [4.0, 4.0, 4.0, 4.0] rad/s (expect ~4.0)
[PASS] Rotate in place 1 rad/s x 3 s: odom yaw change 162 deg, position drift 0.0 cm, left/right wheel -3.43/3.43 rad/s (opposite signs)
[PASS] Camera pan to +1.57 rad: reached +1.571 rad
[PASS] Camera pan to -1.00 rad: reached -1.000 rad
[PASS] Camera pan to +0.00 rad: reached -0.000 rad

15/15 checks passed
```

## 4. Skid-steer calibration (`scripts/calibrate_skid_steer.py`)
| | Rotation odom / truth | Straight odom / truth |
|---|---|---|
| Physical track 0.36 m | 217° / 152° (1.427) | 0.603 / 0.606 m (0.994) |
| Effective track 0.514 m (shipped) | 217° / 221° (0.981) | 0.646 / 0.650 m (0.994) |

## 5. Not yet tested
* Gazebo GUI rendering, and running on teammates' machines/GPUs.
* SLAM, Nav2/A*, and CNN nodes consuming these topics.
* Long-duration or rough-terrain driving; collisions with obstacles.
