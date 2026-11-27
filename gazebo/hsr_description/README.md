# hsr_description — Hazardous Environment Search Robot (Gazebo / ROS 2)

Robot description and Gazebo simulation setup for the UCF Intro to Robotics search-and-rescue
robot. Generated from the approved SOLIDWORKS assembly
(`CAD/Assembly/Hazardous_Environment_Search_Robot.SLDASM`, unchanged by the export).

**Status:** spawned and tested in Gazebo Harmonic + ROS 2 Jazzy (headless, Ubuntu 24.04) —
15/15 smoke-test checks passed. See `docs/test_report.md`. Not yet tested with a GUI,
on a teammate's machine, or with SLAM/Nav2/CNN nodes running.

---

## 1. Requirements

| Item | Version used for testing |
|---|---|
| OS | Ubuntu 24.04 (Windows users: WSL2 with Ubuntu 24.04 + WSLg) |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (installed by `ros-jazzy-ros-gz`) |

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-xacro \
                 ros-jazzy-robot-state-publisher ros-jazzy-teleop-twist-keyboard \
                 python3-colcon-common-extensions
```

Other ROS 2 / Gazebo pairings (e.g. Kilted + Ionic) should work but are untested.

## 2. Build

```bash
mkdir -p ~/hsr_ws/src
cp -r hsr_description ~/hsr_ws/src/          # or unzip hsr_description.zip there
cd ~/hsr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash                    # do this in every new terminal
```

Sourcing `install/setup.bash` also adds the package to `GZ_SIM_RESOURCE_PATH`, which is how
Gazebo finds the meshes (`package://hsr_description/meshes/...`).

## 3. Launch and spawn

```bash
ros2 launch hsr_description sim.launch.py                    # Gazebo GUI + robot
ros2 launch hsr_description sim.launch.py headless:=true     # no GUI (faster, servers/CI)
ros2 launch hsr_description sim.launch.py x:=-3 y:=-3 yaw:=0.785   # spawn pose
ros2 launch hsr_description sim.launch.py world:=/abs/path/your_world.sdf
```

The launch file starts Gazebo with `worlds/hsr_test_world.sdf` (8 x 8 m room with a doorway,
debris boxes and a pillar), `robot_state_publisher`, spawns the robot as model `hsr`, and
starts the ROS<->Gazebo bridge (`config/ros_gz_bridge.yaml`).

**Using your own world:** it must load `gz-sim-sensors-system` with `<render_engine>ogre2</render_engine>`
(plus physics, user-commands and scene-broadcaster systems) or the sensors will publish nothing.
Copy the `<plugin>` block from `worlds/hsr_test_world.sdf`.

**Without the launch file** (Gazebo only, no ROS nodes or bridge). With the ROS install above,
`gz` is only on your PATH after `source /opt/ros/jazzy/setup.bash`; a standalone
Gazebo Harmonic install (`sudo apt install gz-harmonic`) also works. Run from inside `hsr_description/`:
```bash
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=/path/containing/hsr_description_folder:$GZ_SIM_RESOURCE_PATH
gz sim -r worlds/hsr_test_world.sdf &
gz service -s /world/hsr_test_world/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 5000 \
  --req "sdf_filename: \"$PWD/urdf/hsr.urdf\", name: \"hsr\", pose: {position: {z: 0.01}}"
```

## 4. Drive, pan the camera, check it works

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard           # drive (publishes /cmd_vel)
ros2 topic pub --once /camera_pan/cmd_pos std_msgs/msg/Float64 "{data: 1.57}"   # pan left 90 deg
python3 src/hsr_description/scripts/smoke_test.py                # automated check (sim must be running)
```

## 5. Interfaces

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | in | skid-steer; max 0.8 m/s, 2.0 rad/s |
| `/camera_pan/cmd_pos` | std_msgs/Float64 | in | pan target angle, rad, ±π, + = left |
| `/odom` | nav_msgs/Odometry | out | wheel odometry `odom` → `base_footprint` |
| `/tf` | tf2_msgs/TFMessage | out | `odom` → `base_footprint` |
| `/joint_states` | sensor_msgs/JointState | out | 4 wheels + pan |
| `/scan` | sensor_msgs/LaserScan | out | 2D LiDAR, frame `laser_frame`, 360 beams @ 1°, 0.15–12 m, 10 Hz |
| `/front_range/scan` | sensor_msgs/LaserScan | out | front distance sensor, frame `front_range_beam_frame`, 5 rays over ~15°, 0.02–4 m, 20 Hz — **use the minimum valid range** |
| `/camera/image_raw` | sensor_msgs/Image | out | 640x480 rgb8, 70° HFOV, 15 Hz, frame `camera_optical_frame` |
| `/camera/camera_info` | sensor_msgs/CameraInfo | out | intrinsics |
| `/clock` | rosgraph_msgs/Clock | out | use `use_sim_time:=true` in all your nodes |

Static TF (`robot_state_publisher`) covers all fixed frames and the wheel/pan joints.

## 6. Frames (robot frame: X forward, Y left, Z up; origin = chassis centre at ground level)

| Frame | Position (m) | Notes |
|---|---|---|
| `base_footprint` | (0, 0, 0) | root; use as SLAM/Nav2 base frame |
| `base_link` | (0, 0, 0) | chassis + LiDAR mount + mast + guard (merged rigid body) |
| `*_wheel_link` | (±0.125, ±0.18, 0.075) | continuous joints about +Y |
| `lidar_link` | (0.08, 0, 0.205) | LiDAR body base |
| `laser_frame` | (0.08, 0, **0.225**) | **scan plane** |
| `camera_pan_link` | (−0.14, 0, 0.27) | revolute joint about +Z |
| `camera_link` | (−0.14, 0, 0.32) at pan 0 | body centre, X = look direction |
| `camera_optical_frame` | lens face, (−0.1125, 0, 0.32) at pan 0 | ROS optical convention (Z forward) |
| `front_range_link` | (0.225, 0, 0.16) | distance sensor emitter |
| `front_range_beam_frame` | same point | +X pitched **30° down**; beam hits flat floor 0.32 m ahead of the emitter |

## 7. Where the numbers come from

All geometry is **[CAD]** (from the approved model, mm → m). Mass/inertia of each part are
**[CAD]** values computed by SOLIDWORKS from geometry + assigned material — they are *not*
physical measurements. Everything else is an **[ESTIMATE]**. Full table:
`docs/mass_and_parameters.md`. All values live in `urdf/hsr_properties.xacro`, which is
generated by `scripts/gen_properties.py` from `docs/cad_mass_properties.json`.

Key estimates you will probably want to change: 2.5 kg internal payload (battery/computer),
wheel friction (μ1 = 1.0, μ2 = 0.8), the skid-steer effective track (0.514 m), and sensor
specs (generic, not a specific product).

## 8. Notes for SLAM, A* navigation and CNN detection

* **LiDAR rear shadow (accepted in design review):** the camera mast blocks ~5 beams at ±180°
  (readings ≈ 0.21 m). Filter them before SLAM, e.g. with `laser_filters`
  (`LaserScanAngularBoundsFilterInPlace`, remove |angle| > 177°) or set the scan's usable
  range minimum above 0.25 m.
* **SLAM (slam_toolbox):** `base_frame: base_footprint`, `odom_frame: odom`,
  `scan_topic: /scan`, `use_sim_time: true`.
* **Navigation footprint:** 0.44 m x 0.40 m (X −0.20 to +0.24, Y ±0.20). Inflate accordingly
  for A*/Nav2 costmaps.
* **Front distance sensor** sees obstacles below the LiDAR plane (Z < 0.225 m) and floor
  drop-offs: on flat floor it reads ≈ 0.32 m; a much larger value = hole/step down,
  a smaller value = low obstacle.
* **CNN:** the LiDAR and the front of the deck are visible at the bottom of the camera image
  when the camera faces forward (bottom ~20% of the frame). Crop or mask that region. For a
  test victim, uncomment the "Standing person" include in `worlds/hsr_test_world.sdf`
  (downloads from Gazebo Fuel on first run).

## 9. Skid-steer calibration

Skid-steer robots slip when turning, so Gazebo's DiffDrive needs an *effective* wheel
separation (0.514 m) rather than the physical 0.36 m track. With the physical value, the
robot only turned ~70% of the commanded angle and odometry over-reported yaw by ~43%. After
calibration: rotation odometry within 2% and straight-line within 1% of Gazebo ground truth.
If you change mass, friction or wheels, re-run with the sim running:
```bash
python3 src/hsr_description/scripts/calibrate_skid_steer.py
```
and update `skid_steer_wheel_separation` in `scripts/gen_properties.py`, then
`python3 scripts/gen_properties.py` and rebuild.

## 10. Package contents

```
hsr_description/
├── CMakeLists.txt, package.xml       ROS 2 ament_cmake package
├── README.md                         this file
├── urdf/
│   ├── hsr.urdf.xacro                main robot description (links, joints, visuals, collisions)
│   ├── hsr.gazebo.xacro              Gazebo sensors, drive/joint plugins, friction
│   ├── hsr_properties.xacro          GENERATED numeric values, tagged [CAD]/[ESTIMATE]
│   └── hsr.urdf                      GENERATED flat URDF (for non-ROS use / inspection)
├── meshes/                           9 binary STL files, metres, part frames (from SOLIDWORKS)
├── launch/sim.launch.py              Gazebo + spawn + robot_state_publisher + bridge
├── config/ros_gz_bridge.yaml         topic bridge definitions
├── worlds/hsr_test_world.sdf         test arena with required Gazebo systems
├── hooks/hsr_description.dsv.in      adds the package to GZ_SIM_RESOURCE_PATH
├── scripts/
│   ├── gen_properties.py             CAD JSON -> hsr_properties.xacro
│   ├── validate_urdf.py              geometry/inertia checks vs CAD
│   ├── smoke_test.py                 live simulation test (15 checks)
│   └── calibrate_skid_steer.py       measures turning slip vs ground truth
└── docs/
    ├── cad_mass_properties.json      raw SOLIDWORKS mass properties (SI)
    ├── mass_and_parameters.md        value provenance table
    ├── test_report.md                validation + simulation test record
    └── camera_view_test.png          camera frame captured during the test
```

After editing a `.xacro` file, regenerate the flat URDF:
`xacro urdf/hsr.urdf.xacro > urdf/hsr.urdf`.
