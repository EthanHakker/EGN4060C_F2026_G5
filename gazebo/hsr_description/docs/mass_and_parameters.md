# Mass properties and parameter provenance

**[CAD]** = computed by SOLIDWORKS from the approved geometry and the material assigned in CAD
(aluminium 6061 = 2700 kg/m³, natural rubber = 960 kg/m³, ABS = 1020 kg/m³). These are
calculated, **not physically measured**. **[ESTIMATE]** = engineering estimate, tune against hardware.

## Per-part CAD mass properties (part frames)

| Part | Mesh | Mass (kg) | Density (kg/m³) | CoM (mm, part frame) | Source |
|---|---|---|---|---|---|
| HSR_Chassis | `chassis.stl` | 3.2257 | 2700 | -0.0, -0.0, 125.0 | CAD |
| HSR_Wheel | `wheel.stl` | 0.6786 | 960 | 0.0, 0.0, 0.0 | CAD |
| HSR_LiDAR_Mount | `lidar_mount.stl` | 0.2386 | 2700 | -0.0, 0.0, 10.0 | CAD |
| HSR_LiDAR | `lidar.stl` | 0.1802 | 1020 | -0.0, 0.0, 20.0 | CAD |
| HSR_Camera_Mast | `camera_mast.stl` | 0.0944 | 2700 | 0.0, -0.0, 33.1 | CAD |
| HSR_Camera_Pan_Head | `camera_pan_head.stl` | 0.0679 | 1020 | -0.0, 0.0, 16.4 | CAD |
| HSR_Camera | `camera.stl` | 0.1209 | 1020 | 0.3, 0.0, 0.0 | CAD |
| HSR_Front_Distance_Sensor | `front_range_sensor.stl` | 0.0249 | 1020 | -12.2, -0.0, 4.4 | CAD |
| HSR_Front_Sensor_Guard | `front_sensor_guard.stl` | 0.0451 | 2700 | 22.1, -0.0, 5.3 | CAD |

Inertia tensors (about CoM) are in `cad_mass_properties.json`. SOLIDWORKS reports products of
inertia as +∫xy dm; `gen_properties.py` flips their sign for URDF.

## URDF links

| Link | Contents | Mass (kg) | Source |
|---|---|---|---|
| `base_link` | chassis shell + LiDAR mount + camera mast + sensor guard | 3.604 | CAD |
|  | + internal payload (battery, computer, motor drivers): 2.5 kg uniform box 0.36 x 0.26 x 0.06 m centred at z = 0.10 m | 2.500 | **ESTIMATE** |
| `*_wheel_link` (x4) | solid rubber wheel Ø150 x 40 | 0.679 each | CAD |
| `lidar_link` | LiDAR body | 0.180 | CAD (solid ABS block; a real 2D LiDAR is typically 0.17–0.35 kg) |
| `camera_pan_link` | pan turntable + seat plate | 0.068 | CAD |
| `camera_link` | camera body + lens | 0.121 | CAD |
| `front_range_link` | distance sensor + mounting plate | 0.025 | CAD |
| **Total** | | **9.212** | 6.712 CAD + 2.5 estimate |

The CAD-only total (6.712 kg) matches the SOLIDWORKS assembly mass exactly.

## Simulation parameters [ESTIMATE]

| Parameter | Value | Notes |
|---|---|---|
| Wheel friction μ1 / μ2 | 1.0 / 0.8 | rubber on concrete/tile; μ2 lower to ease skid turning |
| Wheel contact kp / kd | 1e6 N/m / 100 N·s/m | stiff contact, minDepth 1 mm |
| Body friction | 0.4 | |
| Wheel joint damping | 0.01 N·m·s/rad | |
| Wheel effort / speed limit | 5 N·m / 13.3 rad/s (1.0 m/s) | URDF joint limits |
| DiffDrive limits | 0.8 m/s, 2.0 rad/s, 1.0 m/s², 3.0 rad/s² | |
| DiffDrive effective track | 0.514 m | **calibrated** in Gazebo vs ground truth (physical track 0.36 m x 1.427) |
| Pan joint | ±π rad, 1 N·m, 2 rad/s, damping 0.05 | no cable-wrap limit modelled |
| Pan controller | PID p=3.0, d=0.05, |cmd| ≤ 1 N·m | reaches target within 0.001 rad |
| LiDAR | 360 beams, 1°, 0.15–12 m, 10 Hz, σ = 0.01 m | generic 2D LiDAR |
| Camera | 640x480, 70° HFOV, 15 Hz, clip 0.05–30 m, σ = 0.007 | generic USB camera |
| Front distance sensor | 5 rays, ±7.5°, 0.02–4 m, 20 Hz, σ = 0.005 m | ultrasonic/ToF-like cone |
