#!/usr/bin/env python3
"""Geometry/inertia validation of urdf/hsr.urdf against the approved SOLIDWORKS assembly.
Run from the package root after:  xacro urdf/hsr.urdf.xacro > urdf/hsr.urdf"""
import math, os, struct, sys
import numpy as np
from urdf_parser_py.urdf import URDF

P = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
r = URDF.from_xml_string(open(f"{P}/urdf/hsr.urdf").read())
fails = []

def T(xyz, rpy):
    cx, sx, cy, sy, cz, sz = (math.cos(rpy[0]), math.sin(rpy[0]), math.cos(rpy[1]), math.sin(rpy[1]),
                              math.cos(rpy[2]), math.sin(rpy[2]))
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]); Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    M = np.eye(4); M[:3, :3] = Rz @ Ry @ Rx; M[:3, 3] = xyz; return M

parent = {j.child: j for j in r.joints}
def fk(link):
    M = np.eye(4)
    while link in parent:
        j = parent[link]; o = j.origin
        M = T(o.xyz if o else [0, 0, 0], o.rpy if o else [0, 0, 0]) @ M; link = j.parent
    return M
def check(name, ok, detail):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    if not ok: fails.append(name)

print("== Frames (robot frame, mm) ==")
expect = {"front_left_wheel_link": (125, 180, 75), "front_right_wheel_link": (125, -180, 75),
          "rear_left_wheel_link": (-125, 180, 75), "rear_right_wheel_link": (-125, -180, 75),
          "lidar_link": (80, 0, 205), "laser_frame": (80, 0, 225), "camera_pan_link": (-140, 0, 270),
          "camera_link": (-140, 0, 320), "camera_optical_frame": (-112.5, 0, 320),
          "front_range_link": (225, 0, 160), "front_range_beam_frame": (225, 0, 160)}
for l, e in expect.items():
    p = fk(l)[:3, 3] * 1000
    check(l, np.allclose(p, e, atol=0.01), f"{np.round(p, 2).tolist()} (CAD {list(e)})")

L = fk("laser_frame")
check("LiDAR scan plane", abs(L[2, 3] - 0.225) < 1e-6 and np.allclose(L[:3, 2], [0, 0, 1]),
      f"Z = {L[2, 3]*1000:.2f} mm, plane normal {np.round(L[:3, 2], 4).tolist()}")
bx = fk("front_range_beam_frame")[:3, 0]
tilt = math.degrees(math.asin(-bx[2]))
check("Range beam tilt", abs(tilt - 30) < 1e-6 and bx[0] > 0 and abs(bx[1]) < 1e-9,
      f"direction {np.round(bx, 4).tolist()} = {tilt:.2f} deg below horizontal, forward")
O = fk("camera_optical_frame")
check("Camera optical frame", np.allclose(O[:3, 2], [1, 0, 0]) and np.allclose(O[:3, 0], [0, -1, 0]),
      "optical Z = robot +X (looks forward), optical X = robot -Y (image right)")

print("== Joints ==")
mov = [j for j in r.joints if j.type != "fixed"]
check("Moving joint count", len(mov) == 5, f"{len(mov)}: " + ", ".join(f"{j.name}({j.type}, axis {j.axis})" for j in mov))
for j in mov:
    if "wheel" in j.name:
        check(j.name, j.type == "continuous" and list(j.axis) == [0, 1, 0], "continuous about +Y")
    else:
        check(j.name, j.type == "revolute" and list(j.axis) == [0, 0, 1], f"revolute about +Z, limits [{j.limit.lower:.3f}, {j.limit.upper:.3f}] rad")

print("== Mesh placement vs SOLIDWORKS assembly component boxes ==")
cad = {"chassis": [-200, -150, 65, 200, 150, 185], "lidar_mount": [42.5, -37.5, 185, 117.5, 37.5, 205],
       "lidar": [42.5, -37.5, 205, 117.5, 37.5, 245], "camera_mast": [-165, -25, 185, -115, 25, 270],
       "camera_pan_head": [-165, -35, 270, -115, 35, 300], "camera": [-162.5, -32.5, 300, -112.5, 32.5, 340],
       "front_range_sensor": [200, -22.5, 140, 230, 22.5, 180], "front_sensor_guard": [200, -33, 140, 240, 33, 185],
       "wheel@front_left": [50, 160, 0, 200, 200, 150], "wheel@front_right": [50, -200, 0, 200, -160, 150],
       "wheel@rear_left": [-200, 160, 0, -50, 200, 150], "wheel@rear_right": [-200, -200, 0, -50, -160, 150]}
def stl_pts(f):
    b = open(f, "rb").read(); n = struct.unpack("<I", b[80:84])[0]
    assert len(b) == 84 + 50 * n, f"{f}: not a valid binary STL"
    return np.array([struct.unpack("<9f", b[84 + 50 * i + 12:84 + 50 * i + 48]) for i in range(n)]).reshape(-1, 3)
allpts = []
for link in r.links:
    for v in link.visuals:
        fn = getattr(v.geometry, "filename", None)
        if not fn: continue
        assert fn.startswith("package://hsr_description/meshes/"), fn
        name = fn.split("/")[-1][:-4]
        path = f"{P}/meshes/{name}.stl"
        key = f"wheel@{link.name.replace('_wheel_link', '')}" if name == "wheel" else name
        o = v.origin; M = fk(link.name) @ T(o.xyz if o else [0, 0, 0], o.rpy if o else [0, 0, 0])
        p = (M[:3, :3] @ stl_pts(path).T).T + M[:3, 3]; allpts.append(p)
        box = list(p.min(0) * 1000) + list(p.max(0) * 1000)
        err = max(abs(a - b) for a, b in zip(box, cad[key]))
        check(f"mesh {key}", err < 0.25, f"{fn} -> max box error {err:.2f} mm")
p = np.vstack(allpts); ext = (p.max(0) - p.min(0)) * 1000
check("Overall size", np.allclose(ext, [440, 400, 340], atol=0.5), f"{np.round(ext, 1).tolist()} mm L x W x H (CAD 440 x 400 x 340)")

print("== Inertials ==")
tot = 0
for l in r.links:
    if not l.inertial: continue
    i = l.inertial.inertia
    I = np.array([[i.ixx, i.ixy, i.ixz], [i.ixy, i.iyy, i.iyz], [i.ixz, i.iyz, i.izz]]); e = np.linalg.eigvalsh(I)
    tot += l.inertial.mass
    check(l.name, l.inertial.mass > 0 and e.min() > 0 and e[0] + e[1] >= e[2] * 0.999,
          f"m = {l.inertial.mass:.4f} kg, principal I = {np.round(e, 7).tolist()}")
print(f"  total mass {tot:.3f} kg (CAD 6.712 kg + 2.5 kg estimated payload)")

print(f"\nRESULT: {'ALL CHECKS PASSED' if not fails else 'FAILED: ' + ', '.join(fails)}")
sys.exit(1 if fails else 0)
