#!/usr/bin/env python3
"""HSR simulation smoke test.  Start the sim first:
       ros2 launch hsr_description sim.launch.py            (or headless:=true)
   then, in a second sourced terminal:
       python3 <path-to>/hsr_description/scripts/smoke_test.py
   Spawn the robot in open space (default pose 0,0 in hsr_test_world is fine).
   Checks joints, sensor topics/frames/data, driving, rotation in place and camera pan.
   Optionally saves one camera frame:  --save-image /tmp/hsr_camera.png
"""
import math, struct, sys, time, zlib
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Float64

EXPECTED_JOINTS = {"front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint",
                   "rear_right_wheel_joint", "camera_pan_joint"}


class Smoke(Node):
    def __init__(self):
        super().__init__("hsr_smoke_test", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.last = {}
        for topic, typ, qos in [("/joint_states", JointState, 10), ("/odom", Odometry, 10),
                                ("/scan", LaserScan, qos_profile_sensor_data),
                                ("/front_range/scan", LaserScan, qos_profile_sensor_data),
                                ("/camera/image_raw", Image, qos_profile_sensor_data),
                                ("/camera/camera_info", CameraInfo, qos_profile_sensor_data)]:
            self.create_subscription(typ, topic, lambda m, t=topic: self.last.__setitem__(t, m), qos)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pan = self.create_publisher(Float64, "/camera_pan/cmd_pos", 10)

    def spin_for(self, sim_seconds, during=None, wall_timeout=None):
        """Spin until `sim_seconds` of simulation time have elapsed (robust to slow sims)."""
        wall_timeout = wall_timeout or sim_seconds * 20 + 30
        start_wall = time.time()
        while self.get_clock().now().nanoseconds == 0 and time.time() - start_wall < 30:
            rclpy.spin_once(self, timeout_sec=0.05)
        t0 = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - t0) / 1e9 < sim_seconds:
            if during: during()
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.time() - start_wall > wall_timeout:
                return False
        return True


def yaw_of(odom):
    q = odom.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def write_png(path, img):
    if img.encoding not in ("rgb8", "bgr8"):
        return False
    raw = bytes(img.data); w, h, step = img.width, img.height, img.step
    rows = b"".join(b"\x00" + raw[y * step:y * step + 3 * w] for y in range(h))
    def chunk(t, d): return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(rows, 6)) + chunk(b"IEND", b""))
    return True


def main():
    save = sys.argv[sys.argv.index("--save-image") + 1] if "--save-image" in sys.argv else None
    rclpy.init(); n = Smoke(); results = []
    def check(name, ok, detail):
        results.append(ok); print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)

    print("Waiting for data (5 s sim time)...", flush=True)
    n.spin_for(5.0)
    for t in ["/joint_states", "/odom", "/scan", "/front_range/scan", "/camera/image_raw", "/camera/camera_info"]:
        check(f"topic {t}", t in n.last, "receiving" if t in n.last else "NO DATA")
    if len(results) != sum(results):
        print("Missing topics - is the simulation running and the sensors system loaded?"); sys.exit(1)

    js = n.last["/joint_states"]
    check("joint names", EXPECTED_JOINTS <= set(js.name), ", ".join(js.name))

    s = n.last["/scan"]; valid = [r for r in s.ranges if s.range_min <= r <= s.range_max]
    rear = [r for i, r in enumerate(s.ranges) if abs(abs(s.angle_min + i * s.angle_increment) - math.pi) < math.radians(4)
            and s.range_min <= r <= s.range_max]
    check("LiDAR scan", s.header.frame_id == "laser_frame" and len(s.ranges) == 360 and len(valid) > 200,
          f"frame '{s.header.frame_id}', {len(s.ranges)} beams, {len(valid)} valid, "
          f"min {min(valid):.2f} m, max {max(valid):.2f} m")
    print(f"       rear sector (180 +/- 4 deg) hits: {len(rear)} at {[round(r, 3) for r in rear]} m "
          f"(camera mast shadow, expected ~0.21 m)")

    f = n.last["/front_range/scan"]; fv = [r for r in f.ranges if f.range_min <= r <= f.range_max]
    fmin = min(fv) if fv else float("nan")
    check("Front range sensor", f.header.frame_id == "front_range_beam_frame" and len(fv) == len(f.ranges)
          and abs(fmin - 0.32) < 0.03,
          f"frame '{f.header.frame_id}', {len(f.ranges)} rays, min {fmin:.3f} m "
          f"(flat floor at 30 deg from 0.16 m height -> 0.320 m)")

    im, ci = n.last["/camera/image_raw"], n.last["/camera/camera_info"]
    check("Camera", im.width == 640 and im.height == 480 and im.header.frame_id == "camera_optical_frame"
          and ci.k[0] > 0, f"{im.width}x{im.height} {im.encoding}, frame '{im.header.frame_id}', fx {ci.k[0]:.1f}")
    if save:
        print(f"       saved camera frame: {save}" if write_png(save, im) else "       (could not save image)")

    # --- drive forward
    o0 = n.last["/odom"]; x0, y0 = o0.pose.pose.position.x, o0.pose.pose.position.y
    tw = Twist(); tw.linear.x = 0.3
    n.spin_for(3.0, during=lambda: n.cmd.publish(tw))
    js = n.last["/joint_states"]; wv = {nm: v for nm, v in zip(js.name, js.velocity)}
    n.cmd.publish(Twist()); n.spin_for(1.5, during=lambda: n.cmd.publish(Twist()))
    o1 = n.last["/odom"]; d = math.hypot(o1.pose.pose.position.x - x0, o1.pose.pose.position.y - y0)
    wheel_w = [wv.get(j, 0) for j in sorted(EXPECTED_JOINTS - {"camera_pan_joint"})]
    check("Drive forward 0.3 m/s x 3 s", 0.6 < d < 1.0 and all(w > 2.0 for w in wheel_w),
          f"odom moved {d:.2f} m; wheel speeds {[round(w, 2) for w in wheel_w]} rad/s (expect ~4.0)")

    # --- rotate in place
    yaw0 = yaw_of(n.last["/odom"]); p0 = n.last["/odom"].pose.pose.position
    tw = Twist(); tw.angular.z = 1.0
    n.spin_for(3.0, during=lambda: n.cmd.publish(tw))
    js = n.last["/joint_states"]; wv = {nm: v for nm, v in zip(js.name, js.velocity)}
    n.cmd.publish(Twist()); n.spin_for(1.5, during=lambda: n.cmd.publish(Twist()))
    o2 = n.last["/odom"]; dyaw = math.degrees((yaw_of(o2) - yaw0 + math.pi) % (2 * math.pi) - math.pi)
    drift = math.hypot(o2.pose.pose.position.x - p0.x, o2.pose.pose.position.y - p0.y)
    lr = (wv["front_left_wheel_joint"], wv["front_right_wheel_joint"])
    check("Rotate in place 1 rad/s x 3 s", lr[0] < -0.5 and lr[1] > 0.5 and abs(dyaw) > 60 and drift < 0.10,
          f"odom yaw change {dyaw:.0f} deg, position drift {drift*100:.1f} cm, "
          f"left/right wheel {lr[0]:.2f}/{lr[1]:.2f} rad/s (opposite signs)")

    # --- camera pan
    for target in (1.5708, -1.0, 0.0):
        n.spin_for(4.0, during=lambda: n.pan.publish(Float64(data=target)))
        js = n.last["/joint_states"]; pos = dict(zip(js.name, js.position))["camera_pan_joint"]
        check(f"Camera pan to {target:+.2f} rad", abs(pos - target) < 0.05, f"reached {pos:+.3f} rad")

    print(f"\n{sum(results)}/{len(results)} checks passed")
    n.destroy_node(); rclpy.shutdown()
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
