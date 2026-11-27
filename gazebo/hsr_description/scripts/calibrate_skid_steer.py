#!/usr/bin/env python3
"""Measure skid-steer slip in Gazebo and recommend the DiffDrive effective wheel separation.

Skid-steer robots scrub their tyres sideways when turning, so they rotate less than ideal
differential-drive kinematics predict. The gz DiffDrive plugin therefore needs an *effective*
wheel separation (larger than the physical track) for /cmd_vel angular rates and /odom yaw
to match reality. Re-run this after changing mass, friction or wheel geometry.

Usage (sim running in open space, robot stationary):
    python3 scripts/calibrate_skid_steer.py
Reads ground truth with the `gz model -m hsr -p` CLI and odometry from /odom.
"""
import math, re, subprocess, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CURRENT_SEPARATION_PARAM = "skid_steer_wheel_separation"   # in urdf/hsr_properties.xacro


def truth():
    out = subprocess.run(["gz", "model", "-m", "hsr", "-p"], capture_output=True, text=True, timeout=30).stdout
    nums = [float(v) for v in re.findall(r"[-+]?\d*\.\d+(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?",
                                         out.split("Pose [ XYZ (m) ] [ RPY (rad) ]:")[-1])[:6]]
    return nums[0], nums[1], nums[5]


class Cal(Node):
    def __init__(self):
        super().__init__("hsr_skid_calibration", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.odom = None
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def run(self, tw, sim_s):
        while self.get_clock().now().nanoseconds == 0 or self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        t0 = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - t0) / 1e9 < sim_s:
            self.pub.publish(tw); rclpy.spin_once(self, timeout_sec=0.02)

    def odom_pose(self):
        p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def unwrap(prev, new):
    d = (new - prev + math.pi) % (2 * math.pi) - math.pi
    return d


def main():
    rclpy.init(); n = Cal()
    n.run(Twist(), 1.0)
    # --- rotation: integrate yaw in small steps to avoid wrap issues
    rot = Twist(); rot.angular.z = 1.0
    t_yaw = o_yaw = 0.0; tp = truth(); op = n.odom_pose()
    for _ in range(4):
        n.run(rot, 1.0)
        tn, on = truth(), n.odom_pose()
        t_yaw += unwrap(tp[2], tn[2]); o_yaw += unwrap(op[2], on[2]); tp, op = tn, on
    n.run(Twist(), 1.5); tn, on = truth(), n.odom_pose()
    t_yaw += unwrap(tp[2], tn[2]); o_yaw += unwrap(op[2], on[2])
    # --- straight line
    fwd = Twist(); fwd.linear.x = 0.3
    t0, o0 = truth(), n.odom_pose()
    n.run(fwd, 3.0); n.run(Twist(), 1.5)
    t1, o1 = truth(), n.odom_pose()
    td, od = math.hypot(t1[0] - t0[0], t1[1] - t0[1]), math.hypot(o1[0] - o0[0], o1[1] - o0[1])

    ratio = o_yaw / t_yaw
    print(f"rotation : odom {math.degrees(o_yaw):7.1f} deg | ground truth {math.degrees(t_yaw):7.1f} deg | odom/truth {ratio:.3f}")
    print(f"straight : odom {od:7.3f} m   | ground truth {td:7.3f} m   | odom/truth {od / td:.3f}")
    print(f"=> multiply '{CURRENT_SEPARATION_PARAM}' in urdf/hsr_properties.xacro by {ratio:.3f}")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
