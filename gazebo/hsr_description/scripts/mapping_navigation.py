#!/usr/bin/env python3
"""
Test of mapping and navigation
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

# ------------ mapping and navigation ------------ #

size = 20
detail = 6
#grid = [[0 for n in range(size*)] for m in range(size)]
dt = 1
t=0
remap = 0

class A_node:
    def __init__(self, x, y, parent, g, h):
        self.x = x
        self.y = y
        self.parent = parent
        self.h = h
        self.g = g
        self.line = None #cv.create_line((x+0.5)*usize, (y+0.5)*usize, (x+0.5)*usize, (y+0.5)*usize, fill='blue', width=1, tags="front")

    def path(self):
        if self.parent==None: return [[self.x,self.y]]
        return self.parent.path() + [[self.x,self.y]]

def h(dx,dy):
    return math.sqrt(2)*min(abs(dx),abs(dy))+abs(dx-dy)

map_arr = [[0 for n in range(size*detail)] for m in range(size*detail)]
nav_arr = [[0 for n in range(size*detail)] for m in range(size*detail)]



class robot:
    def __init__(self, ID, x, y, orientation, vision, node):
        self.ID = ID
        self.x = x
        self.y = y
        self.o = orientation
        self.o_true = 0
        self.vision = vision
        self.node = node
        self.v = 0.0
        self.t_r = 0
        self.pth = []
        self.nd = 0
        self.mode = 0
        self.dest = [0.0,0.0]
        self.done = False
        self.map_arr = map_arr #[[0 for n in range(size)] for m in range(size)]
        self.nav_arr = nav_arr #[[0 for n in range(size)] for m in range(size)]

    def forward(self, du, v):
        global t
        self.t_r = int(du/v)
        self.v = v

    def lookat(self, x, y):
        if self.x==x and self.y==y: return
        if y==self.y:
            if x<self.x: self.o = -math.pi*0.5
            else: self.o = math.pi*0.5
        elif y>self.y: self.o = math.atan((x-self.x)/(y-self.y))
        else: self.o = (math.atan((x-self.x)/(y-self.y))+math.pi)
        self.o = self.o%(2.0*math.pi)
        
    def goto(self, x, y):
        self.lookat(x,y)
        
        self.forward(math.sqrt(math.pow(x-self.x,2)+math.pow(y-self.y,2)),0.1)

    #def val_dist(self, du, o):
    #    x = int(self.x+math.sin(self.o_true+o)*du*detail)
    #    y = int(self.y+math.cos(self.o_true+o)*du*detail)
    #    if x<0 or x>=size*detail or y<0 or y>=size*detail: return 0
    #    if grid[x][y]>1: return 1
    #    return 0

    def map_dist(self, du, o):
        x = int(self.x+math.sin(self.o_true+o)*du*detail)
        y = int(self.y+math.cos(self.o_true+o)*du*detail)
        if x<0 or x>=size*detail or y<0 or y>=size*detail: return -1
        return self.map_arr[x][y]

    def nav_dist(self, du, o):
        x = int(self.x+math.sin(self.o_true+o)*du*detail)
        y = int(self.y+math.cos(self.o_true+o)*du*detail)
        if x<0 or x>=size*detail or y<0 or y>=size*detail: return -1
        return self.nav_arr[x][y]

    def map_add(self, du, o, val):
        x = int(self.x+math.sin(self.o_true+o)*du*detail)
        y = int(self.y+math.cos(self.o_true+o)*du*detail)
        if x<0 or x>=size*detail or y<0 or y>=size*detail: return
        self.map_arr[x][y] = val
    
    def dist(self, o=0):
        #for n in range(self.vision*5):
        #    if self.val_dist(n*0.2,o)==1: return n*0.2
        #return 0
        s = self.node.last["/scan"]
        return s.ranges[int(math.degrees(o%(2*math.pi)))]

    def map_area(self):
        for theta in range(360):
            rad = theta*math.pi/180
            curD = self.dist(self.o_true+rad)*detail
            for n in range(int(curD)):
                map_val = self.map_dist(n,theta*math.pi/180)
                if map_val==-1: break
                temp_val = 0
                if n==int(curD)-1: temp_val = 1
                
                if map_val==0:
                    if temp_val==0: self.map_add(n,theta*math.pi/180, 1)
                    else:
                        self.map_add(n,theta*math.pi/180, 2)
                        break
                else:
                    if temp_val!=0:
                        self.map_add(n,theta*math.pi/180, map_val+1)
                        break
                
    
    def update_nav(self):
        for n in range(int(self.vision*2*detail)):
            X = int(self.x-self.vision+n)
            if X>=0 and X<size*detail:
                for m in range(int(self.vision*2*detail)):
                    Y = int(self.y-self.vision+m)
                    if Y>=0 and Y<size*detail:
                        map_val = self.map_arr[X][Y]
                        nav_val = self.nav_arr[X][Y]
                        if map_val==1 and nav_val==0:
                            self.nav_arr[X][Y] = 1
                        elif map_val>1:
                            for a in range(5):
                                for b in range(5):
                                    if abs(a-2)+abs(b-2)<=2:
                                        A=int(max(0,min(size*detail-1,X+a-2)))
                                        B=int(max(0,min(size*detail-1,Y+b-2)))
                                        self.nav_arr[A][B] = -1


    def find(self):
        s = 1
        x = round(self.x+math.sin(self.o_true)*5)
        y = round(self.y+math.cos(self.o_true)*5)
        while s<size*detail:
            X = max(1,min(size*detail-2,x+s))
            Y = max(1,min(size*detail-2,y))
            if self.nav_arr[X][Y]==0 and self.closest(X, Y): return [X,Y]
            X = max(1,min(size*detail-2,x-s))
            Y = max(1,min(size*detail-2,y))
            if self.nav_arr[X][Y]==0 and self.closest(X, Y): return [X,Y]
            X = max(1,min(size*detail-2,x))
            Y = max(1,min(size*detail-2,y+s))
            if self.nav_arr[X][Y]==0 and self.closest(X, Y): return [X,Y]
            X = max(1,min(size*detail-2,x))
            Y = max(1,min(size*detail-2,y-s))
            if self.nav_arr[X][Y]==0 and self.closest(X, Y): return [X,Y]
            for n in range(s-1):
                X = max(1,min(size*detail-2,x-s+n+1))
                Y = max(1,min(size*detail-2,y+n+1))
                if self.nav_arr[X][Y]==0 and self.closest(X, Y): return [X,Y]
                X = max(1,min(size*detail-2,x-s+n+1))
                Y = max(1,min(size*detail-2,y-n-1))
                if self.nav_arr[X][Y]==0 and self.closest(X, Y): return [X,Y]
                X = max(1,min(size*detail-2,x+s-n-1))
                Y = max(1,min(size*detail-2,y+n+1))
                if self.nav_arr[X][Y]==0 and self.closest(X, Y): return [X,Y]
                X = max(1,min(size*detail-2,x+s-n-1))
                Y = max(1,min(size*detail-2,y-n-1))
                if self.nav_arr[X][Y]==0 and self.closest(X, Y): return [X,Y]
            s+=1
        s=1
        while s<size*detail:
            X = max(1,min(size*detail-2,x+s))
            Y = max(1,min(size*detail-2,y))
            if self.nav_arr[X][Y]==0: return [X,Y]
            X = max(1,min(size*detail-2,x-s))
            Y = max(1,min(size*detail-2,y))
            if self.nav_arr[X][Y]==0: return [X,Y]
            X = max(1,min(size*detail-2,x))
            Y = max(1,min(size*detail-2,y+s))
            if self.nav_arr[X][Y]==0: return [X,Y]
            X = max(1,min(size*detail-2,x))
            Y = max(1,min(size*detail-2,y-s))
            if self.nav_arr[X][Y]==0: return [X,Y]
            for n in range(s-1):
                X = max(1,min(size*detail-2,x-s+n+1))
                Y = max(1,min(size*detail-2,y+n+1))
                if self.nav_arr[X][Y]==0: return [X,Y]
                X = max(1,min(size*detail-2,x-s+n+1))
                Y = max(1,min(size*detail-2,y-n-1))
                if self.nav_arr[X][Y]==0: return [X,Y]
                X = max(1,min(size*detail-2,x+s-n-1))
                Y = max(1,min(size*detail-2,y+n+1))
                if self.nav_arr[X][Y]==0: return [X,Y]
                X = max(1,min(size*detail-2,x+s-n-1))
                Y = max(1,min(size*detail-2,y-n-1))
                if self.nav_arr[X][Y]==0: return [X,Y]
            s+=1
        #print("no more")
        return [-1,-1]

    def closest(self, x, y):
        if True: return True
        choose = True
        d = h(x-self.x, y-self.y)
        for R in Rs:
            if h(x-R.x, y-R.y)<d: choose = False
        return choose

    def A_star(self, x1, y1):
        #time_arr = [0, 0, 0, 0]
        if x1<0 or y1<0: return [[x0, y0]]
        ni=1
        x0 = round(self.x)
        y0 = round(self.y)
        opn = [A_node(x0,y0,None, 0, h(x1-x0,y1-y0))]
        clsd = []
        while len(opn)!=0:
            #t0 = time.time()
            cur = opn[0]
            N = 0
            M = size*size*detail*detail
            for n in range(len(opn)):
                if (opn[n].h+opn[n].g)<M or ((opn[n].h+opn[n].g)==M and opn[n].h<cur.h):
                    cur = opn[n]
                    N = n
                    M = opn[n].h+opn[n].g
            clsd.append(cur)
            opn.pop(N)
            
            #time_arr[0] += time.time()-t0
            
            for a in range(3):
                for b in range(3):
                    if a!=1 or b!=1:
                        X = cur.x-1+a
                        Y = cur.y-1+b
                        if X>=0 and X<size*detail and Y>=0 and Y<size*detail:
                            if self.nav_arr[X][Y]>=0:
                                #t0 = time.time()
                                dg = 1
                                brk = 0
                                if a!=1 and b!=1: dg = math.sqrt(2)
                                for nd in opn:
                                    if X==nd.x and Y==nd.y:
                                        if cur.g+dg<nd.g:
                                            nd.parent = cur
                                            nd.g = cur.g+dg
                                            #cv.coords(nd.line, [((nd.x+0.5)*usize, (nd.y+0.5)*usize), ((cur.x+0.5)*usize, (cur.y+0.5)*usize)])
                                        brk = 1
                                        break
                                for nd in clsd:
                                    if X==nd.x and Y==nd.y:
                                        if cur.g+dg<nd.g:
                                            nd.parent = cur
                                            nd.g = cur.g+dg
                                            #cv.coords(nd.line, [((nd.x+0.5)*usize, (nd.y+0.5)*usize), ((cur.x+0.5)*usize, (cur.y+0.5)*usize)])
                                        brk = 1
                                        break
                                    
                                if brk==1: continue
                                newn = A_node(X, Y, cur, cur.g+dg, h(x1-X,y1-Y))

                                #time_arr[1] += time.time()-t0
                                #t0 = time.time()
                                
                                if newn.x==x1 and newn.y==y1:
                                    pth = newn.path()
                                    for nd in opn:
                                        #cv.delete(nd.line)
                                        del nd
                                    for nd in clsd:
                                        #cv.delete(nd.line)
                                        del nd
                                    opn.clear()
                                    clsd.clear()
                                    #for tv in range(len(time_arr)): time_arr[tv] /= ni
                                    #print(time_arr+[ni])
                                    return pth
                                opn.append(newn)
                                #cv.coords(newn.line, [((newn.x+0.5)*usize, (newn.y+0.5)*usize), ((cur.x+0.5)*usize, (cur.y+0.5)*usize)])
                                #time_arr[2] += time.time()-t0
            #t0 = time.time()
            #time_arr[3] += time.time()-t0
            ni+=1
        print("no path")
        self.done = True
        return [[x0, y0]]

    def map(self):
        global t
        global remap
        #t00 = time.time()
        #while 1:
        d_tr = 0
        n = self.node
        o0 = n.last["/odom"]
        self.x, self.y = o0.pose.pose.position.x+10.0, o0.pose.pose.position.y+10.0
        self.o_true = yaw_of(n.last["/odom"])
        d_o = ((self.o-self.o_true)%(2.0*math.pi))
        if d_o>math.pi: d_o-=2.0*math.pi
        if self.done:
            #print("done")
            return
        if remap != 0:
            if remap == self.ID: remap = 0
            else:
                self.t_r = 0
                self.v = 0.0
                self.nd = 0
        if self.nd > 0:
            if self.nd >= len(self.pth):
                self.nd = 0
            else:
                if self.t_r <= 0:
                    self.goto(self.pth[self.nd][0],self.pth[self.nd][1])
                    self.nd += 1
                else:
                    d = self.dist()
                    dl = self.dist(-0.5*math.pi)
                    dr = self.dist(0.5*math.pi)
                    if d!=0 and d<=3 or self.nav_dist(1,0)==0:
                        self.t_r = 0
                        self.v = 0.0
                        self.nd = 0
                    else:
                        #self.x+= math.sin(self.o)*self.v
                        #self.y+= math.cos(self.o)*self.v
                        d_tr = 1
                        #self.t_r -= 1
        else:
            if remap == 0: remap = self.ID
            self.nd = 0
            self.v = 0.0
            if self.mode==0:
                self.map_area()
                self.mode += 1
            elif self.mode==1:
                self.update_nav()
                self.mode += 1
            else:
                pos = self.find()
                if pos==[-1,-1]:
                    self.done = True
                    return
                else:
                    self.pth = self.A_star(pos[0],pos[1])
                    self.goto(self.pth[0][0],self.pth[0][1])
                    self.nd += 1
                    self.mode = 0
        tw = Twist()
        if abs(d_o)>0.1:
            tw.angular.z = d_o
            tw.linear.x = 0.0
        else:
            tw.linear.x = self.v
            self.t_r -= d_tr
        n.cmd.publish(tw)




        


# ------------ Everything else ------------ #

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
    o0 = n.last["/odom"]
    x0, y0 = o0.pose.pose.position.x, o0.pose.pose.position.y

    tw = Twist()
    tw.linear.x = 0.3

    n.spin_for(3.0, during=lambda: n.cmd.publish(tw))
    js = n.last["/joint_states"]
    wv = {nm: v for nm, v in zip(js.name, js.velocity)}

    n.cmd.publish(Twist())
    n.spin_for(1.5, during=lambda: n.cmd.publish(Twist()))

    o1 = n.last["/odom"]
    d = math.hypot(o1.pose.pose.position.x - x0, o1.pose.pose.position.y - y0)

    wheel_w = [wv.get(j, 0) for j in sorted(EXPECTED_JOINTS - {"camera_pan_joint"})]
    check("Drive forward 0.3 m/s x 3 s", 0.6 < d < 1.0 and all(w > 2.0 for w in wheel_w),
          f"odom moved {d:.2f} m; wheel speeds {[round(w, 2) for w in wheel_w]} rad/s (expect ~4.0)")

    # --- rotate in place
    yaw0 = yaw_of(n.last["/odom"])
    p0 = n.last["/odom"].pose.pose.position
    
    tw = Twist()
    tw.angular.z = 1.0
    
    n.spin_for(3.0, during=lambda: n.cmd.publish(tw))
    js = n.last["/joint_states"]
    wv = {nm: v for nm, v in zip(js.name, js.velocity)}
    
    n.cmd.publish(Twist())
    n.spin_for(1.5, during=lambda: n.cmd.publish(Twist()))
    
    o2 = n.last["/odom"]
    dyaw = math.degrees((yaw_of(o2) - yaw0 + math.pi) % (2 * math.pi) - math.pi)
    
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
    print("engaging mapping mode")
    R = robot(1,0.0,0.0,0.0,8.0,n)
    while 1:
        R.map()
        if R.done: break
    
    n.destroy_node(); rclpy.shutdown()
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
