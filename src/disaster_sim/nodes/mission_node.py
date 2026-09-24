#!/usr/bin/env python3
"""Scripted search-and-rescue mission.

A visualisation of an already-evaluated decision engine - NOT a new perception
system. Sequence:

  1. six drones lift off from the base station
  2. they fan out toward the damaged building in a swarm arc
  3. one drone (the scanner) peels off to the east facade and flies a scan pattern
  4. its camera actually sees the victim - honest OpenCV orange-blob detection
     on /<scanner>/camera/image_raw, gated to the scan phase so nothing else
     orange (van livery, containers) can trip it
  5. detection fires -> alert on /base_station/alerts (std_msgs/String) with the
     victim position and detecting drone id, plus a Marker for RViz
  6. a clear terminal banner is printed
  7. the node exits cleanly (the launch file shuts the rest down on its exit)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy,
                       QoSHistoryPolicy)
from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

try:
    import cv2
    HAVE_CV2 = True
except Exception:
    HAVE_CV2 = False

# --- world layout (all measured; see tools/measure_meshes.py) --------------
# datum = city ground plate top = world Z 0.
# damaged_building at (46, 0, 0.33), yaw 0; carved victim window (mesh local
# +X) faces east; world window centre ~ (51.7, 0, 7.4).
BASE = (125.0, -8.0)                # 5G van, ~79 m east of the building
BUILDING = (46.0, 0.0)             # building origin (swarm heads here)
VICTIM = (51.7, 0.0, 7.4)          # window centre = alert position

DRONES = ["drone_0", "drone_1", "drone_2", "drone_3", "drone_4", "drone_5"]
SCANNER = "drone_2"

# spawn / launch / return slots: 3x2, 3 m apart, just south of the van
SLOTS = [(121.0, -13.0), (124.0, -13.0), (127.0, -13.0),
         (121.0, -16.0), (124.0, -16.0), (127.0, -16.0)]

# swarm assembly arc, east of the building (building max x ~ 58.5), facing it
FAN = {
    "drone_0": (72.0, -16.0, 9.0),
    "drone_1": (70.0, -7.0, 11.0),
    "drone_2": (68.0, 0.0, 12.0),
    "drone_3": (70.0, 8.0, 12.0),
    "drone_4": (72.0, 17.0, 10.0),
    "drone_5": (76.0, 25.0, 8.5),
}

# camera boomed 0.8 m forward, pitched 0.72 rad (41 deg) down, HFOV 1.40,
# VFOV ~64 deg. At scan waypoint (60,0,13.5) the window (51.7,0,7.4) sits ~2 deg
# off the optical axis at 7.5 m horizontal standoff -> well inside frame.
APPROACH_POS = (66.0, -12.0, 7.0)   # SE of the window, low - victim NOT framed
SCAN_PATH = [
    (62.0, -10.0, 8.0),     # 0  south end of the east facade, low
    (62.0, -6.0, 10.5),     # 1  climb
    (62.0, -2.0, 12.5),     # 2  up to facade height
    (62.0, 6.0, 12.5),      # 3  sweep north along the facade
    (60.0, 3.0, 13.5),      # 4  close the standoff
    (60.0, 0.0, 13.5),      # 5  squared on the carved window   <- MIN_SCAN_WP
    (58.0, -3.0, 13.5),     # 6  lateral confirm pass
    (60.0, 0.0, 13.5),      # 7
    (61.0, 4.0, 13.0),      # 8
    (60.0, 0.0, 13.5),      # 9
]
MIN_SCAN_WP = 5             # ignore detections before this scan waypoint
SCAN_DWELL = 3.0           # s to hold at each scan waypoint (visible pause)
HOLD_POS = (60.0, 0.0, 13.5)

LAUNCH_Z = 7.0
ARRIVE_TOL = 1.2
ORANGE_LO = (5, 90, 90)
ORANGE_HI = (30, 255, 255)
MIN_BLOB_AREA = 90
CONFIRM_FRAMES = 4


def base_slot(i):
    return (SLOTS[i][0], SLOTS[i][1], LAUNCH_Z)


class Mission(Node):
    def __init__(self):
        super().__init__("mission_node")
        self.declare_parameter("hold", False)
        self.hold = bool(self.get_parameter("hold").value)
        self.cmd = {d: self.create_publisher(Pose, f"{d}/cmd_pose", 10) for d in DRONES}
        self.pos = {d: None for d in DRONES}
        self.target = {d: None for d in DRONES}
        for d in DRONES:
            self.create_subscription(PoseStamped, f"{d}/pose",
                                     lambda m, dd=d: self._pose(dd, m), 10)

        self.alert_pub = self.create_publisher(String, "/base_station/alerts", 10)
        # transient-local so the marker survives for late/again subscribers (RViz)
        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            Marker, "/base_station/alert_marker", latched)

        self.detect_on = False
        self.hits = 0
        self.last_blob = 0.0
        self.create_subscription(Image, f"{SCANNER}/camera/image_raw",
                                 self._image, qos_profile_sensor_data)

        self.phase = "WAIT"
        self.entered = True
        self.t0 = self.now()
        self.phase_t = self.now()
        self.scan_i = 0
        self.scan_start = self.now()
        self.wp_t = self.now()
        self.detected = False
        self.done = False
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"mission_node up (OpenCV detection "
            f"{'ENABLED' if HAVE_CV2 else 'UNAVAILABLE - pose fallback'})")

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _pose(self, d, msg):
        self.pos[d] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def go(self, d, xyz):
        self.target[d] = tuple(map(float, xyz))
        self._send(d)

    def _send(self, d):
        xyz = self.target[d]
        if xyz is None:
            return
        p = Pose()
        p.position.x, p.position.y, p.position.z = xyz
        yaw = math.atan2(BUILDING[1] - xyz[1], BUILDING[0] - xyz[0])
        p.orientation.z = math.sin(yaw / 2)
        p.orientation.w = math.cos(yaw / 2)
        self.cmd[d].publish(p)

    def republish(self):
        # keep re-sending targets so late pub/sub discovery can't drop a command
        for d in DRONES:
            self._send(d)

    def arrived(self, d):
        if self.pos[d] is None or self.target[d] is None:
            return False
        return math.dist(self.pos[d], self.target[d]) < ARRIVE_TOL

    def all_arrived(self, ds):
        return all(self.arrived(d) for d in ds)

    def enter(self, phase):
        self.phase = phase
        self.phase_t = self.now()
        self.entered = False          # entry action not yet run for this phase
        self.get_logger().info(f"--- phase: {phase} ---")

    def first(self):
        """True exactly once per phase, on the first tick after enter()."""
        if not self.entered:
            self.entered = True
            return True
        return False

    # -- detection --------------------------------------------------------
    def _image(self, msg):
        if not self.detect_on or self.detected or not HAVE_CV2:
            return
        if self.scan_i < MIN_SCAN_WP:      # scanner not close enough yet
            return
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        hsv = cv2.cvtColor(a[:, :, :3], cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, ORANGE_LO, ORANGE_HI)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area = max((cv2.contourArea(c) for c in cnts), default=0.0)
        self.last_blob = area
        self.hits = self.hits + 1 if area >= MIN_BLOB_AREA else max(0, self.hits - 1)
        if self.hits >= CONFIRM_FRAMES:
            self.detected = True
            self.get_logger().info(
                f"orange contour confirmed ({area:.0f}px, {self.hits} frames)")

    # -- state machine ---------------------------------------------------
    def _tick(self):
        if self.done:
            return
        self.republish()
        t = self.now() - self.phase_t
        p = self.phase

        if p == "WAIT":
            # wait until we have a pose from every drone (controller is live)
            if all(self.pos[d] is not None for d in DRONES) or self.now() - self.t0 > 25:
                self.enter("LAUNCH")

        elif p == "LAUNCH":
            if self.first():
                self.get_logger().info("six drones lifting off from the base station")
                for i, d in enumerate(DRONES):
                    self.go(d, base_slot(i))
            elif t > 3 and (self.all_arrived(DRONES) or t > 35):
                self.enter("FANOUT")

        elif p == "FANOUT":
            if self.first():
                self.get_logger().info("swarm fanning out toward the damaged building")
                for d in DRONES:
                    self.go(d, FAN[d])
            elif t > 3 and (self.all_arrived(DRONES) or t > 55):
                self.enter("APPROACH")

        elif p == "APPROACH":
            if self.first():
                self.get_logger().info(
                    f"{SCANNER} breaking formation to sweep the east facade")
                self.scan_i = 0
                self.go(SCANNER, APPROACH_POS)
            elif t > 3 and (self.arrived(SCANNER) or t > 45):
                self.enter("SCAN")

        elif p == "SCAN":
            self.detect_on = True
            if self.first():
                self.get_logger().info(f"{SCANNER} running scan pattern, camera live")
                self.scan_start = self.now()
                self.wp_t = self.now()
                self.go(SCANNER, SCAN_PATH[self.scan_i])
            if self.detected:
                self.enter("ALERT")
                return
            if self.arrived(SCANNER) and self.now() - self.wp_t > SCAN_DWELL:
                self.scan_i = self.scan_i + 1 if self.scan_i + 1 < len(SCAN_PATH) else 6
                self.go(SCANNER, SCAN_PATH[self.scan_i])
                self.wp_t = self.now()
            if self.now() - self.scan_start > 240:
                self.get_logger().warn("scan timed out with no detection")
                self.enter("RTB")

        elif p == "ALERT":
            if self.first():
                self._fire_alert()
            elif t > 6:
                self.enter("HOLD" if self.hold else "RTB")

        elif p == "HOLD":
            if self.first():
                self.get_logger().info(
                    "holding station over the victim - Ctrl-C to end the demo")
                self.go(SCANNER, HOLD_POS)
            # keep spinning forever; nothing shuts down

        elif p == "RTB":
            if self.first():
                self.get_logger().info("objective met - drones returning to base")
                for i, d in enumerate(DRONES):
                    self.go(d, base_slot(i))
            elif t > 3 and (self.all_arrived(DRONES) or t > 40):
                self._finish()

    def _fire_alert(self):
        self.detect_on = False
        method = "OpenCV orange-contour" if HAVE_CV2 else "pose-gated"
        txt = (f"VICTIM DETECTED by {SCANNER} | "
               f"position x={VICTIM[0]:.1f} y={VICTIM[1]:.1f} z={VICTIM[2]:.1f} | "
               f"method={method} blob_px={self.last_blob:.0f}")
        self.alert_pub.publish(String(data=txt))

        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "alert", 0
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position = Point(x=VICTIM[0], y=VICTIM[1], z=VICTIM[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.5
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 0.9
        self.marker_pub.publish(m)

        bar = "=" * 68
        print(f"\n{bar}\n"
              f"  >>> ALERT  |  VICTIM LOCATED  <<<\n"
              f"  detecting drone : {SCANNER}\n"
              f"  victim position : x={VICTIM[0]:.1f}  y={VICTIM[1]:.1f}  z={VICTIM[2]:.1f}  (world)\n"
              f"  detection       : {method}  (blob {self.last_blob:.0f}px)\n"
              f"  published to    : /base_station/alerts\n"
              f"{bar}\n", flush=True)
        self.get_logger().info("ALERT published to /base_station/alerts")

    def _finish(self):
        self.done = True
        print("\n  MISSION COMPLETE - shutting down.\n", flush=True)
        self.get_logger().info("mission complete")


def main():
    rclpy.init()
    node = Mission()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
