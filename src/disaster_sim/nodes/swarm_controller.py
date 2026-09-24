#!/usr/bin/env python3
"""Kinematic hover controller for the drone swarm.

Holds the authoritative pose of every drone, flies each one toward its
commanded target at a capped speed (no flight dynamics - this is a
visualisation of an already-evaluated decision engine), and pushes the
result into Gazebo Classic via /gazebo/set_entity_state.

Per drone <ns> (drone_0 .. drone_5):
  sub  <ns>/cmd_pose   geometry_msgs/Pose        "fly here"
  pub  <ns>/pose       geometry_msgs/PoseStamped current pose  (30 Hz)
  pub  <ns>/at_goal    std_msgs/Bool             within tolerance of cmd_pose
  tf   world -> <ns>/base_link -> <ns>/camera_link
Also: pub /swarm/markers  visualization_msgs/MarkerArray  (one sphere per drone)
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Quaternion
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.srv import SetEntityState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# per-drone marker colours (index order = drones param order)
PALETTE = [(0.95, 0.15, 0.10), (0.98, 0.55, 0.05), (0.95, 0.90, 0.12),
           (0.15, 0.85, 0.22), (0.12, 0.78, 0.92), (0.90, 0.18, 0.80)]

# camera sensor pose in base_link (matches description/drone.sdf.template)
CAM_XYZ = (0.80, 0.0, 0.05)
CAM_PITCH = 0.72


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quat_mul_pitch_yaw(pitch, yaw):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return Quaternion(x=-sp * sy, y=sp * cy, z=cp * sy, w=cp * cy)


class DroneState:
    def __init__(self, x, y, z, yaw):
        self.x, self.y, self.z, self.yaw = x, y, z, yaw
        self.tx, self.ty, self.tz = x, y, z          # target
        self.tyaw = yaw
        self.have_yaw_cmd = False


class SwarmController(Node):
    def __init__(self):
        super().__init__("swarm_controller")
        self.declare_parameter("drones", ["drone_0"])
        self.declare_parameter("init_x", [0.0])
        self.declare_parameter("init_y", [0.0])
        self.declare_parameter("init_z", [0.0])
        self.declare_parameter("init_yaw", [0.0])
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("max_speed", 5.0)
        self.declare_parameter("max_climb", 3.0)
        self.declare_parameter("yaw_rate", 1.5)
        self.declare_parameter("pos_tol", 0.6)

        names = self.get_parameter("drones").value
        ix = self.get_parameter("init_x").value
        iy = self.get_parameter("init_y").value
        iz = self.get_parameter("init_z").value
        iyaw = self.get_parameter("init_yaw").value
        self.rate = self.get_parameter("rate").value
        self.max_speed = self.get_parameter("max_speed").value
        self.max_climb = self.get_parameter("max_climb").value
        self.yaw_rate = self.get_parameter("yaw_rate").value
        self.pos_tol = self.get_parameter("pos_tol").value

        self.drones = {}
        self.pose_pub = {}
        self.goal_pub = {}
        self.color = {}
        for i, n in enumerate(names):
            st = DroneState(ix[i], iy[i], iz[i], iyaw[i])
            self.drones[n] = st
            self.color[n] = PALETTE[i % len(PALETTE)]
            self.pose_pub[n] = self.create_publisher(PoseStamped, f"{n}/pose", 10)
            self.goal_pub[n] = self.create_publisher(Bool, f"{n}/at_goal", 10)
            self.create_subscription(Pose, f"{n}/cmd_pose",
                                     lambda m, nn=n: self._on_cmd(nn, m), 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/swarm/markers", 10)

        self.cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.get_logger().info("waiting for /gazebo/set_entity_state ...")
        self.cli.wait_for_service()
        self.get_logger().info(f"controlling {list(self.drones)}")

        self.tf = TransformBroadcaster(self)
        stf = StaticTransformBroadcaster(self)
        stf.sendTransform([self._cam_tf(n) for n in self.drones])

        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self._tick)

    def _on_cmd(self, n, msg: Pose):
        st = self.drones[n]
        st.tx, st.ty, st.tz = msg.position.x, msg.position.y, msg.position.z
        # use commanded yaw only if a real orientation was given
        q = msg.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) > 1e-6:
            st.tyaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
            st.have_yaw_cmd = True
        else:
            st.have_yaw_cmd = False

    def _cam_tf(self, n):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f"{n}/base_link"
        t.child_frame_id = f"{n}/camera_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = CAM_XYZ
        t.transform.rotation = quat_mul_pitch_yaw(CAM_PITCH, 0.0)
        return t

    def _tick(self):
        now = self.get_clock().now().to_msg()
        marr = MarkerArray()
        for n, st in self.drones.items():
            dx, dy, dz = st.tx - st.x, st.ty - st.y, st.tz - st.z
            dist = math.hypot(dx, dy)
            step = self.max_speed * self.dt
            if dist > step:
                st.x += dx / dist * step
                st.y += dy / dist * step
            else:
                st.x, st.y = st.tx, st.ty
            cstep = self.max_climb * self.dt
            st.z += max(-cstep, min(cstep, dz))

            # desired yaw: commanded, else face direction of travel
            if st.have_yaw_cmd:
                des = st.tyaw
            elif dist > 1.0:
                des = math.atan2(dy, dx)
            else:
                des = st.yaw
            err = math.atan2(math.sin(des - st.yaw), math.cos(des - st.yaw))
            ystep = self.yaw_rate * self.dt
            st.yaw += max(-ystep, min(ystep, err))

            self._push(n, st)

            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = "world"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = st.x, st.y, st.z
            ps.pose.orientation = yaw_to_quat(st.yaw)
            self.pose_pub[n].publish(ps)

            at = (math.hypot(st.tx - st.x, st.ty - st.y) < self.pos_tol
                  and abs(st.tz - st.z) < self.pos_tol)
            self.goal_pub[n].publish(Bool(data=at))

            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "world"
            tf.child_frame_id = f"{n}/base_link"
            tf.transform.translation.x = st.x
            tf.transform.translation.y = st.y
            tf.transform.translation.z = st.z
            tf.transform.rotation = yaw_to_quat(st.yaw)
            self.tf.sendTransform(tf)

            r, g, b = self.color[n]
            mk = Marker()
            mk.header.frame_id = "world"
            mk.header.stamp = now
            mk.ns = "drones"
            mk.id = list(self.drones).index(n)
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = st.x, st.y, st.z
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 1.4
            mk.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
            marr.markers.append(mk)
            lbl = Marker()
            lbl.header.frame_id = "world"
            lbl.header.stamp = now
            lbl.ns = "drone_labels"
            lbl.id = mk.id
            lbl.type = Marker.TEXT_VIEW_FACING
            lbl.action = Marker.ADD
            lbl.pose.position.x, lbl.pose.position.y = st.x, st.y
            lbl.pose.position.z = st.z + 1.4
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = 1.2
            lbl.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            lbl.text = n
            marr.markers.append(lbl)

        self.marker_pub.publish(marr)

    def _push(self, n, st):
        req = SetEntityState.Request()
        req.state.name = n
        req.state.reference_frame = "world"
        req.state.pose.position.x = st.x
        req.state.pose.position.y = st.y
        req.state.pose.position.z = st.z
        req.state.pose.orientation = yaw_to_quat(st.yaw)
        # zero the twist every tick: kinematic control, no residual tumble
        req.state.twist.linear.x = req.state.twist.linear.y = req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = req.state.twist.angular.y = req.state.twist.angular.z = 0.0
        self.cli.call_async(req)


def main():
    try:
        rclpy.init()
        node = SwarmController()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
