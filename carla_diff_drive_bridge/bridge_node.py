#!/usr/bin/env python3
"""
CARLA Differential Drive Bridge Node

Subscribes to /cmd_vel (geometry_msgs/Twist) and applies kinematic control
to a CARLA actor, bypassing the vehicle physics engine entirely.

Publishes:
  - /odom (nav_msgs/Odometry) — ground-truth odometry from CARLA
  - /tf (tf2) — odom -> base_link transform

Coordinate convention:
  ROS:  right-handed, +X forward, +Y left,  +Z up, yaw CCW positive
  CARLA: left-handed, +X forward, +Y right, +Z up, yaw CW positive
  => Invert Y and angular Z when crossing the boundary.
"""

import math
import time
import threading

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    """Convert yaw (radians) to ROS quaternion (only rotation about Z)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class DiffDriveBridge(Node):
    def __init__(self):
        super().__init__('carla_diff_drive_bridge')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('actor_id', -1)          # -1 = auto-find first vehicle
        self.declare_parameter('actor_role_name', 'ego_robot')  # find by role_name attribute
        self.declare_parameter('control_rate', 20.0)     # Hz — tick rate for applying velocity
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cmd_vel_timeout', 0.5)   # seconds — zero velocity if no cmd_vel

        self._carla_host = self.get_parameter('carla_host').value
        self._carla_port = self.get_parameter('carla_port').value
        self._actor_id = self.get_parameter('actor_id').value
        self._role_name = self.get_parameter('actor_role_name').value
        self._control_rate = self.get_parameter('control_rate').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value

        # ── State ───────────────────────────────────────────────────
        self._latest_twist = Twist()  # zero by default
        self._twist_stamp = 0.0
        self._twist_lock = threading.Lock()

        self._actor: carla.Actor | None = None
        self._client: carla.Client | None = None
        self._world: carla.World | None = None

        # Origin for odometry (set on first tick)
        self._origin_set = False
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._origin_yaw = 0.0

        # ── ROS I/O ────────────────────────────────────────────────
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        self._cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self._cmd_vel_cb,
            qos
        )

        self._odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            qos
        )

        self._tf_broadcaster = TransformBroadcaster(self)

        # ── Connect to CARLA ────────────────────────────────────────
        self._connect_carla()

        # ── Control loop timer ──────────────────────────────────────
        period = 1.0 / self._control_rate
        self._timer = self.create_timer(period, self._control_tick)

        self.get_logger().info(
            f'Diff-drive bridge started — CARLA {self._carla_host}:{self._carla_port}, '
            f'actor_id={self._actor.id if self._actor else "pending"}, '
            f'rate={self._control_rate}Hz'
        )

    # ── CARLA connection ────────────────────────────────────────────

    def _connect_carla(self):
        self.get_logger().info(f'Connecting to CARLA at {self._carla_host}:{self._carla_port}…')
        self._client = carla.Client(self._carla_host, self._carla_port)
        self._client.set_timeout(10.0)
        self._world = self._client.get_world()
        self.get_logger().info(f'Connected. Map: {self._world.get_map().name}')

        self._actor = self._find_actor()
        if self._actor is None:
            self.get_logger().warn(
                'No actor found yet. Will retry each tick. '
                'Spawn one with: ros2 run carla_diff_drive_bridge spawn_robot'
            )

    def _find_actor(self) -> carla.Actor | None:
        """Find the target actor by explicit ID or role_name attribute."""
        if self._actor_id >= 0:
            actor = self._world.get_actor(self._actor_id)
            if actor:
                self.get_logger().info(f'Found actor by ID: {actor.id} ({actor.type_id})')
                return actor

        # Search by role_name attribute
        for actor in self._world.get_actors():
            attrs = actor.attributes
            if attrs.get('role_name', '') == self._role_name:
                self.get_logger().info(
                    f'Found actor by role_name "{self._role_name}": '
                    f'id={actor.id} ({actor.type_id})'
                )
                return actor

        return None

    # ── Callbacks ───────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        with self._twist_lock:
            self._latest_twist = msg
            self._twist_stamp = time.monotonic()

    # ── Control loop ────────────────────────────────────────────────

    def _control_tick(self):
        # Retry actor discovery
        if self._actor is None:
            self._actor = self._find_actor()
            if self._actor is None:
                return

        # Check if actor is still alive
        if not self._actor.is_alive:
            self.get_logger().warn('Actor destroyed. Searching again…')
            self._actor = None
            self._origin_set = False
            return

        # Get latest twist (with timeout → zero if stale)
        with self._twist_lock:
            twist = self._latest_twist
            age = time.monotonic() - self._twist_stamp

        if age > self._cmd_vel_timeout and self._twist_stamp > 0:
            twist = Twist()  # zero velocity — safety stop

        # ── Read current CARLA state ────────────────────────────────
        transform = self._actor.get_transform()
        carla_yaw_deg = transform.rotation.yaw
        carla_yaw_rad = math.radians(carla_yaw_deg)

        # Set odometry origin on first valid tick
        if not self._origin_set:
            self._origin_x = transform.location.x
            self._origin_y = transform.location.y
            self._origin_yaw = carla_yaw_rad
            self._origin_set = True
            self.get_logger().info(
                f'Odometry origin set at CARLA ({self._origin_x:.1f}, '
                f'{self._origin_y:.1f}, yaw={math.degrees(self._origin_yaw):.1f}°)'
            )

        # ── Apply cmd_vel → CARLA velocity control ─────────────────
        # Physics must be enabled on the actor for this to work.
        # CARLA handles collisions; we just set the desired velocity each tick.

        # ROS linear.x = forward speed (m/s)
        v_linear = twist.linear.x

        # Project local forward velocity into CARLA world frame
        vx_world = v_linear * math.cos(carla_yaw_rad)
        vy_world = v_linear * math.sin(carla_yaw_rad)

        self._actor.set_target_velocity(carla.Vector3D(x=vx_world, y=vy_world, z=0.0))

        # ROS angular.z (rad/s, CCW+) → CARLA angular Z (deg/s, CW+)
        omega_carla_deg = -math.degrees(twist.angular.z)
        self._actor.set_target_angular_velocity(
            carla.Vector3D(x=0.0, y=0.0, z=omega_carla_deg)
        )

        # ── Publish odometry ────────────────────────────────────────
        self._publish_odom(transform, twist)

    def _publish_odom(self, transform: carla.Transform, twist: Twist):
        now = self.get_clock().now().to_msg()

        # CARLA world → ROS odom frame (relative to origin)
        # CARLA Y is right, ROS Y is left → invert Y
        dx_carla = transform.location.x - self._origin_x
        dy_carla = transform.location.y - self._origin_y

        # Rotate into odom frame (undo origin yaw)
        cos_o = math.cos(-self._origin_yaw)
        sin_o = math.sin(-self._origin_yaw)
        odom_x = dx_carla * cos_o - dy_carla * sin_o
        odom_y = -(dx_carla * sin_o + dy_carla * cos_o)  # invert for ROS

        # Yaw: CARLA CW+ → ROS CCW+
        carla_yaw_rad = math.radians(transform.rotation.yaw)
        ros_yaw = -(carla_yaw_rad - self._origin_yaw)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(ros_yaw)

        # Velocity in child (base_link) frame — just pass through cmd_vel
        odom.twist.twist = twist

        self._odom_pub.publish(odom)

        # TF broadcast
        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = odom_x
            t.transform.translation.y = odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation = yaw_to_quaternion(ros_yaw)
            self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zero out velocity before shutdown
        if node._actor and node._actor.is_alive:
            node._actor.set_target_velocity(carla.Vector3D(0, 0, 0))
            node._actor.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
            node.get_logger().info('Zeroed actor velocity on shutdown.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
