"""Put the sensors to sleep while Bagheera idles in the dock.

In the dock the map pose is fixed, so nothing has to be observed there. After
``sleep_delay_s`` without motion commands the node switches the LiDAR off
(its driver process runs under this node), and the WT901, the PMW3901 and the
camera follow the latched ``/dock/sleep_state``. The costmaps' obstacle layers
are disabled meanwhile; otherwise they warn several times a second that /scan
is stale.

A request on ``/dock/wake`` (sent by the autonomy dock guard or the teleop
node, or by hand) wakes everything in this order:

1. start the LiDAR; WT901 and PMW3901 resume (the WT901 re-measures its gyro
   bias, the robot is guaranteed to stand still in the dock);
2. wait for a steady ``/scan`` and for fresh IMU and optical-flow data, then
   enable the obstacle layers again (confirmed, otherwise no undocking);
3. reset the EKF at its current pose (clears any velocity left from before);
4. ask ``bagheera_pose_persistence`` to anchor AMCL to the dock pose again and
   wait for AMCL to confirm it.

Only then the state becomes ``awake`` and the guard starts the undock
manoeuvre. A failed step ends in ``fault``: sensors stay on, autonomy stays
gated, and the next wake request retries.
"""

from __future__ import annotations

from collections import deque
import signal
import subprocess
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, TwistWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String, UInt32


AWAKE = "awake"
SLEEPING = "sleeping"
WAKING = "waking"
ANCHORING = "anchoring"
FAULT = "fault"


def scan_is_steady(stamps: deque[float], now: float, window_s: float, min_rate_hz: float) -> bool:
    """True when the scans of the last window arrive at least at min_rate_hz."""
    while stamps and now - stamps[0] > window_s:
        stamps.popleft()
    return len(stamps) >= int(window_s * min_rate_hz)


def _command_is_active(message: TwistStamped) -> bool:
    twist = message.twist
    return abs(twist.linear.x) > 1e-3 or abs(twist.angular.z) > 1e-3


class DockSleep(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_dock_sleep")
        self.declare_parameter("sleep_delay_s", 3.0)
        # After a wake-up the robot stays awake at least this long, so a held
        # goal or the teleop driver has time to actually leave the dock.
        self.declare_parameter("wake_grace_s", 30.0)
        self.declare_parameter("lidar_enabled", True)
        self.declare_parameter(
            "lidar_executable",
            "/bagheera_ws/install/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node",
        )
        self.declare_parameter(
            "lidar_parameters", "/bagheera_ws/install/share/bagheera_base/config/sensors.yaml"
        )
        self.declare_parameter("wt901_enabled", True)
        self.declare_parameter("optical_flow_enabled", True)
        self.declare_parameter("ekf_reset_enabled", True)
        self.declare_parameter("anchor_enabled", True)
        self.declare_parameter("scan_window_s", 2.0)
        self.declare_parameter("scan_min_rate_hz", 8.0)
        self.declare_parameter("sensor_timeout_s", 30.0)
        self.declare_parameter("ekf_settle_s", 0.5)
        self.declare_parameter("anchor_timeout_s", 15.0)
        self.declare_parameter(
            "costmap_nodes", ["/local_costmap/local_costmap", "/global_costmap/global_costmap"]
        )
        # With navigation: the costmap obstacle layers follow the sleep, and
        # nothing sleeps before these lifecycle nodes are active. Nav2 needs
        # map -> base_link to activate, which AMCL only publishes with scans.
        self.declare_parameter("navigation_enabled", True)
        self.declare_parameter("startup_nodes", ["/bt_navigator", "/docking_server"])

        self._sleep_delay = float(self.get_parameter("sleep_delay_s").value)
        self._wake_grace = float(self.get_parameter("wake_grace_s").value)
        self._sleep_not_before = 0.0
        self._lidar_enabled = bool(self.get_parameter("lidar_enabled").value)
        self._wt901_enabled = bool(self.get_parameter("wt901_enabled").value)
        self._flow_enabled = bool(self.get_parameter("optical_flow_enabled").value)
        self._ekf_reset_enabled = bool(self.get_parameter("ekf_reset_enabled").value)
        self._anchor_enabled = bool(self.get_parameter("anchor_enabled").value)
        self._scan_window = float(self.get_parameter("scan_window_s").value)
        self._scan_min_rate = float(self.get_parameter("scan_min_rate_hz").value)
        self._sensor_timeout = float(self.get_parameter("sensor_timeout_s").value)
        self._ekf_settle = float(self.get_parameter("ekf_settle_s").value)
        self._anchor_timeout = float(self.get_parameter("anchor_timeout_s").value)

        self._state = AWAKE
        self._state_since = time.monotonic()
        self._docked = False
        self._dock_active = False
        self._last_activity = time.monotonic()
        self._lidar: subprocess.Popen | None = None
        self._lidar_last_start = 0.0
        self._odom: Odometry | None = None
        self._scan_stamps: deque[float] = deque()
        self._imu_fresh = False
        self._flow_fresh = False
        self._wake_subscriptions: list = []
        self._ekf_reset_at: float | None = None
        # Seeded from the clock: a latched answer from before a restart of
        # this node must never match a new request.
        self._anchor_seq = int(time.time()) & 0x7FFFFFFF
        self._anchor_sent_at = 0.0
        self._layer_clients = (
            [
                AsyncParameterClient(self, str(name))
                for name in self.get_parameter("costmap_nodes").value
            ]
            if bool(self.get_parameter("navigation_enabled").value)
            else []
        )
        self._startup_clients = (
            [
                self.create_client(GetState, f"{str(name).rstrip('/')}/get_state")
                for name in self.get_parameter("startup_nodes").value
            ]
            if bool(self.get_parameter("navigation_enabled").value)
            else []
        )
        self._startup_done = not self._startup_clients
        self._startup_active: set[int] = set()
        self._startup_pending = False
        self._startup_checked_at = 0.0
        # Wanted and confirmed obstacle-layer state; None = unknown.
        self._layers_wanted = True
        self._layers_confirmed: bool | None = None
        self._layers_pending = 0
        self._layers_failed = False
        self._layers_sent_at = 0.0
        self._layers_batch = 0

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(String, "/dock/sleep_state", latched)
        self._anchor_publisher = self.create_publisher(UInt32, "/dock/anchor_request", latched)
        self._set_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/set_pose", 10
        )
        self.create_subscription(Bool, "/docked", self._on_docked, latched)
        self.create_subscription(Bool, "/dock/active", self._on_dock_active, latched)
        self.create_subscription(Bool, "/dock/wake", self._on_wake, 10)
        self.create_subscription(Bool, "/dock/sleep_request", self._on_sleep_request, 10)
        self.create_subscription(UInt32, "/dock/pose_anchored", self._on_anchored, latched)
        self.create_subscription(
            TwistStamped, "/cmd_vel_automatic_raw", self._on_command, 10
        )
        self.create_subscription(TwistStamped, "/cmd_vel_teleop", self._on_command, 10)
        self.create_timer(0.2, self._tick)
        self._publish_state()
        self._ensure_lidar()
        self.get_logger().info(
            "Dock sleep ready: sensors sleep after %.0f s idle in the dock" % self._sleep_delay
        )

    # -- inputs ---------------------------------------------------------------

    def _on_docked(self, message: Bool) -> None:
        docked = bool(message.data)
        if docked == self._docked:
            return
        self._docked = docked
        self._last_activity = time.monotonic()
        if not docked and self._state != AWAKE:
            # Pushed or driven out by hand: nothing to anchor any more.
            self._drop_wake_subscriptions()
            self._set_state(AWAKE, "left the dock")
            self._want_layers(True)
            self._ensure_lidar()

    def _on_dock_active(self, message: Bool) -> None:
        # bagheera_dock_trigger repeats this every second: only a change or
        # an ongoing docking counts as activity.
        active = bool(message.data)
        if active or active != self._dock_active:
            self._last_activity = time.monotonic()
        self._dock_active = active

    def _on_command(self, message: TwistStamped) -> None:
        if _command_is_active(message):
            self._last_activity = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message

    def _on_wake(self, message: Bool) -> None:
        if not message.data:
            return
        self._last_activity = time.monotonic()
        if self._state in (SLEEPING, FAULT):
            self._start_wake("wake request")

    def _on_sleep_request(self, message: Bool) -> None:
        if message.data:
            if self._state == AWAKE and self._docked and not self._dock_active:
                self._sleep("sleep request")
        elif self._state in (SLEEPING, FAULT):
            self._start_wake("sleep request withdrawn")

    def _on_anchored(self, message: UInt32) -> None:
        if self._state == ANCHORING and message.data == self._anchor_seq:
            self._set_state(AWAKE, "sensors running, AMCL anchored to the dock pose")
            self._woke_up(time.monotonic())

    def _woke_up(self, now: float) -> None:
        self._last_activity = now
        self._sleep_not_before = now + self._wake_grace

    def _on_scan(self, _message: LaserScan) -> None:
        self._scan_stamps.append(time.monotonic())

    def _on_imu(self, _message: Imu) -> None:
        self._imu_fresh = True

    def _on_flow(self, _message: TwistWithCovarianceStamped) -> None:
        self._flow_fresh = True

    # -- transitions ----------------------------------------------------------

    def _set_state(self, state: str, reason: str) -> None:
        if state == self._state:
            return
        self._state = state
        self._state_since = time.monotonic()
        self._publish_state()
        # rclpy refuses different severities from one call site.
        if state == FAULT:
            self.get_logger().error(f"Dock sleep state {state}: {reason}")
        else:
            self.get_logger().info(f"Dock sleep state {state}: {reason}")

    def _publish_state(self) -> None:
        self._state_publisher.publish(String(data=self._state))

    def _sleep(self, reason: str) -> None:
        self._drop_wake_subscriptions()
        self._set_state(SLEEPING, reason)
        self._stop_lidar()
        self._want_layers(False)

    def _start_wake(self, reason: str) -> None:
        self._drop_wake_subscriptions()
        self._scan_stamps.clear()
        self._imu_fresh = not self._wt901_enabled
        self._flow_fresh = not self._flow_enabled
        self._ekf_reset_at = None
        self._odom = None
        # Monitor the sensors only while waking; /scan and the IMU are too
        # frequent to deserialize in Python all the time.
        if self._lidar_enabled:
            self._wake_subscriptions.append(
                self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
            )
        if self._wt901_enabled:
            self._wake_subscriptions.append(
                self.create_subscription(
                    Imu, "/imu/wt901/data_raw", self._on_imu, qos_profile_sensor_data
                )
            )
        if self._flow_enabled:
            self._wake_subscriptions.append(
                self.create_subscription(
                    TwistWithCovarianceStamped, "/optical_flow/twist", self._on_flow, 10
                )
            )
        if self._ekf_reset_enabled:
            self._wake_subscriptions.append(
                self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 5)
            )
        self._set_state(WAKING, reason)
        self._ensure_lidar()

    def _drop_wake_subscriptions(self) -> None:
        for subscription in self._wake_subscriptions:
            self.destroy_subscription(subscription)
        self._wake_subscriptions = []

    def _fail(self, reason: str) -> None:
        self._drop_wake_subscriptions()
        self._want_layers(True)
        self._set_state(FAULT, f"{reason}; autonomy stays gated, next wake request retries")

    # -- LiDAR process --------------------------------------------------------

    def _ensure_lidar(self) -> None:
        if not self._lidar_enabled or self._state == SLEEPING:
            return
        if self._lidar is not None and self._lidar.poll() is None:
            return
        if time.monotonic() - self._lidar_last_start < 2.0:
            return
        if self._lidar is not None:
            self.get_logger().error(
                f"LiDAR driver exited with code {self._lidar.returncode}; restarting"
            )
        command = [
            str(self.get_parameter("lidar_executable").value),
            "--ros-args",
            "-r", "__node:=ydlidar_ros2_driver_node",
            "--params-file", str(self.get_parameter("lidar_parameters").value),
        ]
        self._lidar_last_start = time.monotonic()
        try:
            self._lidar = subprocess.Popen(command)
        except OSError as error:
            self.get_logger().error(f"Cannot start LiDAR driver: {error}")
            return
        self.get_logger().info("LiDAR driver started")

    def _stop_lidar(self) -> None:
        process = self._lidar
        self._lidar = None
        if process is None:
            return
        if process.poll() is None:
            # SIGINT lets the driver send its stop command (motor off) and
            # close the port cleanly.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        self.get_logger().info("LiDAR driver stopped")

    # -- startup --------------------------------------------------------------

    def _check_startup(self, now: float) -> None:
        """Poll the lifecycle states until all startup nodes are active."""
        if self._startup_done or self._startup_pending or now - self._startup_checked_at < 1.0:
            return
        self._startup_checked_at = now
        for index, client in enumerate(self._startup_clients):
            if index in self._startup_active or not client.service_is_ready():
                continue
            self._startup_pending = True
            client.call_async(GetState.Request()).add_done_callback(
                lambda future, index=index: self._on_startup_state(future, index)
            )

    def _on_startup_state(self, future, index: int) -> None:
        self._startup_pending = False
        response = future.result()
        if response is not None and response.current_state.id == State.PRIMARY_STATE_ACTIVE:
            self._startup_active.add(index)
        if len(self._startup_active) == len(self._startup_clients):
            self._startup_done = True
            self._last_activity = time.monotonic()
            self.get_logger().info("Navigation is active; dock sleep enabled")

    # -- costmap obstacle layers ----------------------------------------------

    def _want_layers(self, enabled: bool) -> None:
        if enabled != self._layers_wanted:
            self._layers_wanted = enabled
            self._layers_confirmed = None
        self._sync_layers()

    def _layers_ready(self) -> bool:
        return not self._layer_clients or self._layers_confirmed is True

    def _sync_layers(self) -> None:
        if not self._layer_clients or self._layers_confirmed == self._layers_wanted:
            return
        if self._layers_pending and time.monotonic() - self._layers_sent_at < 5.0:
            return
        if not all(client.services_are_ready() for client in self._layer_clients):
            return
        wanted = self._layers_wanted
        self._layers_batch += 1
        batch = self._layers_batch
        self._layers_pending = len(self._layer_clients)
        self._layers_failed = False
        self._layers_sent_at = time.monotonic()
        parameter = Parameter("obstacle_layer.enabled", Parameter.Type.BOOL, wanted)
        for client in self._layer_clients:
            future = client.set_parameters([parameter])
            future.add_done_callback(
                lambda done, wanted=wanted, batch=batch: self._on_layers_set(
                    done, wanted, batch
                )
            )

    def _on_layers_set(self, future, wanted: bool, batch: int) -> None:
        if batch != self._layers_batch:
            return
        response = future.result()
        if response is None or not all(result.successful for result in response.results):
            self._layers_failed = True
        self._layers_pending -= 1
        if self._layers_pending > 0 or wanted != self._layers_wanted:
            return
        if self._layers_failed:
            self.get_logger().error("Cannot switch the costmap obstacle layers; retrying")
            return
        self._layers_confirmed = wanted
        self.get_logger().info(
            "Costmap obstacle layers %s" % ("enabled" if wanted else "disabled while asleep")
        )

    # -- main loop ------------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()
        self._sync_layers()
        if self._state == AWAKE:
            self._ensure_lidar()
            self._check_startup(now)
            if (
                self._startup_done
                and self._docked
                and not self._dock_active
                and now - self._last_activity >= self._sleep_delay
                and now >= self._sleep_not_before
            ):
                self._sleep(f"docked and idle for {self._sleep_delay:.0f} s")
            return
        if self._state == FAULT:
            self._ensure_lidar()
            return
        if self._state == WAKING:
            self._ensure_lidar()
            self._tick_waking(now)
            return
        if self._state == ANCHORING:
            if now - self._state_since > self._anchor_timeout:
                self._fail("AMCL did not confirm the dock pose")
            elif now - self._anchor_sent_at >= 3.0:
                # Latched, but repeat in case pose_persistence restarted.
                self._anchor_publisher.publish(UInt32(data=self._anchor_seq))
                self._anchor_sent_at = now

    def _tick_waking(self, now: float) -> None:
        scan_ok = not self._lidar_enabled or scan_is_steady(
            self._scan_stamps, now, self._scan_window, self._scan_min_rate
        )
        if not (scan_ok and self._imu_fresh and self._flow_fresh):
            if now - self._state_since > self._sensor_timeout:
                missing = [
                    name
                    for name, ok in (
                        ("scan", scan_ok),
                        ("WT901", self._imu_fresh),
                        ("optical flow", self._flow_fresh),
                    )
                    if not ok
                ]
                self._fail(f"no data from {', '.join(missing)}")
            return
        # Scans are back: let the costmaps see them before anything moves.
        self._want_layers(True)
        if not self._layers_ready():
            if now - self._state_since > self._sensor_timeout:
                self._fail("costmap obstacle layers could not be enabled")
            return
        if self._ekf_reset_enabled and self._ekf_reset_at is None:
            if not self._reset_ekf():
                if now - self._state_since > self._sensor_timeout:
                    self._fail("no /odometry/filtered for the EKF reset")
                return
            self._ekf_reset_at = now
            return
        if self._ekf_reset_at is not None and now - self._ekf_reset_at < self._ekf_settle:
            return
        self._drop_wake_subscriptions()
        if not self._anchor_enabled or not self._docked:
            self._set_state(AWAKE, "sensors running")
            self._woke_up(now)
            return
        self._anchor_seq += 1
        self._anchor_publisher.publish(UInt32(data=self._anchor_seq))
        self._anchor_sent_at = now
        self._set_state(ANCHORING, "sensors running, anchoring AMCL to the dock pose")

    def _reset_ekf(self) -> bool:
        """Restart the EKF at its current pose with zero velocity."""
        odom = self._odom
        if odom is None:
            return False
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = odom.header.frame_id or "odom"
        message.pose.pose = odom.pose.pose
        message.pose.covariance[0] = 1e-3
        message.pose.covariance[7] = 1e-3
        message.pose.covariance[14] = 1e-6
        message.pose.covariance[21] = 1e-6
        message.pose.covariance[28] = 1e-6
        message.pose.covariance[35] = 1e-3
        self._set_pose_publisher.publish(message)
        self.get_logger().info("EKF reset at its current pose")
        return True

    def destroy_node(self) -> bool:
        self._stop_lidar()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockSleep()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
