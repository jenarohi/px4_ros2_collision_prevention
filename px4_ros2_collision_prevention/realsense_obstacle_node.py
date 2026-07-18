#!/usr/bin/env python3
"""
RealSense D435/D435i → PX4 Collision Prevention (ROS 2 Humble)
==============================================================

Architecture
------------
This node is the ONLY component you need on the companion computer side.
It reads depth frames from a RealSense D435/D435i camera and publishes
px4_msgs/ObstacleDistance to /fmu/in/obstacle_distance at ~15 Hz.

The uXRCE-DDS bridge (MicroXRCEAgent) running alongside transparently
forwards this ROS2 topic into PX4's internal obstacle_distance uORB topic.
PX4's built-in Collision Prevention (CP) module then brakes the vehicle
in Position mode whenever an obstacle enters the CP_DIST envelope.

Data Flow
---------
RealSense D435 (USB3)
  └─► pyrealsense2 depth frame
        └─► depth_frame_to_obstacle_bins()   [meters → 72-bin uint16 cm array]
              └─► px4_msgs/ObstacleDistance
                    └─► /fmu/in/obstacle_distance  (ROS 2 topic, 15 Hz)
                          └─► MicroXRCEAgent  (uXRCE-DDS bridge)
                                └─► PX4 obstacle_distance uORB
                                      └─► Collision Prevention module
                                            └─► Drone brakes in Position mode ✓

ObstacleDistance message format
--------------------------------
  distances[72]  : uint16 array, one per 5° sector, value in cm
                   UINT16_MAX (65535) = unknown (no obstacle data)
  frame          : 12 = MAV_FRAME_BODY_FRD  (rotates with vehicle heading)
  increment      : 5.0 degrees per bin
  angle_offset   : -43.5° (left edge of D435 FOV → index 0)
  min_distance   : 20 cm
  max_distance   : 1000 cm (10 m)

PX4 Parameters Required (set in QGroundControl)
-----------------------------------------------
  CP_DIST   = 3.0   # metres — activates Collision Prevention
  CP_DELAY  = 0.5   # sensor + actuator delay estimate (seconds)
  MPC_POS_MODE = 0  # acceleration-based position control (required)

Usage
-----
  # Build first (in ~/ros2_ws):
  colcon build --packages-select px4_msgs px4_ros2_collision_prevention
  source install/setup.bash

  # Real hardware:
  ros2 run px4_ros2_collision_prevention realsense_obstacle_node

  # SITL (no camera, synthetic obstacle at 2 m forward):
  ros2 run px4_ros2_collision_prevention realsense_obstacle_node \
    --ros-args -p simulate_depth:=true

  # With custom parameters:
  ros2 run px4_ros2_collision_prevention realsense_obstacle_node \
    --ros-args -p publish_hz:=10.0 -p max_depth_m:=8.0
"""

import math
import time
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import ObstacleDistance

# ── Constants matching MAVLink / PX4 expectations ────────────────────────────
UINT16_MAX = 65535          # Unknown / out-of-range sentinel
MAV_FRAME_BODY_FRD = 12     # Required frame for Collision Prevention
MAV_DISTANCE_SENSOR_LASER = 0


class RealSenseObstacleNode(Node):
    """
    ROS 2 node: RealSense D435 depth → ObstacleDistance → /fmu/in/obstacle_distance

    Designed for PX4 >= v1.14 with uXRCE-DDS middleware.
    No MAVROS, no pymavlink, no MAVLink socket — just native ROS2.
    """

    def __init__(self):
        super().__init__('realsense_obstacle_node')

        # ── Declare parameters (can be overridden at launch) ─────────────────
        self.declare_parameter('publish_hz',    15.0)   # Hz
        self.declare_parameter('min_depth_m',   0.20)   # metres
        self.declare_parameter('max_depth_m',   10.0)   # metres
        self.declare_parameter('fov_h_deg',     87.0)   # D435 horizontal FOV
        self.declare_parameter('num_bins',      72)     # 360/5 = 72 bins
        self.declare_parameter('use_filters',   True)   # RealSense post-proc
        self.declare_parameter('simulate_depth',False)  # SITL synthetic mode
        self.declare_parameter('depth_width',   640)
        self.declare_parameter('depth_height',  480)
        self.declare_parameter('depth_fps',     30)

        # Read parameters
        self._hz          = self.get_parameter('publish_hz').value
        self._min_m       = self.get_parameter('min_depth_m').value
        self._max_m       = self.get_parameter('max_depth_m').value
        self._fov_h       = self.get_parameter('fov_h_deg').value
        self._num_bins    = self.get_parameter('num_bins').value
        self._use_filters = self.get_parameter('use_filters').value
        self._simulate    = self.get_parameter('simulate_depth').value
        self._width       = self.get_parameter('depth_width').value
        self._height      = self.get_parameter('depth_height').value
        self._fps         = self.get_parameter('depth_fps').value

        # Derived geometry
        self._increment_deg = 360.0 / self._num_bins          # 5.0°
        self._angle_offset  = -self._fov_h / 2.0              # −43.5°

        # ── QoS: PX4 expects BEST_EFFORT on inbound topics ───────────────────
        # Using BEST_EFFORT on the publisher ensures the uXRCE-DDS bridge
        # doesn't drop messages due to QoS mismatch.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            ObstacleDistance,
            '/fmu/in/obstacle_distance',
            qos,
        )

        # ── State ─────────────────────────────────────────────────────────────
        self._pipeline    = None
        self._depth_scale = 0.001   # D435 default; overridden by firmware read
        self._filters     = []
        self._watchdog_ok = False   # True when camera is streaming

        # ── Camera init ───────────────────────────────────────────────────────
        if self._simulate:
            self.get_logger().warn(
                '⚠️  simulate_depth=true — injecting synthetic obstacle at 2 m '
                'forward. Use this mode ONLY for SITL testing.'
            )
        else:
            self._init_realsense()

        # ── Timer: publish at requested Hz ───────────────────────────────────
        self._timer = self.create_timer(1.0 / self._hz, self._timer_callback)

        self.get_logger().info(
            f'✅ realsense_obstacle_node started\n'
            f'   publish_hz     : {self._hz} Hz\n'
            f'   depth range    : {self._min_m}–{self._max_m} m\n'
            f'   bins           : {self._num_bins} × {self._increment_deg}°\n'
            f'   FOV            : {self._fov_h}° (offset {self._angle_offset}°)\n'
            f'   simulate_depth : {self._simulate}\n'
            f'   Publishing to  : /fmu/in/obstacle_distance\n'
            f'   ──────────────────────────────────────────\n'
            f'   PX4 param needed → CP_DIST > 0  (e.g. 3.0 m)\n'
            f'   Fly in POSITION MODE to activate braking'
        )

    # ── RealSense init ────────────────────────────────────────────────────────
    def _init_realsense(self):
        """Start the depth pipeline. Exits node if camera is not found."""
        try:
            import pyrealsense2 as rs
            self._rs = rs
        except ImportError:
            self.get_logger().fatal(
                '❌ pyrealsense2 not found. Run: pip install pyrealsense2\n'
                '   Or use --ros-args -p simulate_depth:=true for SITL.'
            )
            rclpy.shutdown()
            return

        try:
            self._pipeline = self._rs.pipeline()
            cfg = self._rs.config()
            cfg.enable_stream(
                self._rs.stream.depth,
                self._width, self._height,
                self._rs.format.z16,
                self._fps,
            )
            profile = self._pipeline.start(cfg)

            # Read actual depth scale from firmware (more reliable than hardcoding)
            sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = sensor.get_depth_scale()
            self.get_logger().info(
                f'📷 RealSense started | depth_scale={self._depth_scale:.6f} m/unit'
            )

            # Build post-processing filter chain
            if self._use_filters:
                self._filters = [
                    self._rs.decimation_filter(),
                    self._rs.threshold_filter(),
                    self._rs.disparity_transform(True),
                    self._rs.spatial_filter(),
                    self._rs.temporal_filter(),
                    self._rs.disparity_transform(False),
                ]

            self._watchdog_ok = True

        except Exception as e:
            self.get_logger().fatal(
                f'❌ Failed to start RealSense pipeline: {e}\n'
                f'   Check USB 3.0 connection and run: rs-enumerate-devices'
            )
            rclpy.shutdown()

    # ── Timer callback: grab frame → bins → publish ───────────────────────────
    def _timer_callback(self):
        if self._simulate:
            distances_cm = self._synthetic_obstacle()
        else:
            distances_cm = self._grab_and_process()
            if distances_cm is None:
                return  # sensor issue — skip this cycle

        self._publish(distances_cm)

    # ── Frame grab ───────────────────────────────────────────────────────────
    def _grab_and_process(self):
        """
        Pull one depth frame from the camera and convert to the 72-bin array.
        Returns None on any camera error (caller skips publication).
        """
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=200)
        except Exception as e:
            self.get_logger().warn(f'⚠️  Frame timeout: {e}', throttle_duration_sec=5.0)
            self._watchdog_ok = False
            return None

        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            self.get_logger().warn('⚠️  No depth frame', throttle_duration_sec=5.0)
            return None

        self._watchdog_ok = True
        return self._depth_frame_to_bins(depth_frame)

    # ── Core conversion: depth frame → 72-bin distance array (cm) ─────────────
    def _depth_frame_to_bins(self, depth_frame):
        """
        Convert a RealSense depth frame into a 72-element uint16 array (cm).

        Algorithm:
          1. Apply post-processing filters (decimation → spatial → temporal)
          2. Sample the middle row (horizontal plane)
          3. Map each pixel's angle to one of the 72 angular bins
          4. Keep the MINIMUM distance in each bin (most conservative = safest)
          5. Clamp to [min_depth, max_depth]; unknown → UINT16_MAX

        The result is in the MAV_FRAME_BODY_FRD frame:
          Index 0 = angle_offset (−43.5° for D435, i.e. left edge of FOV)
          Index increases clockwise
          Bins outside the camera FOV remain UINT16_MAX (unknown)

        Returns: np.ndarray shape (72,) dtype uint16
        """
        if self._use_filters:
            for f in self._filters:
                depth_frame = f.process(depth_frame)

        depth_image = np.asanyarray(depth_frame.get_data())

        # Sample the middle row → horizontal distances only
        mid_row = depth_image[depth_image.shape[0] // 2, :]

        # Convert raw units → metres (vectorised, fast)
        row_m = mid_row.astype(np.float32) * self._depth_scale

        # Pixels that read 0 (no return) or beyond sensor limits → treat as max
        invalid_mask = (row_m == 0) | (row_m < self._min_m) | (row_m > self._max_m)
        row_m[invalid_mask] = self._max_m

        # Initialise all bins as UNKNOWN
        bins_cm = np.full(self._num_bins, UINT16_MAX, dtype=np.uint16)

        num_cols = len(row_m)
        fov_start = self._angle_offset   # −43.5°

        # Map each pixel column → angular bin
        # col=0 → fov_start, col=num_cols-1 → fov_start + FOV_H
        col_indices = np.arange(num_cols)
        angles = fov_start + (col_indices / num_cols) * self._fov_h

        # Bin indices (0-indexed, clockwise)
        bin_indices = (np.round(angles % 360 / self._increment_deg)
                       .astype(int) % self._num_bins)

        # Convert to cm (clamped below UINT16_MAX so sentinel is unambiguous)
        dist_cm = np.minimum((row_m * 100).astype(np.uint16), 65534)

        # Scatter-minimum into bins
        # np.minimum.at performs "unbuffered" minimum reduction
        np.minimum.at(bins_cm, bin_indices, dist_cm)

        return bins_cm

    # ── Synthetic obstacle (SITL / no-camera test) ───────────────────────────
    def _synthetic_obstacle(self):
        """
        Inject a wall 2 m ahead (forward bins) so you can verify CP braking
        in SITL without a physical camera attached.

        The synthetic obstacle sweeps a ±30° arc around forward (bin 0 / bin 72).
        """
        bins_cm = np.full(self._num_bins, UINT16_MAX, dtype=np.uint16)

        # Forward direction in FRD frame = 0° → bin index = 0
        # Cover −30° to +30° (12 bins at 5°/bin)
        sweep_bins = 6  # ±30° = 6 bins each side
        obstacle_cm = 200  # 2 m

        for offset in range(-sweep_bins, sweep_bins + 1):
            idx = offset % self._num_bins
            bins_cm[idx] = obstacle_cm

        return bins_cm

    # ── Publish ───────────────────────────────────────────────────────────────
    def _publish(self, distances_cm: np.ndarray):
        """
        Build and publish px4_msgs/ObstacleDistance.

        Field notes:
          timestamp   : microseconds since epoch (PX4 syncs this internally)
          frame       : 12 = MAV_FRAME_BODY_FRD  ← DO NOT change this
          sensor_type : 0  = MAV_DISTANCE_SENSOR_LASER
          distances   : uint16[72] in cm; UINT16_MAX = unknown
          increment   : float32, degrees per bin
          min_distance: uint16, cm
          max_distance: uint16, cm
          angle_offset: float32, degrees (offset of bin[0] from forward)
        """
        msg = ObstacleDistance()
        msg.timestamp     = int(self.get_clock().now().nanoseconds / 1000)  # µs
        msg.frame         = MAV_FRAME_BODY_FRD
        msg.sensor_type   = MAV_DISTANCE_SENSOR_LASER
        msg.distances     = distances_cm.tolist()
        msg.increment     = float(self._increment_deg)
        msg.min_distance  = int(self._min_m * 100)   # cm
        msg.max_distance  = int(self._max_m * 100)   # cm
        msg.angle_offset  = float(self._angle_offset)

        self._pub.publish(msg)

        # ── Debug log (throttled — every 2 s) ────────────────────────────────
        valid = distances_cm[distances_cm < UINT16_MAX]
        if valid.size:
            self.get_logger().debug(
                f'ObstacleDistance | valid bins: {valid.size}/{self._num_bins} | '
                f'min: {valid.min()/100:.2f} m | max: {valid.max()/100:.2f} m',
                throttle_duration_sec=2.0,
            )

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
                self.get_logger().info('🛑 RealSense pipeline stopped.')
            except Exception:
                pass
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RealSenseObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
