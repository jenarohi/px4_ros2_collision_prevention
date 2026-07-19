#!/usr/bin/env python3
"""
realsense_obstacle_node.py  (v2 — reviewed & fixed)

Fixes vs v1:
  1. Intrinsics-based angle per column: arctan2(col - ppx, fx)  — no linear FOV assumption
  2. 10th-percentile depth per bin instead of absolute min       — noise robust
  3. MIN_PIXELS_PER_BIN guard                                    — reject sparse bins
  4. increment + angle_offset derived from actual intrinsics     — computed at runtime

PX4 note:
  _addObstacleSensorData() reads increment + angle_offset from the incoming message
  and re-maps each bin into PX4's internal 72×5° collision map automatically.
  You do NOT need to pre-arrange bins as 5°/bin — any increment is valid.

Verify after running:
  NuttShell: listener obstacle_distance
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import ObstacleDistance

# ── Constants ─────────────────────────────────────────────────────────────────
BIN_COUNT        = 72       # fixed PX4 message array size
UINT16_MAX       = 65535    # unknown / not covered
NUM_BINS         = 9        # angular sectors across camera FOV
MIN_RANGE_M      = 0.20
MAX_RANGE_M      = 8.00
MIN_CM           = int(MIN_RANGE_M * 100)   # 20
MAX_CM           = int(MAX_RANGE_M * 100)   # 800
CLEAR_CM         = MAX_CM + 1               # 801 = "looked, nothing found" (PX4 spec)
BAND_FRAC        = 0.20     # middle 20% of rows sampled
PUBLISH_HZ       = 10.0
DEPTH_PERCENTILE = 10       # 10th percentile — robust, conservative
MIN_PIXELS_PER_BIN = 5      # bins with fewer valid pixels → CLEAR (not unknown)


class RealSenseObstacleNode(Node):

    def __init__(self):
        super().__init__('realsense_obstacle_node')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(
            ObstacleDistance, '/fmu/in/obstacle_distance', px4_qos
        )

        self._pipeline      = None
        self._depth_scale   = 0.001
        self._filters       = []
        self._col_bin       = None
        self._increment_deg = None
        self._angle_offset  = None
        # Store full-res intrinsics so we can scale after decimation
        self._fx_full = None
        self._ppx_full = None
        self._width_full = None

        self._init_realsense()
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._timer_cb)

    # ── RealSense init ────────────────────────────────────────────────────────
    def _init_realsense(self):
        try:
            import pyrealsense2 as rs
            self._rs = rs
        except ImportError:
            self.get_logger().fatal('pyrealsense2 not installed. pip install pyrealsense2')
            rclpy.shutdown()
            return

        try:
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            profile = pipeline.start(cfg)

            # Depth scale (firmware value — D435i = 0.001)
            sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = sensor.get_depth_scale()

            # Camera intrinsics — needed for accurate angle mapping
            intr = profile.get_stream(rs.stream.depth) \
                          .as_video_stream_profile().get_intrinsics()
            self._fx_full    = intr.fx
            self._ppx_full   = intr.ppx
            self._width_full = intr.width

            self.get_logger().info(
                f'RealSense started | depth_scale={self._depth_scale:.6f} | '
                f'fx={self._fx_full:.1f} ppx={self._ppx_full:.1f} w={self._width_full}'
            )

            self._filters = [
                rs.decimation_filter(),        # halves resolution → width 320
                rs.threshold_filter(),
                rs.disparity_transform(True),
                rs.spatial_filter(),
                rs.temporal_filter(),
                rs.disparity_transform(False),
            ]

            self._pipeline = pipeline
            # Build lookup for full-res first; will rebuild after first decimated frame
            self._build_col_bin_lookup(self._width_full, self._fx_full, self._ppx_full)

        except Exception as exc:
            self.get_logger().fatal(
                f'Failed to start RealSense: {exc}\n'
                f'Check USB 3.0 cable. Run: rs-enumerate-devices'
            )
            rclpy.shutdown()

    # ── Column → bin lookup (intrinsics-based) ────────────────────────────────
    def _build_col_bin_lookup(self, width: int, fx: float, ppx: float):
        """
        For each pixel column x:
            angle = arctan2(x - ppx, fx)   [degrees]

        This is the correct pinhole model — not a linear FOV approximation.
        Negative angles = left of centre, positive = right.

        Bin 0 = leftmost angular slice, Bin NUM_BINS-1 = rightmost.
        increment and angle_offset are derived from actual intrinsics,
        so PX4's _addObstacleSensorData() maps them correctly into its
        internal 72×5° collision map.
        """
        col_idx = np.arange(width)
        angles  = np.degrees(np.arctan2(col_idx - ppx, fx))   # shape (width,)

        lo, hi  = float(angles.min()), float(angles.max())
        edges   = np.linspace(lo, hi, NUM_BINS + 1)

        self._col_bin       = np.clip(np.digitize(angles, edges) - 1, 0, NUM_BINS - 1)
        self._increment_deg = float(edges[1] - edges[0])
        # angle_offset = centre of bin 0 (what PX4 expects)
        self._angle_offset  = float(0.5 * (edges[0] + edges[1]))

        self.get_logger().info(
            f'Bin lookup | w={width} fx={fx:.1f} ppx={ppx:.1f} | '
            f'FOV={hi-lo:.1f}° | {NUM_BINS}×{self._increment_deg:.2f}°/bin | '
            f'angle_offset={self._angle_offset:.2f}°'
        )

    # ── Timer ─────────────────────────────────────────────────────────────────
    def _timer_cb(self):
        if self._pipeline is None or self._col_bin is None:
            return
        distances = self._process_frame()
        if distances is not None:
            self._publish(distances)

    # ── Frame processing ──────────────────────────────────────────────────────
    def _process_frame(self):
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=200)
        except Exception as exc:
            self.get_logger().warn(f'Frame timeout: {exc}', throttle_duration_sec=5.0)
            return None

        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None

        for f in self._filters:
            depth_frame = f.process(depth_frame)

        depth_raw = np.asanyarray(depth_frame.get_data())                   # uint16
        depth_m   = depth_raw.astype(np.float32) * self._depth_scale        # metres

        h, w = depth_m.shape

        # Rebuild lookup if decimation filter changed the width
        if len(self._col_bin) != w:
            scale = w / self._width_full
            self._build_col_bin_lookup(w,
                                       self._fx_full  * scale,
                                       self._ppx_full * scale)

        # Horizontal band — middle BAND_FRAC of rows
        band_h = max(1, int(h * BAND_FRAC))
        r0     = (h - band_h) // 2
        band   = depth_m[r0 : r0 + band_h, :]          # (band_h, w)

        valid  = (band > 0) & (band >= MIN_RANGE_M) & (band <= MAX_RANGE_M)

        # ── Per-bin: 10th-percentile depth ───────────────────────────────────
        # Why percentile, not min:
        #   - One reflective/bad pixel at 0.3 m should NOT override a real wall at 5 m
        #   - 10th percentile = conservative (close to min) but ignores outliers
        # Why MIN_PIXELS_PER_BIN:
        #   - A bin with 1–2 valid pixels is statistically unreliable
        bin_depth_cm = np.full(NUM_BINS, -1.0, dtype=np.float64)  # -1 = nothing reliable

        for b in range(NUM_BINS):
            col_mask    = (self._col_bin == b)
            slice_valid = valid[:, col_mask]
            n_valid     = int(slice_valid.sum())

            if n_valid >= MIN_PIXELS_PER_BIN:
                valid_depths   = band[:, col_mask][slice_valid]          # metres
                robust_depth_m = float(np.percentile(valid_depths, DEPTH_PERCENTILE))
                bin_depth_cm[b] = robust_depth_m * 100.0                 # → cm

        # ── 72-element distances array ────────────────────────────────────────
        # Bins 0–(NUM_BINS-1) : camera data
        # Bins NUM_BINS–71    : UINT16_MAX (camera doesn't see these angles)
        #
        # PX4 _addObstacleSensorData() uses increment + angle_offset from the
        # message header to re-map each bin into its 72×5° internal map.
        # We do NOT need to pre-arrange at 5°/bin.
        distances = [UINT16_MAX] * BIN_COUNT

        for i in range(NUM_BINS):
            v = bin_depth_cm[i]
            if v < 0:
                # Not enough valid pixels — could be glass, dark surface, or out of range.
                # Report UNKNOWN (65535), NOT clear (801).
                # 801 = "I looked and confirmed nothing there" — we cannot claim that here.
                distances[i] = UINT16_MAX                       # 65535 — unknown
            else:
                distances[i] = int(np.clip(v, MIN_CM, MAX_CM)) # clamped cm

        self.get_logger().debug(
            f'bins={[distances[i] for i in range(NUM_BINS)]}',
            throttle_duration_sec=1.0,
        )

        return distances

    # ── Publish ───────────────────────────────────────────────────────────────
    def _publish(self, distances: list):
        msg = ObstacleDistance()
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)  # µs
        msg.frame        = ObstacleDistance.MAV_FRAME_BODY_FRD
        msg.sensor_type  = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        msg.min_distance = MIN_CM
        msg.max_distance = MAX_CM
        msg.increment    = float(self._increment_deg)
        msg.angle_offset = float(self._angle_offset)
        msg.distances    = distances
        self._pub.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
                self.get_logger().info('RealSense pipeline stopped.')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
