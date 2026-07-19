#!/usr/bin/env python3
"""
realsense_obstacle_node.py
==========================
RealSense D435i depth → PX4 Collision Prevention via uXRCE-DDS

Pipeline (verified end-to-end via NuttShell listener obstacle_distance):
  RealSense D435i (USB 3)
    → pyrealsense2 depth frame
      → horizontal band (middle 20% of rows)
        → 9 angular bins covering 87° horizontal FOV
          → px4_msgs/ObstacleDistance @ 10 Hz
            → /fmu/in/obstacle_distance
              → MicroXRCEAgent (uXRCE-DDS bridge)
                → PX4 obstacle_distance uORB
                  → Collision Prevention → brakes in Position mode

Bin geometry (matches verified fake-data test):
  NUM_BINS    = 9
  INCREMENT   = 87 / 9 = 9.667 deg/bin
  ANGLE_OFFSET= -43.5 deg  (left edge of FOV = bin 0)
  bin 0 = -43.5°   (far left)
  bin 4 = ~0°      (dead ahead)
  bin 8 = +43.5°   (far right)
  bins 9-71 = 65535 (UINT16_MAX = unknown, camera doesn't cover these)

Distance encoding (cm):
  20  .. 800 : measured obstacle distance
  801        : clear (looked, nothing in range)  MAX_CM + 1
  65535      : unknown (bin outside camera FOV)  UINT16_MAX

PX4 parameters required (set once in QGC):
  CP_DIST   > 0   (e.g. 3.0 ft ≈ 1 m) — activates Collision Prevention
  CP_GO_NO_DATA = 0 or 1 per preference

Run alongside:
  MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import ObstacleDistance

# ─────────────────────────────────────────────────────────────────────────────
# Geometry — D435i 87° horizontal FOV split into 9 bins
# These values were verified via NuttShell: listener obstacle_distance
# ─────────────────────────────────────────────────────────────────────────────
BIN_COUNT    = 72                       # PX4 message always has 72 slots (fixed)
UINT16_MAX   = 65535

NUM_BINS     = 9                        # active bins covering camera FOV
FOV_H_DEG   = 87.0                     # D435i horizontal FOV (degrees)
INCREMENT   = FOV_H_DEG / NUM_BINS      # 9.667 deg/bin
ANGLE_OFFSET = -(FOV_H_DEG / 2.0)      # -43.5 deg → left edge of FOV = bin 0

MIN_RANGE_M  = 0.20                    # minimum valid depth (metres)
MAX_RANGE_M  = 8.00                    # maximum valid depth (metres)
MIN_CM       = int(MIN_RANGE_M * 100)  # 20 cm
MAX_CM       = int(MAX_RANGE_M * 100)  # 800 cm
CLEAR_CM     = MAX_CM + 1              # 801 = "looked, nothing in range"

BAND_FRAC    = 0.20                    # fraction of image height sampled (middle)
PUBLISH_HZ   = 10.0                    # publish rate


class RealSenseObstacleNode(Node):

    def __init__(self):
        super().__init__('realsense_obstacle_node')

        # ── PX4 QoS: BEST_EFFORT + VOLATILE + depth 1 (required by uXRCE-DDS)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub = self.create_publisher(
            ObstacleDistance, '/fmu/in/obstacle_distance', px4_qos
        )

        self._pipeline     = None
        self._depth_scale  = 0.001      # D435i default (1 mm per unit)
        self._filters      = []

        # Pre-compute column → bin lookup table (done once at start)
        # Filled in _init_realsense() once we know the actual image width.
        self._col_bin = None

        self._init_realsense()

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._timer_cb)

        self.get_logger().info(
            f'\n'
            f'  realsense_obstacle_node started\n'
            f'  ─────────────────────────────────────────────\n'
            f'  Camera FOV   : {FOV_H_DEG}°\n'
            f'  Bins         : {NUM_BINS}  ×  {INCREMENT:.3f}° / bin\n'
            f'  angle_offset : {ANGLE_OFFSET}°  (bin 0 = far left)\n'
            f'  Range        : {MIN_RANGE_M} – {MAX_RANGE_M} m\n'
            f'  Band         : middle {int(BAND_FRAC*100)}% of image rows\n'
            f'  Rate         : {PUBLISH_HZ} Hz\n'
            f'  Topic        : /fmu/in/obstacle_distance\n'
            f'  ─────────────────────────────────────────────\n'
            f'  Verify with: listener obstacle_distance  (NuttShell)\n'
            f'  PX4 param  : CP_DIST > 0 to activate braking'
        )

    # ── RealSense initialisation ──────────────────────────────────────────────
    def _init_realsense(self):
        try:
            import pyrealsense2 as rs
            self._rs = rs
        except ImportError:
            self.get_logger().fatal(
                'pyrealsense2 not installed.\n'
                'Run: pip install pyrealsense2'
            )
            rclpy.shutdown()
            return

        try:
            pipeline = rs.pipeline()
            cfg      = rs.config()
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            profile = pipeline.start(cfg)

            # Read actual depth scale from camera firmware
            # D435i = 0.001 (1 mm per raw unit) but reading it is more reliable
            sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = sensor.get_depth_scale()
            self.get_logger().info(
                f'RealSense pipeline started | depth_scale = {self._depth_scale:.6f} m/unit'
            )

            # ── Post-processing filter chain ─────────────────────────────────
            # These fill holes and smooth the depth image.
            # decimation_filter reduces resolution → faster processing
            # spatial + temporal → fill holes and reduce noise
            self._filters = [
                rs.decimation_filter(),
                rs.threshold_filter(),
                rs.disparity_transform(True),
                rs.spatial_filter(),
                rs.temporal_filter(),
                rs.disparity_transform(False),
            ]

            self._pipeline = pipeline

            # Build column → bin lookup (based on image width after decimation)
            # decimation_filter halves resolution → width becomes 320
            decimated_width = 320
            self._build_col_bin_lookup(decimated_width)

        except Exception as exc:
            self.get_logger().fatal(
                f'Failed to start RealSense: {exc}\n'
                f'Check USB 3.0 cable and run: rs-enumerate-devices'
            )
            rclpy.shutdown()

    def _build_col_bin_lookup(self, width: int):
        """
        Pre-compute which angular bin each image column belongs to.

        Column 0       → ANGLE_OFFSET = -43.5° → bin 0
        Column width/2 → 0°                    → bin 4
        Column width-1 → +43.5°                → bin 8

        This lookup table is computed once and reused for every frame.
        """
        col_idx = np.arange(width)
        # Angle of each column (degrees), linearly interpolated across FOV
        angles  = ANGLE_OFFSET + (col_idx / width) * FOV_H_DEG

        # Map angle → bin index, clamped to [0, NUM_BINS-1]
        self._col_bin = np.clip(
            ((angles - ANGLE_OFFSET) / INCREMENT).astype(int),
            0, NUM_BINS - 1
        )

        self.get_logger().info(
            f'Column-to-bin lookup built | image_width={width} | '
            f'bins span {angles[0]:.1f}° to {angles[-1]:.1f}°'
        )

    # ── Timer callback ────────────────────────────────────────────────────────
    def _timer_cb(self):
        if self._pipeline is None or self._col_bin is None:
            return

        distances = self._process_frame()
        if distances is None:
            return

        self._publish(distances)

    # ── Frame processing ──────────────────────────────────────────────────────
    def _process_frame(self):
        """
        Grab one depth frame → apply filters → sample middle band →
        bucket columns into 9 angular bins → return 72-element list.
        Returns None if the camera fails this cycle.
        """
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=200)
        except Exception as exc:
            self.get_logger().warn(
                f'Frame timeout: {exc}', throttle_duration_sec=5.0
            )
            return None

        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None

        # Apply post-processing filters
        for f in self._filters:
            depth_frame = f.process(depth_frame)

        # Raw depth array (H × W, uint16, units = depth_scale metres)
        depth_raw = np.asanyarray(depth_frame.get_data())      # uint16
        depth_m   = depth_raw.astype(np.float32) * self._depth_scale  # metres

        h, w = depth_m.shape

        # Rebuild lookup if image width changed (e.g. after filter changes)
        if self._col_bin is None or len(self._col_bin) != w:
            self._build_col_bin_lookup(w)

        # ── Sample horizontal band (middle BAND_FRAC of rows) ──────────────
        band_h = max(1, int(h * BAND_FRAC))
        r0     = (h - band_h) // 2
        band   = depth_m[r0 : r0 + band_h, :]   # shape: (band_h, w)

        # Valid pixel mask: must be nonzero, within range
        valid = (band > 0) & (band >= MIN_RANGE_M) & (band <= MAX_RANGE_M)

        # ── Per-bin minimum distance ────────────────────────────────────────
        # bin_min_cm[b] = minimum distance (cm) of any valid pixel in bin b
        # -1 means "no valid pixel found" → report as CLEAR
        bin_min_cm = np.full(NUM_BINS, -1.0, dtype=np.float64)

        for b in range(NUM_BINS):
            col_mask    = (self._col_bin == b)       # columns belonging to bin b
            slice_valid = valid[:, col_mask]          # valid pixels in those columns
            if slice_valid.any():
                min_m = band[:, col_mask][slice_valid].min()
                bin_min_cm[b] = min_m * 100.0         # metres → cm

        # ── Build 72-element distances array ───────────────────────────────
        # Structure (identical to verified fake-data test):
        #   Bins 0–8   : real camera data
        #   Bins 9–71  : UINT16_MAX (unknown — camera doesn't cover these angles)
        distances = [UINT16_MAX] * BIN_COUNT

        for i in range(NUM_BINS):
            v = bin_min_cm[i]
            if v < 0:
                distances[i] = CLEAR_CM                        # 801 = clear
            else:
                distances[i] = int(np.clip(v, MIN_CM, MAX_CM)) # clamped cm

        # Debug: log the 9 active bins
        active = [distances[i] for i in range(NUM_BINS)]
        self.get_logger().debug(
            f'bins[0-8]={active}', throttle_duration_sec=1.0
        )

        return distances

    # ── Publish ───────────────────────────────────────────────────────────────
    def _publish(self, distances: list):
        """
        Publish px4_msgs/ObstacleDistance.
        Fields match exactly what was verified via NuttShell listener.
        """
        msg = ObstacleDistance()
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)  # µs
        msg.frame        = ObstacleDistance.MAV_FRAME_BODY_FRD
        msg.sensor_type  = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        msg.min_distance = MIN_CM           # 20 cm
        msg.max_distance = MAX_CM           # 800 cm
        msg.increment    = float(INCREMENT) # 9.667°
        msg.angle_offset = float(ANGLE_OFFSET)  # -43.5°
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
