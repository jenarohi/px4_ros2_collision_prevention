#!/usr/bin/env python3
"""
depth_replay_node.py
====================
ROS 2 node that replays pre-recorded RealSense D435 depth frames
(stored as .xlsx files) and publishes them as px4_msgs/ObstacleDistance
to PX4's Collision Prevention module — identical pipeline to the live
realsense_obstacle_node, but driven from recorded data instead of hardware.

Recorded data format (from RealSense recording tool):
  - One .xlsx file per frame, named:  depthdata_filter_<N>.xlsx
  - Shape : 270 rows × 480 cols   (half-resolution depth stream)
  - Units : millimetres (mm), float
  - Zero  : invalid / no return pixel

Use cases:
  ✓ Validate CP behaviour without a physical camera
  ✓ Reproduce a specific flight scenario exactly
  ✓ Stress-test PX4 CP with known obstacle data
  ✓ CI / HIL testing on any Linux machine

ROS 2 Parameters:
  data_dir      (str)   Path to folder with depthdata_filter_*.xlsx files
  publish_hz    (float) Publish rate in Hz  [default: 10.0]
  loop          (bool)  Loop the dataset forever  [default: false]
  fov_h_deg     (float) Camera horizontal FOV degrees  [default: 87.0]
  min_depth_m   (float) Minimum valid range metres  [default: 0.20]
  max_depth_m   (float) Maximum valid range metres  [default: 8.00]
  num_bins      (int)   Number of angular bins per FOV  [default: 9]
  depth_percentile (int) Percentile for robust depth per bin  [default: 10]
  min_pixels_per_bin (int) Min valid pixels to report a bin  [default: 5]

Topics Published:
  /fmu/in/obstacle_distance  [px4_msgs/ObstacleDistance]

Build:
  colcon build --packages-select px4_ros2_collision_prevention

Run:
  ros2 run px4_ros2_collision_prevention depth_replay_node \
      --ros-args -p data_dir:=/path/to/xlsx/frames -p loop:=true

Launch:
  ros2 launch px4_ros2_collision_prevention depth_replay.launch.py \
      data_dir:=/path/to/frames loop:=true publish_hz:=10.0
"""

import glob
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from px4_msgs.msg import ObstacleDistance

# ── Optional: openpyxl (warn at startup if missing) ─────────────────────────
try:
    import openpyxl
    _HAS_OPENPYXL = True
except ImportError:
    _HAS_OPENPYXL = False

# ── PX4 message constants ────────────────────────────────────────────────────
BIN_COUNT  = 72       # Fixed array size in ObstacleDistance message
UINT16_MAX = 65535    # "unknown" — PX4 CP ignores these bins (does not brake)

# ── Default parameters ───────────────────────────────────────────────────────
DEFAULT_PUBLISH_HZ       = 10.0
DEFAULT_FOV_H_DEG        = 87.0
DEFAULT_MIN_DEPTH_M      = 0.20
DEFAULT_MAX_DEPTH_M      = 8.00
DEFAULT_NUM_BINS         = 9
DEFAULT_DEPTH_PERCENTILE = 10
DEFAULT_MIN_PIXELS       = 5


class DepthReplayNode(Node):
    """
    Replays recorded depth frames as ObstacleDistance messages.

    Processing pipeline (matches realsense_obstacle_node.py):
      1. Load depth frame from xlsx (mm → float32 numpy array)
      2. Crop to middle BAND_FRAC of rows  (horizontal obstacle plane)
      3. Map columns → angular bins via pinhole angle formula
      4. Compute Nth-percentile depth per bin  (robust vs outliers)
      5. Reject sparse bins  (< min_pixels_per_bin valid pixels)
      6. Convert m → cm, pack into 72-element uint16 array
      7. Publish ObstacleDistance with correct increment + angle_offset
    """

    BAND_FRAC = 0.20   # middle 20% of frame rows (same as live node)

    def __init__(self):
        super().__init__('depth_replay_node')

        # ── Check openpyxl ───────────────────────────────────────────────────
        if not _HAS_OPENPYXL:
            self.get_logger().fatal(
                'openpyxl is not installed. '
                'Run:  pip install openpyxl'
            )
            rclpy.shutdown()
            return

        # ── Declare ROS 2 parameters ─────────────────────────────────────────
        self.declare_parameter('data_dir',           '')
        self.declare_parameter('publish_hz',         DEFAULT_PUBLISH_HZ)
        self.declare_parameter('loop',               False)
        self.declare_parameter('fov_h_deg',          DEFAULT_FOV_H_DEG)
        self.declare_parameter('min_depth_m',        DEFAULT_MIN_DEPTH_M)
        self.declare_parameter('max_depth_m',        DEFAULT_MAX_DEPTH_M)
        self.declare_parameter('num_bins',           DEFAULT_NUM_BINS)
        self.declare_parameter('depth_percentile',   DEFAULT_DEPTH_PERCENTILE)
        self.declare_parameter('min_pixels_per_bin', DEFAULT_MIN_PIXELS)

        # ── Read parameters ──────────────────────────────────────────────────
        self._data_dir    = self.get_parameter('data_dir').value
        self._publish_hz  = self.get_parameter('publish_hz').value
        self._loop        = self.get_parameter('loop').value
        self._fov_h_deg   = self.get_parameter('fov_h_deg').value
        self._min_m       = self.get_parameter('min_depth_m').value
        self._max_m       = self.get_parameter('max_depth_m').value
        self._num_bins    = self.get_parameter('num_bins').value
        self._percentile  = self.get_parameter('depth_percentile').value
        self._min_pixels  = self.get_parameter('min_pixels_per_bin').value

        self._min_cm = int(self._min_m * 100)
        self._max_cm = int(self._max_m * 100)

        # ── Validate data_dir ────────────────────────────────────────────────
        if not self._data_dir:
            self.get_logger().fatal(
                'Parameter "data_dir" is not set. '
                'Pass it with:  --ros-args -p data_dir:=/path/to/frames'
            )
            rclpy.shutdown()
            return

        if not os.path.isdir(self._data_dir):
            self.get_logger().fatal(
                f'data_dir does not exist or is not a directory: {self._data_dir}'
            )
            rclpy.shutdown()
            return

        # ── Discover frame files ─────────────────────────────────────────────
        self._frames = self._discover_frames(self._data_dir)
        if not self._frames:
            self.get_logger().fatal(
                f'No depthdata_filter_*.xlsx files found in: {self._data_dir}'
            )
            rclpy.shutdown()
            return

        self.get_logger().info(
            f'Found {len(self._frames)} depth frames in: {self._data_dir}'
        )

        # ── Pre-compute column → bin lookup using pinhole model ──────────────
        # We use the typical D435 640-pixel width and approximate fx.
        # This matches how realsense_obstacle_node.py builds its lookup.
        # Data frames are 480 wide (after half-res recording).
        self._frame_width = 480    # columns in recorded frames
        self._build_col_bin_lookup(
            width     = self._frame_width,
            fx        = self._frame_width / (2.0 * np.tan(np.radians(self._fov_h_deg / 2.0))),
            ppx       = self._frame_width / 2.0,
        )

        # ── Publisher ────────────────────────────────────────────────────────
        px4_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1,
        )
        self._pub = self.create_publisher(
            ObstacleDistance, '/fmu/in/obstacle_distance', px4_qos
        )

        # ── Playback state ───────────────────────────────────────────────────
        self._frame_idx   = 0
        self._loop_count  = 0
        self._total_sent  = 0
        self._skipped     = 0

        # ── Timer ────────────────────────────────────────────────────────────
        self._timer = self.create_timer(
            1.0 / self._publish_hz,
            self._timer_cb,
        )

        self.get_logger().info(
            f'depth_replay_node started | '
            f'{self._publish_hz} Hz | '
            f'{self._num_bins} bins × {self._increment_deg:.2f}°/bin | '
            f'FOV={self._fov_h_deg}° | '
            f'range={self._min_m}–{self._max_m} m | '
            f'loop={self._loop}'
        )

    # ── Frame discovery ───────────────────────────────────────────────────────
    @staticmethod
    def _discover_frames(data_dir: str) -> list:
        """Return xlsx paths sorted numerically by frame index N."""
        pattern = os.path.join(data_dir, 'depthdata_filter_*.xlsx')
        files   = glob.glob(pattern)

        def _frame_idx(p):
            base = os.path.splitext(os.path.basename(p))[0]
            return int(base.split('_')[-1])

        return sorted(files, key=_frame_idx)

    # ── Column → bin lookup (pinhole model) ───────────────────────────────────
    def _build_col_bin_lookup(self, width: int, fx: float, ppx: float):
        """
        Identical to realsense_obstacle_node._build_col_bin_lookup().

        For each pixel column x:
            angle = arctan2(x - ppx, fx)   [degrees]

        Bin 0 = leftmost angular slice, Bin num_bins-1 = rightmost.
        increment and angle_offset are stored for the ObstacleDistance message
        so PX4 can correctly map bins into its internal 72×5° collision map.
        """
        col_idx = np.arange(width, dtype=float)
        angles  = np.degrees(np.arctan2(col_idx - ppx, fx))   # shape (width,)

        lo, hi = float(angles.min()), float(angles.max())
        edges  = np.linspace(lo, hi, self._num_bins + 1)

        self._col_bin       = np.clip(
            np.digitize(angles, edges) - 1, 0, self._num_bins - 1
        )
        self._increment_deg = float(edges[1] - edges[0])
        # angle_offset = centre of bin 0
        self._angle_offset  = float(0.5 * (edges[0] + edges[1]))

        self.get_logger().info(
            f'Bin lookup | w={width} fx={fx:.1f} ppx={ppx:.1f} | '
            f'FOV={hi - lo:.1f}° | '
            f'{self._num_bins}×{self._increment_deg:.2f}°/bin | '
            f'angle_offset={self._angle_offset:.2f}°'
        )

    # ── Timer callback ────────────────────────────────────────────────────────
    def _timer_cb(self):
        if self._frame_idx >= len(self._frames):
            # Dataset exhausted
            self._loop_count += 1
            if self._loop:
                self.get_logger().info(
                    f'Loop {self._loop_count} complete — restarting '
                    f'(sent={self._total_sent}, skipped={self._skipped})'
                )
                self._frame_idx = 0
            else:
                self.get_logger().info(
                    f'Replay finished — all {len(self._frames)} frames sent '
                    f'(total={self._total_sent}, skipped={self._skipped}). '
                    f'Shutting down node.'
                )
                self._timer.cancel()
                rclpy.shutdown()
                return

        xlsx_path = self._frames[self._frame_idx]
        self._frame_idx += 1

        # Load frame
        depth_mm = self._load_frame(xlsx_path)
        if depth_mm is None:
            self._skipped += 1
            return   # corrupt file — skip this tick

        # Convert to bin distances
        distances = self._frame_to_bins(depth_mm)

        # Publish
        self._publish(distances)
        self._total_sent += 1

        # Periodic progress log
        if self._total_sent % 50 == 0:
            pct = self._frame_idx / len(self._frames) * 100.0
            known  = [d for d in distances if d != UINT16_MAX]
            closest = (min(known) / 100.0) if known else float('nan')
            self.get_logger().info(
                f'[{self._frame_idx}/{len(self._frames)}] '
                f'{pct:.0f}% | '
                f'valid_bins={len(known)}/{self._num_bins} | '
                f'closest={closest:.2f} m | '
                f'sent={self._total_sent}'
            )

    # ── Load one xlsx frame ───────────────────────────────────────────────────
    def _load_frame(self, xlsx_path: str):
        """Load xlsx → float32 numpy array (H × W) in mm. None if corrupt."""
        try:
            wb = openpyxl.load_workbook(
                xlsx_path, read_only=True, data_only=True
            )
            ws = wb.active
            data = np.array(
                [
                    [c if c is not None else 0.0 for c in row]
                    for row in ws.iter_rows(values_only=True)
                ],
                dtype=np.float32,
            )
            wb.close()
            return data
        except Exception as exc:
            self.get_logger().warn(
                f'Skipping corrupt frame: {os.path.basename(xlsx_path)} '
                f'({exc})',
                throttle_duration_sec=5.0,
            )
            return None

    # ── Depth → bins ──────────────────────────────────────────────────────────
    def _frame_to_bins(self, depth_mm: np.ndarray) -> list:
        """
        Convert depth frame (mm) to 72-element uint16 distances list (cm).

        Matches realsense_obstacle_node._process_frame() logic:
          - Middle BAND_FRAC rows sampled
          - Nth-percentile depth per bin (robust vs hot pixels)
          - Bins with < min_pixels_per_bin valid pixels → UNKNOWN
          - Bins outside camera FOV → UNKNOWN
        """
        H, W = depth_mm.shape

        # Crop to middle band (horizontal plane)
        band_h = max(1, int(H * self.BAND_FRAC))
        r0     = (H - band_h) // 2
        band   = depth_mm[r0 : r0 + band_h, :].astype(np.float32)  # (band_h, W)

        # mm → metres
        band_m = band / 1000.0

        # Valid pixel mask
        valid = (band_m > 0) & (band_m >= self._min_m) & (band_m <= self._max_m)

        # Per-bin percentile depth
        bin_depth_cm = np.full(self._num_bins, -1.0, dtype=np.float64)

        # Resize col_bin lookup if frame width differs from expected
        if W != self._frame_width:
            self.get_logger().warn(
                f'Frame width {W} != expected {self._frame_width}. '
                f'Rebuilding bin lookup.',
                throttle_duration_sec=10.0,
            )
            self._frame_width = W
            self._build_col_bin_lookup(
                width = W,
                fx    = W / (2.0 * np.tan(np.radians(self._fov_h_deg / 2.0))),
                ppx   = W / 2.0,
            )

        for b in range(self._num_bins):
            col_mask    = (self._col_bin == b)
            slice_valid = valid[:, col_mask]
            n_valid     = int(slice_valid.sum())

            if n_valid >= self._min_pixels:
                valid_depths    = band_m[:, col_mask][slice_valid]     # metres
                robust_depth_m  = float(np.percentile(valid_depths, self._percentile))
                bin_depth_cm[b] = robust_depth_m * 100.0               # → cm

        # Pack 72-element distances array
        # Bins 0..(num_bins-1): camera data or UNKNOWN
        # Bins num_bins..71:    always UNKNOWN (outside camera FOV)
        distances = [UINT16_MAX] * BIN_COUNT

        for i in range(self._num_bins):
            v = bin_depth_cm[i]
            if v < 0:
                # Not enough reliable pixels → unknown (NOT clear)
                distances[i] = UINT16_MAX
            else:
                distances[i] = int(np.clip(v, self._min_cm, self._max_cm))

        return distances

    # ── Publish ObstacleDistance ──────────────────────────────────────────────
    def _publish(self, distances: list):
        msg = ObstacleDistance()
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)  # µs
        msg.frame        = ObstacleDistance.MAV_FRAME_BODY_FRD
        msg.sensor_type  = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        msg.min_distance = self._min_cm
        msg.max_distance = self._max_cm
        msg.increment    = float(self._increment_deg)
        msg.angle_offset = float(self._angle_offset)
        msg.distances    = distances
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
