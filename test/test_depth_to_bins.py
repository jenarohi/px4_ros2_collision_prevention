#!/usr/bin/env python3
"""
Unit tests for depth_frame_to_bins logic.
==========================================
These tests run WITHOUT a RealSense camera or ROS installation.
They test the core math: pixel columns → angular bins → cm distances.

Run with:
  python3 -m pytest test/test_depth_to_bins.py -v
"""

import numpy as np
import pytest
import sys
import os

# Allow running without ROS installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ── Replicate the core conversion logic from the node ────────────────────────
NUM_BINS      = 72
INCREMENT_DEG = 360.0 / NUM_BINS   # 5.0°
FOV_H_DEG     = 87.0
ANGLE_OFFSET  = -FOV_H_DEG / 2    # −43.5°
MIN_M         = 0.20
MAX_M         = 10.0
DEPTH_SCALE   = 0.001              # D435 default
UINT16_MAX    = 65535
WIDTH         = 640


def depth_row_to_bins(row_uint16: np.ndarray,
                      depth_scale: float = DEPTH_SCALE) -> np.ndarray:
    """
    Pure Python replica of RealSenseObstacleNode._depth_frame_to_bins().
    Takes a 1D uint16 row (one row from the depth image) and returns bins_cm.
    """
    row_m = row_uint16.astype(np.float32) * depth_scale
    invalid = (row_m == 0) | (row_m < MIN_M) | (row_m > MAX_M)
    row_m[invalid] = MAX_M

    bins_cm = np.full(NUM_BINS, UINT16_MAX, dtype=np.uint16)
    num_cols = len(row_m)

    col_idx = np.arange(num_cols)
    angles  = ANGLE_OFFSET + (col_idx / num_cols) * FOV_H_DEG
    b_idx   = (np.round(angles % 360 / INCREMENT_DEG).astype(int) % NUM_BINS)
    dist_cm = np.minimum((row_m * 100).astype(np.uint16), 65534)
    np.minimum.at(bins_cm, b_idx, dist_cm)

    return bins_cm


# ── Tests ─────────────────────────────────────────────────────────────────────

class TestBinCount:
    def test_72_bins_returned(self):
        row = np.zeros(WIDTH, dtype=np.uint16)
        result = depth_row_to_bins(row)
        assert result.shape == (72,)

    def test_dtype_is_uint16(self):
        row = np.zeros(WIDTH, dtype=np.uint16)
        result = depth_row_to_bins(row)
        assert result.dtype == np.uint16


class TestUnknownBins:
    def test_zero_pixels_become_max_range_not_unknown(self):
        """
        Zero depth (no return) should map to MAX_M (sensor max),
        which then maps to max_distance cm — NOT UINT16_MAX.
        This means "no obstacle detected" at max range.
        """
        row = np.zeros(WIDTH, dtype=np.uint16)
        result = depth_row_to_bins(row)
        # All filled bins should be at max distance
        filled = result[result < UINT16_MAX]
        assert np.all(filled == int(MAX_M * 100))

    def test_bins_outside_fov_remain_uint16_max(self):
        """
        The D435 only covers ±43.5° (87° total).
        Bins outside this FOV should stay UINT16_MAX (unknown).
        """
        row = np.full(WIDTH, int(0.5 / DEPTH_SCALE), dtype=np.uint16)  # 0.5 m
        result = depth_row_to_bins(row)
        # ~180° of bins (far side) should remain unknown
        unknown_count = np.sum(result == UINT16_MAX)
        # At least half the bins should be unknown (camera has 87° not 360° FOV)
        assert unknown_count > 30, f'Expected >30 unknown bins, got {unknown_count}'


class TestDistanceConversion:
    def test_close_obstacle_detected(self):
        """A 0.5 m obstacle should appear as ~50 cm in the forward bins."""
        val = int(0.5 / DEPTH_SCALE)  # 500 raw units = 0.5 m at 0.001 scale
        row = np.full(WIDTH, val, dtype=np.uint16)
        result = depth_row_to_bins(row)
        filled = result[result < UINT16_MAX]
        # All visible bins should be ~50 cm
        assert np.all(filled == 50), f'Expected 50 cm, got: {np.unique(filled)}'

    def test_obstacle_at_min_range_boundary(self):
        """Distance exactly at min_depth_m should be valid (not filtered out)."""
        val = int(MIN_M / DEPTH_SCALE)  # exactly 0.20 m
        row = np.full(WIDTH, val, dtype=np.uint16)
        result = depth_row_to_bins(row)
        filled = result[result < UINT16_MAX]
        assert len(filled) > 0

    def test_obstacle_below_min_range_treated_as_max(self):
        """Values below min_depth_m should be treated as max distance (ignored)."""
        val = int(0.05 / DEPTH_SCALE)  # 5 cm — too close, below min
        row = np.full(WIDTH, val, dtype=np.uint16)
        result = depth_row_to_bins(row)
        filled = result[result < UINT16_MAX]
        # Should be max distance (no obstacle)
        assert np.all(filled == int(MAX_M * 100))

    def test_minimum_distance_wins_in_bin(self):
        """
        When multiple pixels map to the same bin, the closest should win.
        """
        row = np.zeros(WIDTH, dtype=np.uint16)
        # Set all pixels to 5 m
        row[:] = int(5.0 / DEPTH_SCALE)
        # Override a few pixels to 1 m
        row[WIDTH // 2 - 2 : WIDTH // 2 + 2] = int(1.0 / DEPTH_SCALE)

        result = depth_row_to_bins(row)
        # At least one bin should be ~100 cm (the 1 m pixels)
        assert np.any(result[result < UINT16_MAX] <= 100)


class TestGeometry:
    def test_forward_bin_receives_data(self):
        """
        The centre columns map to angle ~0° (forward), which is bin 0.
        With a wall ahead, bin 0 should be populated.
        """
        row = np.full(WIDTH, int(2.0 / DEPTH_SCALE), dtype=np.uint16)
        result = depth_row_to_bins(row)
        # Bin 0 is forward — should have a valid reading
        assert result[0] < UINT16_MAX, 'Forward bin (0) should be populated'

    def test_no_negative_distances(self):
        """No bin should have a distance below min_distance."""
        row = np.random.randint(0, 10000, size=WIDTH, dtype=np.uint16)
        result = depth_row_to_bins(row)
        filled = result[result < UINT16_MAX]
        if len(filled) > 0:
            assert np.all(filled >= int(MIN_M * 100))


class TestEdgeCases:
    def test_all_max_range(self):
        """All pixels at max range → all valid bins should be at max distance cm."""
        row = np.full(WIDTH, int(MAX_M / DEPTH_SCALE), dtype=np.uint16)
        result = depth_row_to_bins(row)
        filled = result[result < UINT16_MAX]
        assert np.all(filled == int(MAX_M * 100))

    def test_empty_row(self):
        """Empty (all-zero) row should not crash."""
        row = np.zeros(WIDTH, dtype=np.uint16)
        result = depth_row_to_bins(row)
        assert result.shape == (72,)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
