import numpy as np

from indoor_nav_costmap.depth_to_scan import DepthToScan


def test_inverse_midas_depth_maps_larger_values_closer():
    depth = np.tile(np.linspace(0.0, 255.0, 120), (40, 1))

    ranges = DepthToScan.ranges_from_relative_depth(
        depth,
        vertical_roi_top=0.20,
        vertical_roi_bottom=0.85,
        target_beams=12,
        min_range=0.15,
        clearing_max_range=3.5,
        inverse_depth=True,
    )

    assert len(ranges) == 12
    assert ranges[0] > ranges[-1]
    assert np.isclose(ranges[-1], 0.15)


def test_flat_relative_depth_clears_without_marking_an_obstacle():
    depth = np.full((40, 120), 127.0, dtype=np.float32)

    ranges = DepthToScan.ranges_from_relative_depth(
        depth,
        vertical_roi_top=0.20,
        vertical_roi_bottom=0.85,
        target_beams=12,
        min_range=0.15,
        clearing_max_range=3.5,
        inverse_depth=True,
    )

    assert np.allclose(ranges, 3.5)


def test_malformed_depth_has_no_scan_output():
    assert DepthToScan.ranges_from_relative_depth(
        np.zeros((4, 4, 3), dtype=np.float32),
        vertical_roi_top=0.20,
        vertical_roi_bottom=0.85,
        target_beams=12,
        min_range=0.15,
        clearing_max_range=3.5,
        inverse_depth=True,
    ) is None
