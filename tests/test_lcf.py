"""Lens Correction Filter (LCF) map parsing and remap: synthetic FFF data only."""

import struct

import numpy as np
import pytest

from flir_calib import FlirCalibration
from flir_lcf import (
    LensCorrectionMap,
    build_synthetic_fff,
    decode_lcf_grid,
    expand_displacement_grid,
    parse_lcf_header,
)

ROWS, COLS = 18, 13  # Gen 1 grid: 26 x 18 entries = 13 x 18 (dx, dy) vectors


def _zoom_grid(src_scale: float):
    """Coarse grid for a pure zoom about the portrait frame center.

    FLIR's software spaces the nodes (cols - 2) / (rows - 2) intervals across the frame,
    so the last column/row sit one interval beyond the right/bottom edge.
    """
    xs = np.arange(COLS) * 59 / (COLS - 2)
    ys = np.arange(ROWS) * 79 / (ROWS - 2)
    gx, gy = np.meshgrid(xs, ys)
    return (src_scale - 1) * (gx - 29.5), (src_scale - 1) * (gy - 39.5)


@pytest.mark.parametrize("order", ["<", ">"])
def test_header_and_signed_fixed_point_via_fff_index(order):
    dx = np.full((ROWS, COLS), -2.5)
    dy = np.full((ROWS, COLS), 1.25)
    blob = build_synthetic_fff(dx, dy, byte_order=order)
    hdr = parse_lcf_header(blob)
    assert (hdr.byte_order, hdr.map_width, hdr.map_height, hdr.fraction_bits) == (order, 26, 18, 10)
    dx2, dy2, _ = decode_lcf_grid(blob)
    assert dx2.shape == (ROWS, COLS)
    assert np.allclose(dx2, -2.5) and np.allclose(dy2, 1.25)


def test_rejects_files_without_an_lcf_entry():
    blob = bytearray(build_synthetic_fff(np.zeros((ROWS, COLS)), np.zeros((ROWS, COLS))))
    struct.pack_into(">H", blob, 64, 0x01)  # retag the only index entry
    with pytest.raises(ValueError):
        parse_lcf_header(bytes(blob))
    with pytest.raises(ValueError):
        parse_lcf_header(b"not an fff file" + bytes(64))


def test_expansion_uses_sdk_node_spacing():
    dx_c = np.arange(ROWS * COLS, dtype=float).reshape(ROWS, COLS)
    dx, dy = expand_displacement_grid(dx_c, -dx_c, 60, 80)
    assert dx.shape == (80, 60)
    # Corners land on nodes (0, 0) and (rows-2, cols-2); the last row/column only
    # serve as the far neighbor, with zero weight at the edge itself.
    assert dx[0, 0] == dx_c[0, 0]
    assert dx[-1, -1] == dx_c[ROWS - 2, COLS - 2]
    assert dx[0, -1] == dx_c[0, COLS - 2] and dx[-1, 0] == dx_c[ROWS - 2, 0]
    assert np.allclose(dy, -dx)
    with pytest.raises(ValueError):
        expand_displacement_grid(np.zeros((2, 2)), np.zeros((2, 2)), 60, 80)


def test_centered_zoom_keeps_the_center_and_pulls_edges_in():
    lcf = LensCorrectionMap(*_zoom_grid(0.8))
    assert lcf.dx.shape == (80, 60)
    # Fixed point at the frame center; edges sample from inside the frame.
    assert abs(lcf.dx[40, 29] - 0.1) < 1e-4 and abs(lcf.dy[39, 30] - 0.1) < 1e-4
    assert lcf.dx[0, 0] > 5 and lcf.dx[0, -1] < -5
    assert lcf.dy[0, 0] > 7 and lcf.dy[-1, 0] < -7


def test_sensor_frame_is_remapped_in_portrait_orientation():
    import cv2

    dx = np.full((ROWS, COLS), 1.0)   # portrait: sample one pixel to the right
    dy = np.zeros((ROWS, COLS))
    lcf = LensCorrectionMap.from_bytes(build_synthetic_fff(dx, dy))
    sensor = np.arange(60 * 80, dtype=np.uint16).reshape(60, 80)
    out = lcf.remap(sensor)
    assert out.shape == (60, 80) and out.dtype == np.uint16
    portrait_in = cv2.rotate(sensor, cv2.ROTATE_90_CLOCKWISE)
    portrait_out = cv2.rotate(out, cv2.ROTATE_90_CLOCKWISE)
    assert np.array_equal(portrait_out[:, :-1], portrait_in[:, 1:])


def test_calib_apply_lcf_is_optional():
    c = FlirCalibration()
    img = np.full((60, 80), 25.0, dtype=np.float32)
    assert not c.lcf_active
    assert c.apply_lcf(img) is img
    c._lcf = LensCorrectionMap(*_zoom_grid(0.9))
    assert c.lcf_active
    assert np.allclose(c.apply_lcf(img), 25.0)
    c.lcf_enabled = False
    assert c.apply_lcf(img) is img
