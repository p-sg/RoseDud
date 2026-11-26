"""
Lens Correction Filter (LCF): geometric correction from *_LCFMap.fff. EXPERIMENTAL.

Follows how FLIR's own software applies the map:
  * The map lives in the FFF index entry with tag 0x14. Its 32-byte header holds
    bytes-per-entry (2), map width (26) and height (18); byte 29 is the number of
    fraction bits (0 means the default, 10). The payload is width x height signed
    16-bit fixed-point values in the header's byte order.
  * Each row interleaves (dx, dy) pairs: a 13 x 18 grid of vectors.
  * The grid is spread over the image with (cols - 2) and (rows - 2) intervals,
    so the last grid column and row only serve as the far neighbor at the
    right/bottom edge.
  * The values are displacements in pixels of the frame the correction is
    applied to, before any resizing.
  * Sampling: I_out(x, y) = I_in(x + dx, y + dy), bilinear.
  * The map is stored in PORTRAIT orientation. FLIR only rotates it for the
    FLIR ONE Edge models; the phone-case models get it as stored, i.e. applied
    to a portrait frame. On the Gen 1 that is the 60 x 80 frame; in the sensor's
    80 x 60 orientation the field is anisotropic and squashes round objects.

Consequence for the Gen 1 map: it is mostly a zoom of about 1.34 x 1.39 (it
crops the frame to roughly x 8-52, y 12-69) plus a ~0.45 degree rotation and
<= ~1 px of real distortion, so applying it discards about a quarter of the
field of view.

Not established: whether the portrait frame is the sensor frame rotated
clockwise (as the GUI shows it) or counter-clockwise. FLIR's current software
does not support the Gen 1, and the map is too symmetric for live frames to
tell. Clockwise is used. The lensCorr rotation/zoom values in calib.rsc are not
used by FLIR's software.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Union

import numpy as np

FFF_MAGIC = b"FFF\x00"
FFF_TAG_LCF_MAP = 0x14
_INDEX_ENTRY_SIZE = 32
_MAP_HEADER_SIZE = 32
_DEFAULT_FRACTION_BITS = 10

# Thermal frame as the map expects it (portrait: width x height).
PORTRAIT_WIDTH = 60
PORTRAIT_HEIGHT = 80


@dataclass
class LcfHeader:
    byte_order: str          # "<" or ">"
    bytes_per_entry: int
    map_width: int           # 16-bit entries per row (dx, dy interleaved)
    map_height: int
    fraction_bits: int
    raw: Tuple[int, ...]     # all 16 header words, for inspection
    payload_offset: int


def _fff_entry(data: bytes, tag: int) -> Tuple[int, int]:
    """(offset, length) of the first FFF index entry with ``tag`` (big-endian index)."""
    if not data.startswith(FFF_MAGIC):
        raise ValueError("not an FFF file")
    _version, index_offset, n_entries = struct.unpack_from(">III", data, 20)
    for k in range(n_entries):
        pos = index_offset + k * _INDEX_ENTRY_SIZE
        if pos + _INDEX_ENTRY_SIZE > len(data):
            break
        entry_tag, _sub, _ver, _id, offset, length = struct.unpack_from(">HHIIII", data, pos)
        if entry_tag == tag:
            return offset, length
    raise ValueError(f"no FFF entry with tag {tag:#x}")


def parse_lcf_header(data: bytes) -> LcfHeader:
    """Find and decode the LCF map header via the FFF index."""
    offset, length = _fff_entry(data, FFF_TAG_LCF_MAP)
    if length < _MAP_HEADER_SIZE or offset + length > len(data):
        raise ValueError("LCF entry is truncated")

    if struct.unpack_from("<H", data, offset)[0] == 2:
        order = "<"
    elif struct.unpack_from(">H", data, offset)[0] == 2:
        order = ">"
    else:
        raise ValueError("unsupported LCF map entry size (expected 2 bytes per entry)")

    words = struct.unpack_from(f"{order}16H", data, offset)
    width, height = words[1], words[2]
    frac = data[offset + 29] or _DEFAULT_FRACTION_BITS
    need = width * height * 2
    if width < 4 or height < 2 or width % 2 or _MAP_HEADER_SIZE + need > length:
        raise ValueError(f"implausible LCF map size {width}x{height}")
    return LcfHeader(
        byte_order=order,
        bytes_per_entry=words[0],
        map_width=width,
        map_height=height,
        fraction_bits=int(frac),
        raw=words,
        payload_offset=offset + _MAP_HEADER_SIZE,
    )


def decode_lcf_grid(data: bytes) -> Tuple[np.ndarray, np.ndarray, LcfHeader]:
    """Coarse (dx, dy) grids, each map_height x (map_width / 2), in map units."""
    hdr = parse_lcf_header(data)
    n = hdr.map_width * hdr.map_height
    values = np.frombuffer(
        data, dtype=f"{hdr.byte_order}i2", count=n, offset=hdr.payload_offset
    ).astype(np.float64)
    grid = values.reshape(hdr.map_height, hdr.map_width) / float(1 << hdr.fraction_bits)
    return grid[:, 0::2].copy(), grid[:, 1::2].copy(), hdr


def expand_displacement_grid(
    dx_c: np.ndarray, dy_c: np.ndarray, out_w: int, out_h: int
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Bilinearly expand a coarse grid the way FLIR's software does: output pixel x
    maps to grid position x * (cols - 2) / (out_w - 1) (likewise for y), so the
    last grid column and row are only reached as the far interpolation neighbor.
    """
    rows, cols = dx_c.shape
    if dy_c.shape != (rows, cols):
        raise ValueError("coarse dx/dy shape mismatch")
    if rows < 3 or cols < 3:
        raise ValueError("coarse grid must be at least 3x3")

    gx = np.arange(out_w) * (cols - 2) / max(out_w - 1, 1)
    gy = np.arange(out_h) * (rows - 2) / max(out_h - 1, 1)
    i0 = np.clip(np.floor(gx).astype(int), 0, cols - 2)
    j0 = np.clip(np.floor(gy).astype(int), 0, rows - 2)
    fx = (gx - i0)[np.newaxis, :]
    fy = (gy - j0)[:, np.newaxis]
    jj, ii = np.meshgrid(j0, i0, indexing="ij")

    def bilinear(c: np.ndarray) -> np.ndarray:
        top = (1 - fx) * c[jj, ii] + fx * c[jj, ii + 1]
        bottom = (1 - fx) * c[jj + 1, ii] + fx * c[jj + 1, ii + 1]
        return ((1 - fy) * top + fy * bottom).astype(np.float32)

    return bilinear(dx_c), bilinear(dy_c)


class LensCorrectionMap:
    """Per-pixel displacement field for the portrait 60x80 thermal frame."""

    def __init__(
        self,
        dx_grid: np.ndarray,
        dy_grid: np.ndarray,
        *,
        header: Optional[LcfHeader] = None,
        source: Optional[str] = None,
    ):
        self.header = header
        self.source = source
        dx, dy = expand_displacement_grid(
            np.asarray(dx_grid, dtype=np.float64),
            np.asarray(dy_grid, dtype=np.float64),
            PORTRAIT_WIDTH,
            PORTRAIT_HEIGHT,
        )
        self.dx, self.dy = dx, dy
        ys, xs = np.indices(dx.shape, dtype=np.float32)
        self._map_x = xs + dx
        self._map_y = ys + dy

    @classmethod
    def from_bytes(cls, data: bytes, source: Optional[str] = None) -> "LensCorrectionMap":
        dx_c, dy_c, hdr = decode_lcf_grid(data)
        return cls(dx_c, dy_c, header=hdr, source=source)

    @classmethod
    def from_fff(cls, path: Union[str, Path]) -> "LensCorrectionMap":
        return cls.from_bytes(Path(path).read_bytes(), source=str(path))

    def remap_portrait(self, image: np.ndarray) -> np.ndarray:
        """Undistort a portrait (80 rows x 60 cols) image; keeps the input dtype."""
        import cv2

        if image.shape[:2] != self.dx.shape:
            raise ValueError(f"image shape {image.shape[:2]} != LCF field {self.dx.shape}")
        src = image.astype(np.float32) if np.issubdtype(image.dtype, np.integer) else image
        out = cv2.remap(
            src, self._map_x, self._map_y,
            interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE,
        )
        if np.issubdtype(image.dtype, np.integer):
            info = np.iinfo(image.dtype)
            return np.clip(np.rint(out), info.min, info.max).astype(image.dtype)
        return out

    def remap(self, image: np.ndarray) -> np.ndarray:
        """Undistort a sensor-oriented (60 rows x 80 cols) frame."""
        import cv2

        portrait = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        return cv2.rotate(self.remap_portrait(portrait), cv2.ROTATE_90_COUNTERCLOCKWISE)


def build_synthetic_fff(
    dx_grid: np.ndarray,
    dy_grid: np.ndarray,
    *,
    fraction_bits: int = 10,
    byte_order: str = "<",
) -> bytes:
    """Minimal FFF with one tag-0x14 LCF entry (for tests)."""
    rows, cols = dx_grid.shape
    width, height = cols * 2, rows
    grid = np.empty((height, width), dtype=np.float64)
    grid[:, 0::2] = dx_grid
    grid[:, 1::2] = dy_grid
    payload = np.rint(grid * (1 << fraction_bits)).astype(f"{byte_order}i2").tobytes()

    header = bytearray(_MAP_HEADER_SIZE)
    struct.pack_into(f"{byte_order}3H", header, 0, 2, width, height)
    header[29] = fraction_bits & 0xFF

    index_offset, entry_offset = 64, 512
    blob = bytearray(entry_offset)
    blob[0:4] = FFF_MAGIC
    struct.pack_into(">III", blob, 20, 100, index_offset, 1)
    struct.pack_into(
        ">HHIIII", blob, index_offset,
        FFF_TAG_LCF_MAP, 2, 100, 1, entry_offset, _MAP_HEADER_SIZE + len(payload),
    )
    return bytes(blob) + bytes(header) + payload
