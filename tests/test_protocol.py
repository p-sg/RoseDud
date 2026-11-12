"""Rosebud (0xBEEF) frame parser and VoSPI unpacking."""

import struct
import zlib

import numpy as np

from flir_calib import FlirCalibration
from flir_protocol import EXPECTED_THERMAL_SIZE, VOSPI_PACKET_SIZE, RosebudParser


def make_thermal(row_order=range(60), discard_before=None):
    """63 VoSPI packets; pixel value encodes the row number (1000 + row)."""
    packets = []
    for row in row_order:
        pkt = struct.pack(">HH", row, 0) + struct.pack(">80H", *[1000 + row] * 80)
        packets.append(pkt)
    for tel in range(60, 63):
        packets.append(struct.pack(">HH", tel, 0) + bytes([tel]) * 160)
    if discard_before is not None:
        discard = struct.pack(">HH", 0x0F00, 0) + b"\xAA" * 160
        packets.insert(discard_before, discard)
        packets.pop()  # keep the segment at exactly 63 packets
    data = b"".join(packets)
    assert len(data) == EXPECTED_THERMAL_SIZE
    return data


def make_packet(frame_index=42, thermal=None, visual=b"", meta=b'{"shutterState":"open"}\x00'):
    thermal = make_thermal() if thermal is None else thermal
    total = len(thermal) + len(visual) + len(meta)
    hdr = struct.pack("<IIIIII", 0x0000BEEF, frame_index, total, len(thermal), len(visual), len(meta))
    hdr += struct.pack("<I", zlib.crc32(hdr) & 0xFFFFFFFF)
    return hdr + thermal + visual + meta


def test_single_frame():
    meta = b'{"shutterState":"closed","shutterTemperature":298.15,"ffcState":"FFC_RAD_VALID"}\x00'
    frames = RosebudParser(FlirCalibration()).feed(make_packet(meta=meta))
    assert len(frames) == 1
    f = frames[0]
    assert f.frame_index == 42
    assert f.shutter_state == "closed"
    assert abs(f.shutter_temp_k - 298.15) < 1e-6
    assert f.ffc_state == "FFC_RAD_VALID"
    assert f.thermal_raw.shape == (60, 80)
    assert f.thermal_raw[0, 0] == 1000 and f.thermal_raw[59, 79] == 1059
    assert set(f.telemetry) == {"telemetry_row_0", "telemetry_row_1", "telemetry_row_2"}


def test_split_chunks_and_garbage_resync():
    parser = RosebudParser(FlirCalibration())
    stream = b"\x00garbage\xef\xbe" + make_packet(1) + make_packet(2)
    frames = []
    for i in range(0, len(stream), 500):
        frames += parser.feed(stream[i:i + 500])
    assert [f.frame_index for f in frames] == [1, 2]


def test_bad_header_crc_is_skipped():
    bad = bytearray(make_packet(7))
    bad[24] ^= 0xFF
    frames = RosebudParser(FlirCalibration()).feed(bytes(bad) + make_packet(8))
    assert [f.frame_index for f in frames] == [8]


def test_rows_placed_by_packet_number():
    order = list(range(59, -1, -1))
    f = RosebudParser(FlirCalibration()).feed(make_packet(thermal=make_thermal(order)))[0]
    assert np.array_equal(f.thermal_raw[:, 0], np.arange(1000, 1060))


def test_discard_packets_ignored():
    thermal = make_thermal(discard_before=10)
    f = RosebudParser(FlirCalibration()).feed(make_packet(thermal=thermal))[0]
    assert not any(k.startswith("telemetry_row_-") for k in f.telemetry)
    assert f.thermal_raw[10, 0] == 1010
    assert len(thermal) == 63 * VOSPI_PACKET_SIZE
