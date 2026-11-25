"""Synthetic FileIO framing + read-only control JSON (no hardware)."""

import json
import queue
import struct
import threading
import time

import pytest

from flir_usb import FileIOFetchError, FlirUsbStreamer
from iap2 import (
    CONFIG_HEADER_SIZE,
    CONFIG_MAGIC,
    FILEIO_HEADER_SIZE,
    FILEIO_MAGIC,
    FileIOPacketParser,
    extract_control_json,
    file_closed_status_message,
    open_file_error_message,
    pack_config_packet,
    pack_fileio_header,
    pack_fileio_packet,
    parse_config_packet,
    parse_file_closed,
    parse_fileio_header,
    parse_open_file_status,
    send_close_file,
    send_open_file,
    send_read_file,
)


def test_send_open_file_refuses_non_read_modes():
    with pytest.raises(ValueError, match="disabled"):
        send_open_file("calib.rsc", mode="w")
    with pytest.raises(ValueError, match="disabled"):
        send_open_file("calib.rsc", mode="rw")


def test_control_builders_use_ea_prefix_and_0x1cc():
    from iap2 import send_open_file_chunks

    open_pl = send_open_file("calib.rsc")
    assert open_pl[:2] == struct.pack(">H", 1)  # EA session prefix
    assert open_pl[2:6] == struct.pack("<I", CONFIG_MAGIC)
    ver, raw = parse_config_packet(open_pl[2:])
    assert ver == 1
    assert json.loads(raw.decode()) == {
        "type": "openFile",
        "data": {"path": "calib.rsc", "mode": "r"},
    }
    # Wire shape: two EA packets (16-byte header, then JSON), matching sled writer.
    chunks = send_open_file_chunks("calib.rsc")
    assert len(chunks) == 2
    assert chunks[0] == struct.pack(">H", 1) + open_pl[2 : 2 + CONFIG_HEADER_SIZE]
    assert chunks[1][:2] == struct.pack(">H", 1)
    assert chunks[1][2:].startswith(b'{"type":"openFile"')
    assert extract_control_json(send_read_file(3)) == {
        "type": "readFile",
        "data": {"streamIdentifier": 3},
    }
    assert extract_control_json(send_close_file(3)) == {
        "type": "closeFile",
        "data": {"streamIdentifier": 3},
    }


def test_config_packet_round_trip_and_bad_crc():
    body = b'{"type":"sledInformation","data":{"operatingMode":"operational"}}\x00'
    pkt = pack_config_packet(body)
    assert parse_config_packet(pkt)[1] == body.rstrip(b"\x00")
    assert extract_control_json(pkt)["type"] == "sledInformation"
    # Live sled→host shape: EA prefix + JSON chunk (0x1CC may be a prior packet).
    wire = struct.pack(">H", 1) + body
    assert extract_control_json(wire)["type"] == "sledInformation"
    # EA prefix + full 0x1CC packet (host→sled / combined form).
    assert extract_control_json(struct.pack(">H", 1) + pkt)["type"] == "sledInformation"
    bad = bytearray(pkt)
    bad[12] ^= 0xFF
    assert parse_config_packet(bytes(bad)) is None


def test_header_parse_round_trip():
    payload = b"hello-fileio"
    packet = pack_fileio_packet(payload, stream_identifier=7, protocol_version=1)
    assert len(packet) == FILEIO_HEADER_SIZE + len(payload)
    hdr = parse_fileio_header(packet[:FILEIO_HEADER_SIZE])
    assert hdr is not None
    assert hdr.magic == FILEIO_MAGIC
    assert hdr.protocol_version == 1
    assert hdr.payload_length == len(payload)
    assert hdr.stream_identifier == 7
    parsed = FileIOPacketParser().feed(packet)
    assert len(parsed) == 1
    assert parsed[0].payload == payload


def test_bad_magic_is_skipped():
    good = pack_fileio_packet(b"abc", stream_identifier=1)
    junk = b"\x00\x01\x02\x03" + good
    packets = FileIOPacketParser().feed(junk)
    assert len(packets) == 1 and packets[0].payload == b"abc"


def test_bad_header_crc_is_skipped():
    good = bytearray(pack_fileio_packet(b"xyz", stream_identifier=2))
    bad = bytearray(good)
    bad[20] ^= 0xFF  # corrupt header CRC
    packets = FileIOPacketParser().feed(bytes(bad) + bytes(good))
    assert len(packets) == 1 and packets[0].payload == b"xyz"


def test_bad_payload_crc_is_dropped():
    packet = bytearray(pack_fileio_packet(b"payload", stream_identifier=3))
    packet[FILEIO_HEADER_SIZE] ^= 0xFF
    # A second good packet should still parse after the corrupt one is discarded.
    good = pack_fileio_packet(b"ok", stream_identifier=3)
    packets = FileIOPacketParser().feed(bytes(packet) + good)
    assert [p.payload for p in packets] == [b"ok"]


def test_multi_chunk_reassembly_and_zero_length_eof():
    chunks = [b"AAA", b"BBBB", b"CCCCCCCC"]
    stream_id = 9
    wire = b"".join(pack_fileio_packet(c, stream_id) for c in chunks)
    wire += pack_fileio_packet(b"", stream_id)  # EOF
    parser = FileIOPacketParser()
    # Split across awkward boundaries (including mid-header).
    packets = []
    for i in range(0, len(wire), 5):
        packets.extend(parser.feed(wire[i : i + 5]))
    assert [p.payload for p in packets] == chunks + [b""]
    assert packets[-1].payload_length == 0
    assert b"".join(p.payload for p in packets if p.payload_length) == b"".join(chunks)


@pytest.mark.parametrize(
    "stream_id,fragment",
    [
        (-13, "permission"),
        (-2, "not found"),
        (-3, "already open"),
        (-1, "invalid mode"),
    ],
)
def test_open_file_status_error_codes(stream_id, fragment):
    # Sled may emit doubles via __aeabi_i2d — accept float form too.
    msg = {
        "type": "openFileStatus",
        "data": {"path": "nope.bin", "mode": "r", "streamIdentifier": float(stream_id)},
    }
    parsed = parse_open_file_status(msg)
    assert parsed["ok"] is False
    assert parsed["stream_identifier"] == stream_id
    assert fragment in parsed["error"]
    assert fragment in open_file_error_message(stream_id)


def test_open_file_status_success_and_file_closed():
    opened = parse_open_file_status(
        {"type": "openFileStatus", "data": {"path": "calib.rsc", "mode": "r", "streamIdentifier": 4.0}}
    )
    assert opened["ok"] and opened["stream_identifier"] == 4
    closed = parse_file_closed(
        {"type": "fileClosed", "data": {"streamIdentifier": 4, "status": -5}}
    )
    assert closed["ok"] is False
    assert "CRC" in closed["error"]
    assert "CRC" in file_closed_status_message(-5)


def test_pack_header_rejects_oversized_payload():
    with pytest.raises(ValueError):
        pack_fileio_header(b"x" * (131072 + 1), stream_identifier=1)


class _FakeLink:
    def __init__(self):
        self.link_ready = threading.Event()
        self.link_ready.set()
        self.control_messages = queue.Queue()
        self.sent = []
        self.sled_info = {"operatingMode": "operational"}

    def drain_control_messages(self):
        while True:
            try:
                self.control_messages.get_nowait()
            except queue.Empty:
                return

    def send_open_file(self, path, mode="r"):
        self.sent.append(("open", path, mode))

    def send_read_file(self, stream_id):
        self.sent.append(("read", stream_id))

    def send_close_file(self, stream_id):
        self.sent.append(("close", stream_id))

    def wait_for_control(self, msg_type, timeout_s=1.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                msg = self.control_messages.get(timeout=0.02)
            except queue.Empty:
                continue
            if msg.get("type") == msg_type:
                return msg
        return None


class _FakeTransport:
    def __init__(self, link):
        self.link_mgr = link

    def connect(self, timeout_sec=15.0):
        return True

    def read_fileio(self, size=4096, timeout_ms=50):
        return b""

    def read_frame(self, size=65536, timeout_ms=500):
        return b""

    def close(self):
        return None


def test_fetch_file_timeout_path():
    link = _FakeLink()
    streamer = FlirUsbStreamer(mock_mode=False)
    streamer.transport = _FakeTransport(link)
    streamer.device_connected = True
    # No openFileStatus is ever queued → timeout.
    with pytest.raises(FileIOFetchError, match="timed out waiting for openFileStatus"):
        streamer.fetch_file("missing.rsc", timeout_s=0.3)
    assert ("open", "missing.rsc", "r") in link.sent


def test_fetch_file_assembles_chunks_until_eof():
    link = _FakeLink()
    streamer = FlirUsbStreamer(mock_mode=False)
    streamer.transport = _FakeTransport(link)
    streamer.device_connected = True

    def respond():
        time.sleep(0.05)
        link.control_messages.put(
            {
                "type": "openFileStatus",
                "data": {"path": "x.bin", "mode": "r", "streamIdentifier": 11},
            }
        )
        # Wait until fetch arms the demuxer and issues the first readFile.
        for _ in range(100):
            if any(s[0] == "read" for s in link.sent):
                break
            time.sleep(0.01)
        streamer._handle_fileio_bytes(pack_fileio_packet(b"hel", 11))
        for _ in range(100):
            if sum(1 for s in link.sent if s[0] == "read") >= 2:
                break
            time.sleep(0.01)
        streamer._handle_fileio_bytes(pack_fileio_packet(b"lo", 11))
        for _ in range(100):
            if sum(1 for s in link.sent if s[0] == "read") >= 3:
                break
            time.sleep(0.01)
        streamer._handle_fileio_bytes(pack_fileio_packet(b"", 11))

    t = threading.Thread(target=respond, daemon=True)
    t.start()
    data = streamer.fetch_file("x.bin", timeout_s=5.0)
    t.join(timeout=2.0)
    assert data == b"hello"
    assert link.sent[0] == ("open", "x.bin", "r")
    assert all(s[0] != "close" for s in link.sent), "EOF path should not need closeFile"
