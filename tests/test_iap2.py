"""iAP2 link framing and the handshake / JSON handling in IAP2LinkManager."""

import json
import queue
import struct

from conftest import SAMPLE_SLED_INFO
from iap2 import (
    LINK_DETECT,
    IAP2LinkManager,
    build_stop_ea_session_message,
    make_iap2_packet,
    parse_iap2_packet,
    pop_iap2_packet,
    send_open_file,
)


def test_packet_round_trip_and_checksums():
    pkt = make_iap2_packet(ctrl=0x40, seq=3, ack=7, session=1, payload=b"\x40\x40\x00\x06\x1d\x02")
    assert pkt[:2] == b"\xff\x5a"
    assert (pkt[2] << 8 | pkt[3]) == len(pkt)
    assert sum(pkt[:9]) & 0xFF == 0
    assert sum(pkt[9:]) & 0xFF == 0
    assert parse_iap2_packet(pkt) == (0x40, 3, 7, 1, b"\x40\x40\x00\x06\x1d\x02")


def test_ack_only_packet_has_no_payload():
    pkt = make_iap2_packet(ctrl=0x40, seq=1, ack=0)
    assert len(pkt) == 9
    assert parse_iap2_packet(pkt)[4] == b""


def test_stop_ea_session_message_is_ea01():
    msg = build_stop_ea_session_message(1)
    assert msg[:6] == bytes([0x40, 0x40, 0x00, 0x0C, 0xEA, 0x01])
    assert msg == bytes([0x40, 0x40, 0x00, 0x0C, 0xEA, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01])


def test_pop_iap2_packet_drains_coalesced_reads():
    """EP 0x81 often returns ACK + DATA in one buffer — both must be recovered."""
    ack = make_iap2_packet(ctrl=0x40, seq=10, ack=3)
    status_json = (
        b'{"type":"openFileStatus","data":'
        b'{"path":"version_op.txt","mode":"r","streamIdentifier":5}}'
    )
    # EA session prefix 0x0001 + raw JSON (cfg=none path used by live sled replies).
    data = make_iap2_packet(
        ctrl=0x40, seq=11, ack=3, session=2, payload=struct.pack(">H", 1) + status_json
    )
    buf = bytearray(ack + data)
    first = pop_iap2_packet(buf)
    second = pop_iap2_packet(buf)
    assert first is not None and first[4] == b"" and first[2] == 3
    assert second is not None and b"openFileStatus" in second[4]
    assert buf == bytearray()
    assert pop_iap2_packet(buf) is None


def test_coalesced_ack_and_open_file_status_reach_control_queue():
    """Regression: openFileStatus glued to the openFile ACK must not be dropped."""
    incoming = queue.Queue()
    incoming.put(make_iap2_packet(ctrl=0x80, seq=5, ack=0, payload=b"\x01\x02\x03"))
    incoming.put(make_iap2_packet(ctrl=0x40, seq=6, ack=1, session=1, payload=b"\x40\x40\x00\x06\xae\x00"))
    incoming.put(make_iap2_packet(ctrl=0x40, seq=7, ack=1, session=1, payload=b"\x40\x40\x00\x06\xae\x03"))

    written = []
    open_payload = send_open_file("version_op.txt", mode="r")

    def fake_read(pipe, size):
        try:
            return incoming.get(timeout=0.05)
        except queue.Empty:
            return b""

    def fake_write(data):
        written.append(data)
        parsed = parse_iap2_packet(data)
        if not parsed:
            return
        _c, seq, _ack, _sess, payload = parsed
        # After host sends openFile, reply with ACK+openFileStatus in ONE USB read.
        if payload == open_payload:
            status_json = (
                b'{"type":"openFileStatus","data":'
                b'{"path":"version_op.txt","mode":"r","streamIdentifier":5}}'
            )
            ack = make_iap2_packet(ctrl=0x40, seq=20, ack=seq)
            status = make_iap2_packet(
                ctrl=0x40,
                seq=21,
                ack=seq,
                session=2,
                payload=struct.pack(">H", 1) + status_json,
            )
            incoming.put(ack + status)

    mgr = IAP2LinkManager(write_fn=fake_write, read_fn=fake_read, log_fn=lambda _m: None)
    mgr.start()
    try:
        assert mgr.link_ready.wait(timeout=5.0)
        mgr.send_open_file("version_op.txt", mode="r")
        msg = mgr.wait_for_control("openFileStatus", timeout_s=3.0)
        assert msg is not None
        assert msg["data"]["streamIdentifier"] == 5
        assert msg["data"]["path"] == "version_op.txt"
    finally:
        mgr.running = False
        mgr.join(timeout=2.0)


def test_handshake_and_sled_information():
    incoming = queue.Queue()
    incoming.put(make_iap2_packet(ctrl=0x80, seq=5, ack=0, payload=b"\x01\x02\x03"))
    # IdentificationAccepted then PowerSourceUpdate — cue to start EA command session.
    incoming.put(make_iap2_packet(ctrl=0x40, seq=6, ack=1, session=1, payload=b"\x40\x40\x00\x06\xae\x00"))
    incoming.put(make_iap2_packet(ctrl=0x40, seq=7, ack=1, session=1, payload=b"\x40\x40\x00\x06\xae\x03"))
    sled_body = json.dumps({"type": "sledInformation", "data": SAMPLE_SLED_INFO}).encode() + b"\x00"
    sled_wire = struct.pack(">H", 1) + sled_body
    incoming.put(make_iap2_packet(ctrl=0x40, seq=8, ack=2, session=2, payload=sled_wire))
    battery = (
        struct.pack(">H", 1)
        + json.dumps({"type": "batteryVoltageUpdate", "data": {"voltage": 3.9, "percentage": 80}}).encode()
        + b"\x00"
    )
    incoming.put(make_iap2_packet(ctrl=0x40, seq=9, ack=2, session=2, payload=battery))

    written = []

    def fake_read(pipe, size):
        try:
            return incoming.get(timeout=0.05)
        except queue.Empty:
            return b""

    mgr = IAP2LinkManager(write_fn=written.append, read_fn=fake_read, log_fn=lambda _m: None)
    mgr.start()
    try:
        assert mgr.link_ready.wait(timeout=5.0)
        deadline_ok = False
        for _ in range(100):
            if mgr.battery_percentage == 80 and mgr.sled_info == SAMPLE_SLED_INFO:
                deadline_ok = True
                break
            mgr.link_ready.wait(0.02)
        assert deadline_ok
    finally:
        mgr.running = False
        mgr.join(timeout=2.0)

    assert written[0] == LINK_DETECT
    parsed = [parse_iap2_packet(p) for p in written if p[:2] == b"\xff\x5a"]
    syn_ack = parsed[0]
    assert syn_ack[0] == 0xC0 and syn_ack[2] == 5 and syn_ack[4] == b"\x01\x02\x03"
    assert any(p[4] == b"\x40\x40\x00\x06\x1d\x02" for p in parsed), "0x1D02 identification not sent"
    assert any(
        p[4] and p[4][:6] == b"\x40\x40\x00\x11\xea\x00" for p in parsed
    ), "0xEA00 Protocol 0 not sent"
    assert mgr.sled_info == SAMPLE_SLED_INFO
    assert mgr.battery_voltage == 3.9


def test_stale_ea_data_stops_then_restarts_command_session():
    """Stale EA traffic: send 0xEA01 then 0xEA00 before continuing."""
    incoming = queue.Queue()
    incoming.put(make_iap2_packet(ctrl=0x80, seq=1, ack=0, payload=b"\x01\x02\x03"))
    # Skip AE00/AE03; deliver EA-framed battery as if the prior host left the session up.
    battery = (
        struct.pack(">H", 1)
        + json.dumps({"type": "batteryVoltageUpdate", "data": {"voltage": 3.7, "percentage": 50}}).encode()
        + b"\x00"
    )
    incoming.put(make_iap2_packet(ctrl=0x40, seq=2, ack=1, session=2, payload=battery))

    written = []

    def fake_read(pipe, size):
        try:
            return incoming.get(timeout=0.05)
        except queue.Empty:
            return b""

    mgr = IAP2LinkManager(write_fn=written.append, read_fn=fake_read, log_fn=lambda _m: None)
    mgr.start()
    try:
        assert mgr.link_ready.wait(timeout=5.0)
    finally:
        mgr.running = False
        mgr.join(timeout=2.0)

    payloads = [
        parse_iap2_packet(p)[4]
        for p in written
        if p[:2] == b"\xff\x5a" and parse_iap2_packet(p) and parse_iap2_packet(p)[4]
    ]
    stop_idxs = [i for i, p in enumerate(payloads) if p[:6] == bytes([0x40, 0x40, 0x00, 0x0C, 0xEA, 0x01])]
    start_idxs = [i for i, p in enumerate(payloads) if p[:6] == bytes([0x40, 0x40, 0x00, 0x11, 0xEA, 0x00])]
    assert stop_idxs, "forced 0xEA01 not sent"
    assert start_idxs, "forced 0xEA00 not sent"
    assert stop_idxs[0] < start_idxs[0], "0xEA01 must precede forced 0xEA00"
    assert mgr.battery_voltage == 3.7
