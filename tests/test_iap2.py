"""iAP2 link framing and the handshake / JSON handling in IAP2LinkManager."""

import json
import queue

from conftest import SAMPLE_SLED_INFO
from iap2 import LINK_DETECT, IAP2LinkManager, make_iap2_packet, parse_iap2_packet


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


def test_handshake_and_sled_information():
    incoming = queue.Queue()
    incoming.put(make_iap2_packet(ctrl=0x80, seq=5, ack=0, payload=b"\x01\x02\x03"))
    sled_msg = json.dumps({"type": "sledInformation", "data": SAMPLE_SLED_INFO}).encode()
    incoming.put(make_iap2_packet(ctrl=0x40, seq=6, ack=1, session=1, payload=sled_msg))
    battery = json.dumps({"type": "batteryVoltageUpdate", "data": {"voltage": 3.9, "percentage": 80}}).encode()
    incoming.put(make_iap2_packet(ctrl=0x40, seq=7, ack=1, session=1, payload=battery))

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
            if mgr.battery_percentage == 80:
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
    assert mgr.sled_info == SAMPLE_SLED_INFO
    assert mgr.battery_voltage == 3.9
