"""Logging levels: quiet by default, iAP2 chatter only at DEBUG, --verbose / --debug."""

import json
import logging
import queue

import pytest

from conftest import SAMPLE_SLED_INFO
from iap2 import (
    IAP2LinkManager,
    make_iap2_packet,
    pack_config_packet,
    pack_ea_payload,
    parse_config_header,
)


def _startup_packets(split_header: bool = False) -> list:
    """SYN, IdentificationAccepted, PowerSourceUpdate, then sledInformation on session 2."""
    pkts = [
        make_iap2_packet(ctrl=0x80, seq=5, ack=0, payload=b"\x01\x02\x03"),
        make_iap2_packet(ctrl=0x40, seq=6, ack=1, session=1, payload=b"\x40\x40\x00\x06\xae\x00"),
        make_iap2_packet(ctrl=0x40, seq=7, ack=1, session=1, payload=b"\x40\x40\x00\x06\xae\x03"),
    ]
    body = json.dumps({"type": "sledInformation", "data": SAMPLE_SLED_INFO}).encode()
    cfg = pack_config_packet(body)
    if split_header:
        pkts.append(make_iap2_packet(ctrl=0x40, seq=8, ack=2, session=2, payload=pack_ea_payload(1, cfg[:16])))
        pkts.append(make_iap2_packet(ctrl=0x40, seq=9, ack=2, session=2, payload=pack_ea_payload(1, cfg[16:])))
    else:
        pkts.append(make_iap2_packet(ctrl=0x40, seq=8, ack=2, session=2, payload=pack_ea_payload(1, cfg)))
    return pkts


def _run_startup(packets: list) -> IAP2LinkManager:
    incoming = queue.Queue()
    for p in packets:
        incoming.put(p)

    def fake_read(pipe, size):
        try:
            return incoming.get(timeout=0.05)
        except queue.Empty:
            return b""

    # No log_fn: messages go through the "rosedud.iap2" logger.
    mgr = IAP2LinkManager(write_fn=lambda _d: None, read_fn=fake_read, log_fn=None)
    mgr.start()
    try:
        assert mgr.link_ready.wait(timeout=5.0)
        for _ in range(200):
            if mgr.sled_info is not None:
                break
            mgr.link_ready.wait(0.02)
        assert mgr.sled_info == SAMPLE_SLED_INFO
    finally:
        mgr.running = False
        mgr.join(timeout=2.0)
    return mgr


def test_parse_config_header_reads_version_and_length():
    cfg = pack_config_packet(b'{"type":"x"}')
    assert parse_config_header(cfg[:16]) == (1, len(cfg) - 16)
    assert parse_config_header(cfg[:15]) is None
    bad = bytearray(cfg[:16])
    bad[12] ^= 0xFF  # corrupt the header CRC
    assert parse_config_header(bytes(bad)) is None


def test_header_only_packet_logs_control_header_at_debug(caplog):
    caplog.set_level(logging.DEBUG, logger="rosedud")
    _run_startup(_startup_packets(split_header=True))

    json_len = len(pack_config_packet(
        json.dumps({"type": "sledInformation", "data": SAMPLE_SLED_INFO}).encode()
    )) - 16
    hits = [r for r in caplog.records if "control header" in r.getMessage()]
    assert hits, "header-only packet was not recognized"
    assert hits[0].name == "rosedud.iap2"
    assert hits[0].levelno == logging.DEBUG
    assert hits[0].getMessage() == f"[iAP2] control header (v1), {json_len}-byte JSON follows"
    assert not any("json=none" in r.getMessage() for r in caplog.records)


def test_startup_at_info_has_no_debug_lines(caplog):
    caplog.set_level(logging.INFO, logger="rosedud")
    _run_startup(_startup_packets(split_header=True))
    rosedud = [r for r in caplog.records if r.name.startswith("rosedud")]
    assert not any("[iAP2" in r.getMessage() for r in rosedud)
    assert all(r.levelno >= logging.INFO for r in rosedud)


def test_startup_iap2_lines_are_debug_level(caplog):
    # Same path at DEBUG: the packet lines exist, and all of them are DEBUG.
    caplog.set_level(logging.DEBUG, logger="rosedud")
    _run_startup(_startup_packets())
    iap2 = [r for r in caplog.records if r.name == "rosedud.iap2"]
    assert any("[iAP2 SENT]" in r.getMessage() for r in iap2)
    assert any("[iAP2 DATA]" in r.getMessage() for r in iap2)
    assert all(r.levelno == logging.DEBUG for r in iap2)


def test_log_fn_still_receives_messages(caplog):
    caplog.set_level(logging.DEBUG, logger="rosedud")
    seen = []
    incoming = queue.Queue()
    for p in _startup_packets():
        incoming.put(p)

    def fake_read(pipe, size):
        try:
            return incoming.get(timeout=0.05)
        except queue.Empty:
            return b""

    mgr = IAP2LinkManager(write_fn=lambda _d: None, read_fn=fake_read, log_fn=seen.append)
    mgr.start()
    try:
        assert mgr.link_ready.wait(timeout=5.0)
    finally:
        mgr.running = False
        mgr.join(timeout=2.0)
    assert any(m.startswith("[iAP2 SENT]") for m in seen)
    assert not [r for r in caplog.records if r.name == "rosedud.iap2"]


@pytest.fixture
def clean_rosedud_logger():
    root = logging.getLogger("rosedud")
    level, handlers = root.level, list(root.handlers)
    yield root
    for h in root.handlers:
        if h not in handlers:
            root.removeHandler(h)
    root.setLevel(level)


@pytest.mark.parametrize(
    "argv, verbose",
    [([], False), (["--verbose"], True), (["-v"], True), (["--debug"], True)],
)
def test_verbose_and_debug_select_debug_level(argv, verbose, clean_rosedud_logger):
    import flir_app

    args = flir_app.parse_args(argv)
    assert args.verbose is verbose
    level = flir_app.configure_logging(args.verbose)
    assert level == (logging.DEBUG if verbose else logging.INFO)
    assert clean_rosedud_logger.level == level


def test_configure_logging_adds_one_handler(clean_rosedud_logger):
    import flir_app

    before = len(clean_rosedud_logger.handlers)
    flir_app.configure_logging(False)
    flir_app.configure_logging(True)
    assert len(clean_rosedud_logger.handlers) == before + 1
