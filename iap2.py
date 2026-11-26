"""
Apple iAP2 link-layer helpers and control session message builders.
Shared by WinUSB and libusb transport backends.
"""

import json
import queue
import struct
import threading
import time
import zlib
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

LINK_DETECT = bytes([0xFF, 0x55, 0x02, 0x00, 0xEE, 0x10])

EP_IAP2_IN = 0x81
EP_IAP2_OUT = 0x02
EP_FILEIO_IN = 0x83
EP_FILEIO_OUT = 0x04
EP_FRAME_IN = 0x85

FILEIO_MAGIC = 0x00005510
FILEIO_HEADER_SIZE = 24
FILEIO_MAX_PAYLOAD = 131072  # 128 KiB

# Command-session (EAP protocol 0) wraps JSON in a 16-byte config header (magic 0x1CC).
CONFIG_MAGIC = 0x000001CC
CONFIG_HEADER_SIZE = 16
CONFIG_PROTOCOL_VERSION = 1

# EA payloads on the wire: big-endian session id (uint16) + body. Live sled-to-host
# command traffic uses iAP2 session=2 with EA prefix 0x0001, then JSON (the 0x1CC
# header may arrive as a separate preceding chunk). Host-to-sled uses the same envelope.
IAP2_COMMAND_SESSION = 2
EA_COMMAND_SESSION_ID = 1
EA_SESSION_PREFIX_SIZE = 2

# batteryChargingStateUpdate wire strings -> GUI labels (match case-insensitively).
CHARGING_STATE_LABELS = {
    "statedeveloper": "Developer",
    "statenocharging": "Not Charging",
    "statemanagedbatterycharging": "Managed Charging",
    "statemanagedbatterychargingonly": "Managed Charging Only",
    "statechargingsmartphone": "Charging Phone",
    "statechargingsmartphoneonly": "Charging Phone Only",
    "statehandlefault": "Handle Fault",
    "statehandlefaultheat": "Handle Fault (Heat)",
    "statehandlefaultbadcharger": "Handle Fault (Bad Charger)",
    "statechargingsmartphonefaultheat": "Phone Charge Fault (Heat)",
    "invalid": "Invalid",
}


def format_charging_state(raw: Optional[str]) -> str:
    """Map a wire chargingState string to a short GUI label; unknown values pass through."""
    if raw is None:
        return "Unknown"
    text = str(raw).strip()
    if not text:
        return "Unknown"
    return CHARGING_STATE_LABELS.get(text.lower(), text)


def pack_ea_payload(ea_session_id: int, body: bytes) -> bytes:
    """Build an EA session payload: BE session id + body bytes."""
    return struct.pack(">H", int(ea_session_id) & 0xFFFF) + body


def strip_ea_prefix(payload: bytes) -> bytes:
    """Remove the leading BE EA session id when present."""
    if len(payload) < EA_SESSION_PREFIX_SIZE:
        return payload
    return payload[EA_SESSION_PREFIX_SIZE:]

# openFileStatus.streamIdentifier < 0 (errno-style codes from the sled)
OPEN_FILE_ERROR_MESSAGES = {
    -13: "permission denied or invalid filename characters",
    -2: "file not found",
    -3: "file already open",
    -1: "invalid mode or filesystem error",
}

# fileClosed.status values observed on the wire / mapped to errno-style codes.
FILE_CLOSED_STATUS_MESSAGES = {
    0: "success",
    -2: "bad file descriptor or not found",
    -5: "CRC-32 mismatch (EIO)",
    -28: "no space left on device",
}


def fileio_crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def pack_config_packet(json_payload: bytes, protocol_version: int = CONFIG_PROTOCOL_VERSION) -> bytes:
    """Wrap NUL-terminated JSON in the 16-byte 0x1CC config header (command session)."""
    if not json_payload.endswith(b"\x00"):
        json_payload = json_payload + b"\x00"
    head12 = struct.pack("<III", CONFIG_MAGIC, int(protocol_version), len(json_payload))
    return head12 + struct.pack("<I", fileio_crc32(head12)) + json_payload


def parse_config_packet(data: bytes) -> Optional[Tuple[int, bytes]]:
    """
    Parse a 0x1CC config packet. Returns (protocol_version, json_bytes_without_trailing_nuls)
    or None if magic/CRC/length are invalid.
    """
    if len(data) < CONFIG_HEADER_SIZE:
        return None
    magic, protocol_version, payload_length, header_crc = struct.unpack(
        "<IIII", data[:CONFIG_HEADER_SIZE]
    )
    if magic != CONFIG_MAGIC:
        return None
    if fileio_crc32(data[:12]) != header_crc:
        return None
    if payload_length > len(data) - CONFIG_HEADER_SIZE:
        return None
    payload = data[CONFIG_HEADER_SIZE : CONFIG_HEADER_SIZE + payload_length]
    return protocol_version, payload.rstrip(b"\x00")


def extract_control_json(payload: bytes) -> Optional[dict]:
    """Decode command-session EA payload (optional 00 01 prefix, optional 0x1CC, JSON)."""
    candidates = [payload]
    stripped = strip_ea_prefix(payload)
    if stripped != payload:
        candidates.append(stripped)

    for candidate in candidates:
        parsed = parse_config_packet(candidate)
        if parsed is not None:
            _ver, raw = parsed
            if not raw:
                continue
            try:
                return json.loads(raw.decode("utf-8"))
            except Exception:
                pass
        # Tolerant path: JSON may sit after the EA prefix without a 0x1CC header
        # on this chunk (sled often sends header and JSON as separate EA packets).
        try:
            text = candidate.decode("utf-8", errors="ignore").strip("\x00\r\n ")
        except Exception:
            continue
        if "{" not in text:
            continue
        text = text[text.find("{") : text.rfind("}") + 1]
        try:
            return json.loads(text)
        except Exception:
            continue
    return None


def make_iap2_packet(
    ctrl: int, seq: int, ack: int, session: int = 0, payload: bytes = b""
) -> bytes:
    total_len = 9 + (len(payload) + 1 if payload else 0)
    hdr_no_chk = bytes([
        0xFF, 0x5A,
        (total_len >> 8) & 0xFF,
        total_len & 0xFF,
        ctrl & 0xFF,
        seq & 0xFF,
        ack & 0xFF,
        session & 0xFF,
    ])
    hdr_chk = (-sum(hdr_no_chk)) & 0xFF
    hdr = hdr_no_chk + bytes([hdr_chk])
    if payload:
        pld_chk = (-sum(payload)) & 0xFF
        return hdr + payload + bytes([pld_chk])
    return hdr


def parse_iap2_packet(data: bytes) -> Optional[Tuple[int, int, int, int, bytes]]:
    if len(data) < 9 or data[0] != 0xFF or data[1] != 0x5A:
        return None
    total_len = (data[2] << 8) | data[3]
    if len(data) < total_len:
        return None
    if (sum(data[:9]) & 0xFF) != 0:
        return None
    ctrl, seq, ack, session = data[4], data[5], data[6], data[7]
    payload = b""
    if total_len > 9:
        payload = data[9 : total_len - 1]
    return ctrl, seq, ack, session, payload


def pop_link_detect(buf: bytearray) -> bool:
    """Consume a 6-byte FF 55 link-detect probe from the front of *buf*."""
    if len(buf) >= 6 and buf[0] == 0xFF and buf[1] == 0x55:
        del buf[:6]
        return True
    return False


def pop_iap2_packet(buf: bytearray) -> Optional[Tuple[int, int, int, int, bytes]]:
    """
    Extract one complete iAP2 packet from *buf*.

    WinUSB/libusb reads of EP 0x81 may contain multiple back-to-back packets
    (e.g. pure ACK + openFileStatus). Callers that only parse the first packet
    and discard the rest silently drop FileIO control replies.
    """
    while len(buf) >= 9:
        # Leave FF 55 link-detect for pop_link_detect; otherwise resync to FF 5A.
        if buf[0] == 0xFF and buf[1] == 0x55:
            return None
        if not (buf[0] == 0xFF and buf[1] == 0x5A):
            idx = -1
            for i in range(len(buf) - 1):
                if buf[i] == 0xFF and buf[i + 1] == 0x5A:
                    idx = i
                    break
                if buf[i] == 0xFF and buf[i + 1] == 0x55:
                    idx = i
                    break
            if idx < 0:
                del buf[:-1]
                return None
            if idx > 0:
                del buf[:idx]
            if buf[0] == 0xFF and buf[1] == 0x55:
                return None
            if len(buf) < 9:
                return None

        total_len = (buf[2] << 8) | buf[3]
        if total_len < 9 or total_len > 8192:
            del buf[0]
            continue
        if len(buf) < total_len:
            return None
        raw = bytes(buf[:total_len])
        parsed = parse_iap2_packet(raw)
        if parsed is None:
            del buf[0]
            continue
        del buf[:total_len]
        return parsed
    return None


def build_identification_message() -> bytes:
    return bytes([0x40, 0x40, 0x00, 0x06, 0x1D, 0x02])


def build_start_command_session_message() -> bytes:
    return bytes([
        0x40, 0x40, 0x00, 0x11,
        0xEA, 0x00,
        0x00, 0x05, 0x00, 0x00, 0x00,
        0x00, 0x06, 0x00, 0x01, 0x00, 0x01,
    ])


def build_stop_ea_session_message(session_id: int = EA_COMMAND_SESSION_ID) -> bytes:
    """
    StopExternalAccessoryProtocolSession (0xEA01).

    Needed to tear down the command EA session so the next start + FileIO
    SET_INTERFACE can produce a fresh sledInformation. Idling FileIO with
    SET_INTERFACE(1,0) alone does not do that.
    """
    sid = int(session_id) & 0xFFFF
    return bytes([
        0x40, 0x40, 0x00, 0x0C,
        0xEA, 0x01,
        # LEN=6, ID=0 (session identifier), data = BE session id
        0x00, 0x06, 0x00, 0x00, (sid >> 8) & 0xFF, sid & 0xFF,
    ])


def build_start_fileio_session_message() -> bytes:
    return bytes([
        0x40, 0x40, 0x00, 0x11,
        0xEA, 0x00,
        0x00, 0x05, 0x00, 0x00, 0x01,
        0x00, 0x06, 0x00, 0x01, 0x00, 0x02,
    ])


def build_start_frame_session_message() -> bytes:
    return bytes([
        0x40, 0x40, 0x00, 0x11,
        0xEA, 0x00,
        0x00, 0x05, 0x00, 0x00, 0x02,
        0x00, 0x06, 0x00, 0x01, 0x00, 0x03,
    ])


def _control_json_chunks(obj: dict) -> list:
    """Host-to-sled framing: two EA packets (0x1CC header, then JSON), each with BE session id."""
    raw = json.dumps(obj, separators=(",", ":")).encode("utf-8") + b"\x00"
    pkt = pack_config_packet(raw)
    header, payload = pkt[:CONFIG_HEADER_SIZE], pkt[CONFIG_HEADER_SIZE:]
    return [
        pack_ea_payload(EA_COMMAND_SESSION_ID, header),
        pack_ea_payload(EA_COMMAND_SESSION_ID, payload),
    ]


def _control_json_payload(obj: dict) -> bytes:
    """Single EA+0x1CC+JSON blob (unit tests). Live path uses _control_json_chunks."""
    raw = json.dumps(obj, separators=(",", ":")).encode("utf-8") + b"\x00"
    return pack_ea_payload(EA_COMMAND_SESSION_ID, pack_config_packet(raw))


def send_open_file(path: str, mode: str = "r") -> bytes:
    """Build an openFile control payload. Phase 1 refuses any mode other than read."""
    if mode != "r":
        raise ValueError(f"FileIO write/other modes are disabled in this phase (mode={mode!r})")
    return _control_json_payload({"type": "openFile", "data": {"path": path, "mode": mode}})


def send_open_file_chunks(path: str, mode: str = "r") -> list:
    if mode != "r":
        raise ValueError(f"FileIO write/other modes are disabled in this phase (mode={mode!r})")
    return _control_json_chunks({"type": "openFile", "data": {"path": path, "mode": mode}})


def send_read_file(stream_id: int) -> bytes:
    return _control_json_payload(
        {"type": "readFile", "data": {"streamIdentifier": int(stream_id)}}
    )


def send_read_file_chunks(stream_id: int) -> list:
    return _control_json_chunks(
        {"type": "readFile", "data": {"streamIdentifier": int(stream_id)}}
    )


def send_close_file(stream_id: int) -> bytes:
    return _control_json_payload(
        {"type": "closeFile", "data": {"streamIdentifier": int(stream_id)}}
    )


def send_close_file_chunks(stream_id: int) -> list:
    return _control_json_chunks(
        {"type": "closeFile", "data": {"streamIdentifier": int(stream_id)}}
    )


def _as_int(value) -> int:
    """Sled JSON may serialize ints via double (__aeabi_i2d)."""
    return int(value)


def open_file_error_message(stream_id: int) -> str:
    sid = _as_int(stream_id)
    return OPEN_FILE_ERROR_MESSAGES.get(sid, f"openFile failed (streamIdentifier={sid})")


def file_closed_status_message(status: int) -> str:
    st = _as_int(status)
    return FILE_CLOSED_STATUS_MESSAGES.get(st, f"fileClosed status={st}")


def parse_open_file_status(msg: dict) -> dict:
    """Normalize an openFileStatus JSON object."""
    if msg.get("type") != "openFileStatus":
        raise ValueError(f"expected openFileStatus, got {msg.get('type')!r}")
    data = msg.get("data") or {}
    stream_id = _as_int(data["streamIdentifier"])
    return {
        "path": data.get("path", ""),
        "mode": data.get("mode", ""),
        "stream_identifier": stream_id,
        "ok": stream_id > 0,
        "error": None if stream_id > 0 else open_file_error_message(stream_id),
    }


def parse_file_closed(msg: dict) -> dict:
    """Normalize a fileClosed JSON object."""
    if msg.get("type") != "fileClosed":
        raise ValueError(f"expected fileClosed, got {msg.get('type')!r}")
    data = msg.get("data") or {}
    status = _as_int(data.get("status", 0))
    return {
        "stream_identifier": _as_int(data["streamIdentifier"]),
        "status": status,
        "ok": status == 0,
        "error": None if status == 0 else file_closed_status_message(status),
    }


@dataclass(frozen=True)
class FileIOPacket:
    magic: int
    protocol_version: int
    payload_length: int
    stream_identifier: int
    payload_crc: int
    header_crc: int
    payload: bytes
    header_raw: bytes


def pack_fileio_header(
    payload: bytes,
    stream_identifier: int,
    protocol_version: int = 0,
) -> bytes:
    """Build a 24-byte FileIO header for *payload* (synthetic / host-to-sled use)."""
    if len(payload) > FILEIO_MAX_PAYLOAD:
        raise ValueError(f"payloadLength {len(payload)} exceeds {FILEIO_MAX_PAYLOAD}")
    payload_crc = fileio_crc32(payload)
    head20 = struct.pack(
        "<IIIiI",
        FILEIO_MAGIC,
        protocol_version,
        len(payload),
        int(stream_identifier),
        payload_crc,
    )
    return head20 + struct.pack("<I", fileio_crc32(head20))


def pack_fileio_packet(
    payload: bytes,
    stream_identifier: int,
    protocol_version: int = 0,
) -> bytes:
    return pack_fileio_header(payload, stream_identifier, protocol_version) + payload


def parse_fileio_header(header: bytes) -> Optional[FileIOPacket]:
    """Parse and CRC-check a 24-byte header. Returns a packet with empty payload."""
    if len(header) < FILEIO_HEADER_SIZE:
        return None
    header_raw = bytes(header[:FILEIO_HEADER_SIZE])
    magic, protocol_version, payload_length, stream_identifier, payload_crc, header_crc = (
        struct.unpack("<IIIiII", header_raw)
    )
    if magic != FILEIO_MAGIC:
        return None
    if payload_length > FILEIO_MAX_PAYLOAD:
        return None
    if fileio_crc32(header_raw[:20]) != header_crc:
        return None
    return FileIOPacket(
        magic=magic,
        protocol_version=protocol_version,
        payload_length=payload_length,
        stream_identifier=stream_identifier,
        payload_crc=payload_crc,
        header_crc=header_crc,
        payload=b"",
        header_raw=header_raw,
    )


class FileIOPacketParser:
    """Incremental EP 0x83 demuxer: resyncs on bad magic / bad header CRC."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def reset(self) -> None:
        self.buffer.clear()

    def feed(self, chunk: bytes) -> List[FileIOPacket]:
        if chunk:
            self.buffer.extend(chunk)
        packets: List[FileIOPacket] = []
        magic_le = struct.pack("<I", FILEIO_MAGIC)

        while len(self.buffer) >= FILEIO_HEADER_SIZE:
            idx = self.buffer.find(magic_le)
            if idx < 0:
                # Keep a few bytes in case magic is split across reads.
                self.buffer = self.buffer[-(FILEIO_HEADER_SIZE - 1) :]
                break
            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < FILEIO_HEADER_SIZE:
                break

            hdr = parse_fileio_header(self.buffer[:FILEIO_HEADER_SIZE])
            if hdr is None:
                # Bad header CRC or invalid length — slide one byte and resync.
                del self.buffer[0:1]
                continue

            total = FILEIO_HEADER_SIZE + hdr.payload_length
            if len(self.buffer) < total:
                break

            payload = bytes(self.buffer[FILEIO_HEADER_SIZE:total])
            del self.buffer[:total]

            if fileio_crc32(payload) != hdr.payload_crc:
                # Drop this framing attempt; continue scanning after the magic byte.
                # Payload already consumed with a bad CRC — skip emitting it.
                continue

            packets.append(
                FileIOPacket(
                    magic=hdr.magic,
                    protocol_version=hdr.protocol_version,
                    payload_length=hdr.payload_length,
                    stream_identifier=hdr.stream_identifier,
                    payload_crc=hdr.payload_crc,
                    header_crc=hdr.header_crc,
                    payload=payload,
                    header_raw=hdr.header_raw,
                )
            )
        return packets


class IAP2LinkManager(threading.Thread):
    """
    Strict iAP2 negotiator matching test_synced_clean.py:
    SYN -> 0x1D02 -> wait AE00 -> wait AE03 -> 0xEA00 protocol 0 only.
    FileIO/Frame sessions are activated via SET_INTERFACE on bulk endpoints.
    """

    def __init__(self, write_fn, read_fn, log_fn, full_startup: bool = False):
        super().__init__(daemon=True)
        self.write_fn = write_fn
        self.read_fn = read_fn
        self.log_fn = log_fn or (lambda _msg: None)
        self.full_startup = full_startup
        
        self.running = True
        self.link_ready = threading.Event()
        self.trigger_fileio = threading.Event()
        self.trigger_frame = threading.Event()
        self.trigger_stop_command = threading.Event()
        # Outbound control/session payloads are sent on the link thread so host_seq stays consistent.
        self._outbound: queue.Queue = queue.Queue()
        # EP 0x81 may deliver several iAP2 packets per read — keep a stream buffer.
        self._rx_buf = bytearray()
        # Packets observed while waiting for an outbound ACK (must not be dropped).
        self._pending_packets: queue.Queue = queue.Queue()
        # openFileStatus / fileClosed (and other future control replies) land here.
        self.control_messages: queue.Queue = queue.Queue()
        
        self.host_seq = 1
        self.camera_seq = 0
        # iAP2 link session for command EA data (live traffic uses 2; see IAP2_COMMAND_SESSION).
        self.command_session = IAP2_COMMAND_SESSION
        
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.battery_charging_current = 0.0
        self.battery_charging_state = "Unknown"
        self.sled_info: Optional[dict] = None

    def send_control_payload(self, payload: bytes) -> None:
        """Queue one command-session EA payload."""
        self._outbound.put(payload)

    def send_control_chunks(self, chunks: list) -> None:
        """Queue header+JSON EA chunks (firmware wire shape)."""
        for chunk in chunks:
            self._outbound.put(chunk)

    def send_open_file(self, path: str, mode: str = "r") -> None:
        # One DATA packet (EA + 0x1CC + JSON). Splitting header/JSON without flow
        # control often drops the second packet before the camera ACKs.
        self.send_control_payload(send_open_file(path, mode=mode))

    def send_read_file(self, stream_id: int) -> None:
        self.send_control_payload(send_read_file(stream_id))

    def send_close_file(self, stream_id: int) -> None:
        self.send_control_payload(send_close_file(stream_id))

    def wait_for_sled_information(self, timeout_s: float = 10.0) -> Optional[dict]:
        """Block until sledInformation arrives (FileIO control channel ready)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.sled_info is not None:
                return self.sled_info
            time.sleep(0.05)
        return self.sled_info

    def drain_control_messages(self) -> None:
        while True:
            try:
                self.control_messages.get_nowait()
            except queue.Empty:
                return

    def wait_for_control(self, msg_type: str, timeout_s: float) -> Optional[dict]:
        """Wait for the next control JSON message of *msg_type* (drops others)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                msg = self.control_messages.get(timeout=min(0.1, remaining))
            except queue.Empty:
                continue
            if msg.get("type") == msg_type:
                return msg
        return None

    def run(self) -> None:
        ident_msg = build_identification_message()
        start_cmd_msg = build_start_command_session_message()
        stop_cmd_msg = build_stop_ea_session_message(EA_COMMAND_SESSION_ID)

        connected = False
        ident_sent = False
        command_sent = False

        try:
            self.write_fn(LINK_DETECT)
        except Exception:
            pass

        while self.running:
            if self.trigger_stop_command.is_set():
                self.trigger_stop_command.clear()
                pkt = make_iap2_packet(
                    ctrl=0x40,
                    seq=self.host_seq,
                    ack=self.camera_seq,
                    session=1,
                    payload=stop_cmd_msg,
                )
                self.write_fn(pkt)
                self.log_fn(
                    f"[iAP2 SENT] 0xEA01 Stop command session "
                    f"(seq={self.host_seq}, sess={EA_COMMAND_SESSION_ID})"
                )
                self.host_seq = (self.host_seq + 1) & 0xFF
                command_sent = False
                self.sled_info = None
                time.sleep(0.2)
                continue

            if self.trigger_fileio.is_set():
                self.trigger_fileio.clear()
                pkt = make_iap2_packet(
                    ctrl=0x40, seq=self.host_seq, ack=self.camera_seq,
                    session=1, payload=build_start_fileio_session_message()
                )
                self.write_fn(pkt)
                self.log_fn(f"[iAP2 SENT] 0xEA00 Protocol 1 (seq={self.host_seq})")
                self.host_seq += 1
                time.sleep(0.2)
                continue

            if self.trigger_frame.is_set():
                self.trigger_frame.clear()
                pkt = make_iap2_packet(
                    ctrl=0x40, seq=self.host_seq, ack=self.camera_seq,
                    session=1, payload=build_start_frame_session_message()
                )
                self.write_fn(pkt)
                self.log_fn(f"[iAP2 SENT] 0xEA00 Protocol 2 (seq={self.host_seq})")
                self.host_seq += 1
                time.sleep(0.2)
                continue

            try:
                outbound = self._outbound.get_nowait()
            except queue.Empty:
                outbound = None
            if outbound is not None:
                sent_seq = self.host_seq
                pkt = make_iap2_packet(
                    ctrl=0x40,
                    seq=sent_seq,
                    ack=self.camera_seq,
                    session=self.command_session,
                    payload=outbound,
                )
                self.write_fn(pkt)
                self.log_fn(
                    f"[iAP2 SENT] control (seq={sent_seq}, sess={self.command_session}, "
                    f"{len(outbound)} B, head={outbound[:16].hex()})"
                )
                self.host_seq = (sent_seq + 1) & 0xFF
                # Wait for camera ACK before another DATA (MaxOutstanding is often 1).
                # Drain any coalesced packets from the same USB read into _pending_packets
                # so openFileStatus glued to the ACK is not dropped.
                ack_deadline = time.monotonic() + 0.5
                got_ack = False
                while time.monotonic() < ack_deadline:
                    if pop_link_detect(self._rx_buf):
                        try:
                            self.write_fn(LINK_DETECT)
                        except Exception:
                            pass
                        continue
                    parsed_ack = pop_iap2_packet(self._rx_buf)
                    if parsed_ack is None:
                        if got_ack and not self._rx_buf:
                            break
                        try:
                            chunk = self.read_fn(EP_IAP2_IN, 512)
                        except Exception:
                            chunk = b""
                        if chunk:
                            self._rx_buf.extend(chunk)
                            continue
                        if got_ack:
                            break
                        time.sleep(0.01)
                        continue
                    _c, cam_seq, ack_num, _sess, piggy = parsed_ack
                    self.camera_seq = cam_seq
                    # ack_num covers sent_seq if it is sent_seq or "ahead" within half the seq space
                    if ((ack_num - sent_seq) & 0xFF) < 0x7F:
                        if not got_ack:
                            got_ack = True
                            self.log_fn(
                                f"[iAP2] camera ACK={ack_num} (covered sent seq={sent_seq})"
                            )
                    if piggy or _c == 0x80:
                        self._pending_packets.put(parsed_ack)
                if not got_ack:
                    self.log_fn(f"[iAP2] no ACK for seq={sent_seq} within 0.5s")
                continue

            try:
                parsed = self._pending_packets.get_nowait()
            except queue.Empty:
                parsed = None

            if parsed is None:
                if pop_link_detect(self._rx_buf):
                    self.write_fn(LINK_DETECT)
                    continue
                parsed = pop_iap2_packet(self._rx_buf)

            if parsed is None:
                try:
                    chunk = self.read_fn(EP_IAP2_IN, 512)
                except Exception:
                    chunk = b""
                if chunk:
                    self._rx_buf.extend(chunk)
                    continue
                if not connected:
                    try:
                        self.write_fn(LINK_DETECT)
                    except Exception:
                        pass
                time.sleep(0.01)
                continue

            ctrl, seq, ack, session, payload = parsed
            self.camera_seq = seq
            msg_id = 0
            if len(payload) >= 6 and payload[0] == 0x40 and payload[1] == 0x40:
                msg_id = (payload[4] << 8) | payload[5]

            if ctrl == 0x80:
                self.host_seq = 0
                syn_ack = make_iap2_packet(
                    ctrl=0xC0, seq=0, ack=self.camera_seq, session=0, payload=payload
                )
                self.write_fn(syn_ack)
                self.log_fn(f"[iAP2] SYN-ACK (seq=0, ack={self.camera_seq})")
                self.host_seq = 1
                if not ident_sent:
                    time.sleep(0.02)
                    ident_pkt = make_iap2_packet(
                        ctrl=0x40,
                        seq=self.host_seq,
                        ack=self.camera_seq,
                        session=1,
                        payload=ident_msg,
                    )
                    self.write_fn(ident_pkt)
                    ident_sent = True
                    self.log_fn("[iAP2 SENT] 0x1D02 Identification (seq=1)")
                    self.host_seq = 2
                continue

            if (ctrl & 0x40) == 0:
                continue

            connected = True

            if not ident_sent:
                ident_pkt = make_iap2_packet(
                    ctrl=0x40,
                    seq=self.host_seq,
                    ack=self.camera_seq,
                    session=1,
                    payload=ident_msg,
                )
                self.write_fn(ident_pkt)
                ident_sent = True
                self.log_fn(f"[iAP2 SENT] 0x1D02 Identification (seq={self.host_seq})")
                self.host_seq = 2
                continue

            if msg_id == 0xAE00:
                data_ack = make_iap2_packet(
                    ctrl=0x40, seq=self.host_seq, ack=self.camera_seq, session=session
                )
                self.write_fn(data_ack)
                self.log_fn(f"[iAP2 RECV] 0xAE00 -> ACK (seq={self.host_seq})")
                continue

            if msg_id == 0xAE03 and not command_sent:
                # ACK AE03 then start the command EA session. link_ready ONLY after EA00 —
                # never from early battery JSON (stale EA state from a prior plug-in).
                data_ack = make_iap2_packet(
                    ctrl=0x40, seq=self.host_seq, ack=self.camera_seq, session=session
                )
                self.write_fn(data_ack)
                cmd_pkt = make_iap2_packet(
                    ctrl=0x40, seq=self.host_seq, ack=self.camera_seq,
                    session=1, payload=start_cmd_msg,
                )
                self.write_fn(cmd_pkt)
                command_sent = True
                self.log_fn(f"[iAP2 SENT] 0xEA00 Protocol 0 (seq={self.host_seq})")
                self.host_seq = (self.host_seq + 1) & 0xFF
                time.sleep(0.2)
                self.link_ready.set()
                continue

            if payload:
                data_ack = make_iap2_packet(
                    ctrl=0x40, seq=self.host_seq, ack=self.camera_seq, session=session
                )
                self.write_fn(data_ack)
                if payload and msg_id == 0:
                    self.command_session = session
                    # Stale path: EA data before we sent EA00 (previous host left the
                    # session up). Stop then restart so FileIO SET_INTERFACE can emit
                    # sledInformation again.
                    if not command_sent and ident_sent:
                        self.log_fn(
                            "[iAP2] EA data before EA00 — stopping stale session then EA00"
                        )
                        stop_pkt = make_iap2_packet(
                            ctrl=0x40,
                            seq=self.host_seq,
                            ack=self.camera_seq,
                            session=1,
                            payload=stop_cmd_msg,
                        )
                        self.write_fn(stop_pkt)
                        self.log_fn(
                            f"[iAP2 SENT] 0xEA01 Stop command session "
                            f"(seq={self.host_seq}, forced)"
                        )
                        self.host_seq = (self.host_seq + 1) & 0xFF
                        time.sleep(0.3)
                        cmd_pkt = make_iap2_packet(
                            ctrl=0x40,
                            seq=self.host_seq,
                            ack=self.camera_seq,
                            session=1,
                            payload=start_cmd_msg,
                        )
                        self.write_fn(cmd_pkt)
                        command_sent = True
                        self.log_fn(
                            f"[iAP2 SENT] 0xEA00 Protocol 0 (seq={self.host_seq}, forced)"
                        )
                        self.host_seq = (self.host_seq + 1) & 0xFF
                        time.sleep(0.2)
                        self.link_ready.set()

                data_obj = extract_control_json(payload)
                if data_obj is None and session == self.command_session:
                    self.log_fn(
                        f"[iAP2 RAW] sess={session} len={len(payload)} "
                        f"json=none head={payload[:16].hex()}"
                    )
                if data_obj is not None:
                    msg_type = data_obj.get("type")
                    data_payload = data_obj.get("data", {})
                    cfg = parse_config_packet(strip_ea_prefix(payload))
                    head = payload[:16].hex()
                    if cfg is not None:
                        self.log_fn(
                            f"[iAP2 RAW] sess={session} len={len(payload)} "
                            f"type={msg_type} cfg_ver={cfg[0]} head={head}"
                        )
                    else:
                        self.log_fn(
                            f"[iAP2 RAW] sess={session} len={len(payload)} "
                            f"type={msg_type} cfg=none head={head}"
                        )
                    if msg_type != "batteryVoltageUpdate":
                        self.log_fn(f"[iAP2 DATA] {json.dumps(data_obj, separators=(',', ':'))}")

                    if msg_type == "batteryVoltageUpdate":
                        self.battery_voltage = data_payload.get("voltage", self.battery_voltage)
                        self.battery_percentage = data_payload.get("percentage", self.battery_percentage)
                    elif msg_type == "batteryChargingCurrentUpdate":
                        self.battery_charging_current = data_payload.get(
                            "chargingCurrent", self.battery_charging_current
                        )
                    elif msg_type == "batteryChargingStateUpdate":
                        self.battery_charging_state = data_payload.get(
                            "chargingState", self.battery_charging_state
                        )
                    elif msg_type == "sledInformation":
                        self.sled_info = data_payload
                    elif msg_type in ("openFileStatus", "fileClosed"):
                        self.control_messages.put(data_obj)
                elif msg_id:
                    self.log_fn(f"[iAP2 RECV] 0x{msg_id:04X} -> ACK (Payload: {payload.hex()})")

    def start_fileio_session(self) -> None:
        self.trigger_fileio.set()

    def start_frame_session(self) -> None:
        self.trigger_frame.set()

    def stop_command_session(self) -> None:
        """Queue 0xEA01 StopExternalAccessoryProtocolSession for the command EA session."""
        self.trigger_stop_command.set()
