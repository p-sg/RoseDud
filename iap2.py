"""
Apple iAP2 link-layer helpers and control session message builders.
Shared by WinUSB and libusb transport backends.
"""

import threading
import time
import json
from typing import Callable, Optional, Tuple

LINK_DETECT = bytes([0xFF, 0x55, 0x02, 0x00, 0xEE, 0x10])

EP_IAP2_IN = 0x81
EP_IAP2_OUT = 0x02
EP_FILEIO_IN = 0x83
EP_FRAME_IN = 0x85


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


def build_identification_message() -> bytes:
    return bytes([0x40, 0x40, 0x00, 0x06, 0x1D, 0x02])


def build_start_command_session_message() -> bytes:
    return bytes([
        0x40, 0x40, 0x00, 0x11,
        0xEA, 0x00,
        0x00, 0x05, 0x00, 0x00, 0x00,
        0x00, 0x06, 0x00, 0x01, 0x00, 0x01,
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
        
        self.host_seq = 1
        self.camera_seq = 0
        
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.battery_charging_current = 0.0
        self.battery_charging_state = "Unknown"
        self.sled_info: Optional[dict] = None

    def run(self) -> None:
        ident_msg = build_identification_message()
        start_cmd_msg = build_start_command_session_message()

        connected = False
        ident_sent = False
        command_sent = False

        try:
            self.write_fn(LINK_DETECT)
        except Exception:
            pass

        while self.running:
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
                raw = self.read_fn(EP_IAP2_IN, 512)
            except Exception:
                raw = b""

            if not raw:
                if not connected:
                    try:
                        self.write_fn(LINK_DETECT)
                    except Exception:
                        pass
                time.sleep(0.01)
                continue

            if len(raw) >= 9 and raw[0] == 0xFF and raw[1] == 0x5A:
                parsed = parse_iap2_packet(raw)
                if not parsed:
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
                    cmd_pkt = make_iap2_packet(
                        ctrl=0x40, seq=self.host_seq, ack=self.camera_seq,
                        session=1, payload=start_cmd_msg,
                    )
                    self.write_fn(cmd_pkt)
                    command_sent = True
                    self.log_fn(f"[iAP2 SENT] 0xEA00 Protocol 0 (seq={self.host_seq})")
                    self.host_seq += 1
                    time.sleep(0.2)
                    self.link_ready.set()
                    continue

                if payload:
                    data_ack = make_iap2_packet(
                        ctrl=0x40, seq=self.host_seq, ack=self.camera_seq, session=session
                    )
                    self.write_fn(data_ack)
                    try:
                        text = payload.decode('utf-8').strip('\x00\r\n ')
                        if "{" in text:
                            text = text[text.find('{'):text.rfind('}')+1]
                            try:
                                data_obj = json.loads(text)
                                msg_type = data_obj.get("type")
                                data_payload = data_obj.get("data", {})
                                
                                if msg_type != "batteryVoltageUpdate":
                                    self.log_fn(f"[iAP2 DATA] {text.strip()}")
                                
                                if msg_type == "batteryVoltageUpdate":
                                    self.battery_voltage = data_payload.get("voltage", self.battery_voltage)
                                    self.battery_percentage = data_payload.get("percentage", self.battery_percentage)
                                elif msg_type == "batteryChargingCurrentUpdate":
                                    self.battery_charging_current = data_payload.get("chargingCurrent", self.battery_charging_current)
                                elif msg_type == "batteryChargingStateUpdate":
                                    self.battery_charging_state = data_payload.get("chargingState", self.battery_charging_state)
                                elif msg_type == "sledInformation":
                                    self.sled_info = data_payload
                            except Exception as e:
                                print(f"JSON Parse Error: {e} for string {repr(text)}")

                            # If we are receiving JSON data, the command session is already active!
                            if not self.link_ready.is_set():
                                self.log_fn("[iAP2] Received JSON data; link is already ready!")
                                self.link_ready.set()
                        elif msg_id:
                            self.log_fn(f"[iAP2 RECV] 0x{msg_id:04X} -> ACK (Payload: {payload.hex()})")
                    except Exception:
                        if msg_id:
                            self.log_fn(f"[iAP2 RECV] 0x{msg_id:04X} -> ACK (Payload: {payload.hex()})")

            elif len(raw) == 6 and raw[0] == 0xFF and raw[1] == 0x55:
                self.write_fn(LINK_DETECT)

    def start_fileio_session(self) -> None:
        self.trigger_fileio.set()

    def start_frame_session(self) -> None:
        self.trigger_frame.set()
