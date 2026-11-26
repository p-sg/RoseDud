"""
FLIR One Rosebud (0xBEEF) Protocol Parser & Frame Demuxer
Unpacks the 28-byte header, 10,332-byte VoSPI thermal data, visual JPEG, and JSON metadata.
"""

import json
import math
import struct
import zlib
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Tuple
import numpy as np
import cv2

from flir_calib import FlirCalibration


# Protocol Constants
ROSEBUD_MAGIC = 0x0000BEEF      # Magic identifier (0xEF, 0xBE, 0x00, 0x00)
HEADER_SIZE = 28                 # 28 bytes
VOSPI_PACKET_SIZE = 164          # 2B ID + 2B CRC + 160B Data (80 pixels * 2B)
VOSPI_PACKET_COUNT = 63          # 60 image rows + 3 telemetry rows
EXPECTED_THERMAL_SIZE = VOSPI_PACKET_SIZE * VOSPI_PACKET_COUNT  # 10,332 bytes
VOSPI_TELEMETRY_WORDS = 80

# Lepton Engineering Datasheet Rev 200 Table 3 — FFC State (Row A status bits 4:5).
VOSPI_FFC_STATE_NAMES = {
    0: "never_commanded",
    1: "imminent",
    2: "in_progress",
    3: "complete",
}

def _telemetry_words_from_payload(payload: bytes | str) -> Optional[list[int]]:
    """Unpack 80 big-endian uint16 words from raw bytes or a hex string."""
    if isinstance(payload, str):
        try:
            raw = bytes.fromhex(payload)
        except ValueError:
            return None
    else:
        raw = payload
    if len(raw) < VOSPI_TELEMETRY_WORDS * 2:
        return None
    return list(struct.unpack(f">{VOSPI_TELEMETRY_WORDS}H", raw[: VOSPI_TELEMETRY_WORDS * 2]))


def _u32_low_word_first(lo: int, hi: int) -> int:
    """Assemble a uint32 where the first telemetry word is the low half.

    Live Gen 1 capture: uptime increases ~1000 per second only with this
    packing, and the high word was seen rolling over. The datasheet does not
    state 32-bit word order; each 16-bit word is big-endian on the wire.
    """
    return ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)


def _kelvin_x100(word: int) -> Tuple[int, float]:
    ck = int(word) & 0xFFFF
    return ck, ck / 100.0


def _words_hex(words: list[int], start: int, count: int) -> str:
    return "".join(f"{w:04x}" for w in words[start : start + count])


# Video output format enum in telemetry words 72-73.
VOSPI_VIDEO_OUTPUT_FORMAT_NAMES = {
    0: "RAW8",
    1: "RAW10",
    2: "RAW12",
    3: "RGB888",
    4: "RGB666",
    5: "RGB565",
    6: "YUV422_8BIT",
    7: "RAW14",
    8: "YUV422_10BIT",
    9: "USER_DEFINED",
}


def format_duration_ms(ms: int) -> str:
    """Humanize a millisecond counter as it grows (s -> m s -> h m s -> d h m)."""
    total_ms = max(0, int(ms))
    total_s, rem_ms = divmod(total_ms, 1000)
    if total_s < 60:
        return f"{total_s} s" if rem_ms == 0 else f"{total_s}.{rem_ms:03d} s"
    total_m, s = divmod(total_s, 60)
    if total_m < 60:
        return f"{total_m} m {s} s"
    total_h, m = divmod(total_m, 60)
    if total_h < 24:
        return f"{total_h} h {m} m {s} s"
    d, h = divmod(total_h, 24)
    return f"{d} d {h} h {m} m"


def format_temp_k(temp_k: float, use_celsius: bool = True) -> str:
    """Format a Kelvin temperature; 0 K is treated as unset."""
    k = float(temp_k)
    if k <= 0.0:
        return "—"
    c = k - 273.15
    if use_celsius:
        return f"{c:.2f} C  ({k:.2f} K)"
    f = (c * 1.8) + 32.0
    return f"{f:.2f} F  ({k:.2f} K)"


def _decode_oem_software_revision(words: list[int]) -> Dict[str, Any]:
    """Words 13-16: GPP then DSP version, each a uint32 (low word first).

    Live Gen 1 words are 0x032f 0x0000 0x032f 0x0000, i.e. both 0.3.47; the DSP
    value matches sledInformation versionLepton. Byte layout inside each uint32
    (major = bits 16-23, minor = 8-15, build = 0-7) is inferred from that match.
    """

    def _ver(lo: int, hi: int) -> Tuple[int, int, int]:
        v = _u32_low_word_first(lo, hi)
        return (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF

    gpp = _ver(words[13], words[14])
    dsp = _ver(words[15], words[16])
    return {
        "gpp_major": gpp[0],
        "gpp_minor": gpp[1],
        "gpp_build": gpp[2],
        "dsp_major": dsp[0],
        "dsp_minor": dsp[1],
        "dsp_build": dsp[2],
        "gpp": f"{gpp[0]}.{gpp[1]}.{gpp[2]}",
        "dsp": f"{dsp[0]}.{dsp[1]}.{dsp[2]}",
        # DSP string matches sledInformation versionLepton on Gen 1 (e.g. 0.3.47).
        "summary": f"GPP {gpp[0]}.{gpp[1]}.{gpp[2]} / DSP {dsp[0]}.{dsp[1]}.{dsp[2]}",
    }


def _status_summary(
    *,
    ffc_state: str,
    ffc_desired: bool,
    agc_enabled: bool,
    shutter_lockout: bool,
    overtemp: bool,
) -> str:
    parts = [
        f"FFC {ffc_state.replace('_', ' ')}"
        + (" (desired)" if ffc_desired else ""),
        f"AGC {'on' if agc_enabled else 'off'}",
        "shutter lockout" if shutter_lockout else "shutter OK",
    ]
    if overtemp:
        parts.append("overtemp shutdown imminent")
    return "; ".join(parts)


def decode_telemetry_row_a(payload: bytes | str) -> Dict[str, Any]:
    """
    Decode VoSPI Packet 60 (telemetry Row A) per Lepton datasheet Table 2.

    Word indices are 0-based over the 160-byte payload (after packet ID/CRC).
    32-bit counters use low-word-first packing (live-verified on Gen 1).
    """
    words = _telemetry_words_from_payload(payload)
    if words is None:
        return {}

    status = _u32_low_word_first(words[3], words[4])
    ffc_code = (status >> 4) & 0x3
    ffc_state = VOSPI_FFC_STATE_NAMES.get(ffc_code, f"unknown({ffc_code})")
    ffc_desired = bool(status & (1 << 3))
    agc_enabled = bool(status & (1 << 12))
    shutter_lockout = bool(status & (1 << 15))
    overtemp = bool(status & (1 << 20))
    fpa_ck, fpa_k = _kelvin_x100(words[24])
    housing_ck, housing_k = _kelvin_x100(words[26])
    fpa_ffc_ck, fpa_ffc_k = _kelvin_x100(words[29])
    housing_ffc_ck, housing_ffc_k = _kelvin_x100(words[32])
    rev = int(words[0]) & 0xFFFF
    serial_words = words[5:13]
    serial_hex = _words_hex(words, 5, 8)
    serial_reported = any(w != 0 for w in serial_words)
    sw = _decode_oem_software_revision(words)
    vof = _u32_low_word_first(words[72], words[73])

    return {
        "telemetry_revision": rev,
        "telemetry_revision_major": (rev >> 8) & 0xFF,
        "telemetry_revision_minor": rev & 0xFF,
        "uptime_ms": _u32_low_word_first(words[1], words[2]),
        "status_bits": status,
        "status_summary": _status_summary(
            ffc_state=ffc_state,
            ffc_desired=ffc_desired,
            agc_enabled=agc_enabled,
            shutter_lockout=shutter_lockout,
            overtemp=overtemp,
        ),
        "ffc_desired": ffc_desired,
        "ffc_state_code": ffc_code,
        "ffc_state": ffc_state,
        "agc_enabled": agc_enabled,
        "shutter_lockout": shutter_lockout,
        "overtemp_shutdown_imminent": overtemp,
        "module_serial_hex": serial_hex,
        "module_serial_reported": serial_reported,
        "software_revision": sw["summary"],
        "software_revision_gpp": sw["gpp"],
        "software_revision_dsp": sw["dsp"],
        "frame_counter": _u32_low_word_first(words[20], words[21]),
        "frame_mean": int(words[22]),
        "fpa_temp_counts": int(words[23]),
        "fpa_temp_ck": fpa_ck,
        "fpa_temp_k": fpa_k,
        "housing_temp_counts": int(words[25]),
        "housing_temp_ck": housing_ck,
        "housing_temp_k": housing_k,
        "fpa_temp_at_last_ffc_ck": fpa_ffc_ck,
        "fpa_temp_at_last_ffc_k": fpa_ffc_k,
        "uptime_at_last_ffc_ms": _u32_low_word_first(words[30], words[31]),
        "housing_temp_at_last_ffc_ck": housing_ffc_ck,
        "housing_temp_at_last_ffc_k": housing_ffc_k,
        # Datasheet labels these (top, left, bottom, right), but live Gen 1
        # reads (0, 0, 79, 59) for an 80x60 frame, i.e. column/row order.
        "agc_roi": {
            "start_col": int(words[34]),
            "start_row": int(words[35]),
            "end_col": int(words[36]),
            "end_row": int(words[37]),
        },
        "agc_clip_limit_high": int(words[38]),
        "agc_clip_limit_low": int(words[39]),
        "video_output_format": vof,
        "video_output_format_name": VOSPI_VIDEO_OUTPUT_FORMAT_NAMES.get(
            vof, f"unknown({vof})"
        ),
        "log2_ffc_frames": int(words[74]),
    }


# Row B on Gen 1 does NOT follow the Lepton 2.5 radiometry layout: words 19-26
# read 0 in every frame of a live capture (2026-11-25). Identified so far:
#   words 3-10   RBFO radiometry parameters: four low-word-first 32-bit values
#                R, B x1000, F x1000, O x1000 (O signed); constant per unit;
#   words 11-18  an identical second copy;
#   word 2       shutter temperature at the last FFC, kelvin x100; 30000
#                (300.00 K) placeholder until the first FFC;
#   word 1       at each FFC exactly floor(RBFO(word 2 / 100)), i.e. the raw
#                counts the closed shutter should read; drifts with word 0
#                between FFCs.
# Words 0, 39 and 42-44 track FPA/housing temperatures but are not identified;
# they are passed through raw.
VOSPI_ROW_B_SHUTTER_PLACEHOLDER_CK = 30000


def _s32_low_word_first(lo: int, hi: int) -> int:
    v = _u32_low_word_first(lo, hi)
    return v - (1 << 32) if v & 0x8000_0000 else v


def _decode_rbfo(words: list[int], start: int) -> Optional[Dict[str, Any]]:
    """RBFO block at ``words[start:start+8]``; None when the block is all zeros."""
    block = words[start : start + 8]
    if not any(block):
        return None
    r = _u32_low_word_first(block[0], block[1])
    b_milli = _s32_low_word_first(block[2], block[3])
    f_milli = _s32_low_word_first(block[4], block[5])
    o_milli = _s32_low_word_first(block[6], block[7])
    return {
        "R": r,
        "B": b_milli / 1000.0,
        "F": f_milli / 1000.0,
        "O": o_milli / 1000.0,
    }


def rbfo_counts(temp_k: float, rbfo: Dict[str, Any]) -> float:
    """Raw counts the Lepton's RBFO curve assigns to a blackbody at ``temp_k``."""
    return rbfo["R"] / (math.exp(rbfo["B"] / temp_k) - rbfo["F"]) + rbfo["O"]


def decode_telemetry_row_b(payload: bytes | str) -> Dict[str, Any]:
    """Decode the identified parts of VoSPI Packet 61 (Row B) on Gen 1."""
    words = _telemetry_words_from_payload(payload)
    if words is None:
        return {}

    shutter_ck, shutter_k = _kelvin_x100(words[2])
    rbfo = _decode_rbfo(words, 3)
    rbfo_copy = _decode_rbfo(words, 11)
    return {
        "rbfo": rbfo,
        "rbfo_copy_matches": rbfo_copy == rbfo,
        "shutter_temp_at_last_ffc_ck": shutter_ck,
        "shutter_temp_at_last_ffc_k": shutter_k,
        "shutter_temp_at_last_ffc_is_placeholder": (
            shutter_ck == VOSPI_ROW_B_SHUTTER_PLACEHOLDER_CK
        ),
        "shutter_reference_counts": int(words[1]),
        "word_0": int(words[0]),
        "word_39": int(words[39]),
        "nonzero_words": {i: int(w) for i, w in enumerate(words) if w},
    }


def telemetry_display_rows(
    telemetry: Dict[str, Any],
    *,
    use_celsius: bool = True,
) -> list[Tuple[str, str, str]]:
    """Flatten decoded VoSPI telemetry into (group, field, value) rows for the GUI."""
    rows: list[Tuple[str, str, str]] = []
    row_a = telemetry.get("row_a") or {}
    row_b = telemetry.get("row_b") or {}

    def add(group: str, field: str, value: Any) -> None:
        rows.append((group, field, str(value)))

    def temp(k: Any) -> str:
        return format_temp_k(float(k or 0.0), use_celsius=use_celsius)

    if row_a:
        add("A", "Telemetry revision",
            f"{row_a.get('telemetry_revision_major', 0)}.{row_a.get('telemetry_revision_minor', 0)}")
        up_ms = int(row_a.get("uptime_ms", 0))
        add("A", "Uptime", f"{format_duration_ms(up_ms)}  ({up_ms} ms)")
        add("A", "Frame counter", row_a.get("frame_counter", 0))
        add("A", "Frame mean", row_a.get("frame_mean", 0))
        add("A", "Status",
            f"{row_a.get('status_summary', '')}  [0x{row_a.get('status_bits', 0):08x}]")
        if row_a.get("module_serial_reported"):
            add("A", "Module serial", row_a.get("module_serial_hex", ""))
        else:
            add("A", "Module serial", "not reported (module sent zeros)")
        add("A", "Software revision", row_a.get("software_revision", ""))
        add("A", "FPA temperature",
            f"{temp(row_a.get('fpa_temp_k'))}  ·  {row_a.get('fpa_temp_counts', 0)} counts")
        add("A", "Housing temperature",
            f"{temp(row_a.get('housing_temp_k'))}  ·  {row_a.get('housing_temp_counts', 0)} counts")
        add("A", "FPA at last FFC", temp(row_a.get("fpa_temp_at_last_ffc_k")))
        add("A", "Housing at last FFC", temp(row_a.get("housing_temp_at_last_ffc_k")))
        up_ffc = int(row_a.get("uptime_at_last_ffc_ms", 0))
        add("A", "Uptime at last FFC",
            "—" if up_ffc == 0 else f"{format_duration_ms(up_ffc)}  ({up_ffc} ms)")
        roi = row_a.get("agc_roi") or {}
        add("A", "AGC ROI",
            f"cols {roi.get('start_col', 0)}–{roi.get('end_col', 0)}, "
            f"rows {roi.get('start_row', 0)}–{roi.get('end_row', 0)}")
        add("A", "AGC clip limits",
            f"high={row_a.get('agc_clip_limit_high', 0)}  low={row_a.get('agc_clip_limit_low', 0)}")
        add("A", "Video output format",
            f"{row_a.get('video_output_format_name', '')}  "
            f"(enum {row_a.get('video_output_format', 0)})")
        add("A", "Log2 FFC frames", row_a.get("log2_ffc_frames", 0))

    if row_b:
        rbfo = row_b.get("rbfo")
        if rbfo:
            copy_note = "" if row_b.get("rbfo_copy_matches") else "  (copy in words 11-18 differs)"
            add("B", "RBFO (words 3-10)",
                f"R={rbfo['R']}  B={rbfo['B']:.3f}  F={rbfo['F']:.3f}  O={rbfo['O']:.3f}{copy_note}")
        else:
            add("B", "RBFO (words 3-10)", "not reported (all zeros)")
        if row_b.get("shutter_temp_at_last_ffc_is_placeholder"):
            shutter = "no FFC yet (300.00 K placeholder)"
        else:
            shutter = temp(row_b.get("shutter_temp_at_last_ffc_k"))
        add("B", "Shutter temp at last FFC (word 2)", shutter)
        ref = int(row_b.get("shutter_reference_counts", 0))
        ref_txt = str(ref)
        if rbfo and not row_b.get("shutter_temp_at_last_ffc_is_placeholder"):
            expected = rbfo_counts(float(row_b.get("shutter_temp_at_last_ffc_k", 0.0)), rbfo)
            ref_txt += f"  (RBFO of shutter temp = {expected:.1f})"
        add("B", "Shutter reference counts (word 1)", ref_txt)
        add("B", "Word 0 (unidentified, tracks FPA)", row_b.get("word_0", 0))
        w39 = int(row_b.get("word_39", 0))
        add("B", "Word 39 (unidentified, alternate FPA conversion)",
            f"{w39}  (as K×100: {temp(w39 / 100.0)})")
        identified = {0, 1, 2, 39, *range(3, 19)}
        other = {
            i: w for i, w in (row_b.get("nonzero_words") or {}).items()
            if i not in identified
        }
        add("B", "Other non-zero words",
            ", ".join(f"{i}={w}" for i, w in sorted(other.items())) or "none")

    return rows


@dataclass
class RosebudFrame:
    frame_index: int  # Rosebud header field 1; always 1 on Gen 1, not a counter
    sequence: int = 0  # host-side count of frames parsed by this RosebudParser
    timestamp_notified: float = 0.0
    timestamp_enqueued: float = 0.0
    
    # Thermal Data
    thermal_raw: np.ndarray = field(default_factory=lambda: np.zeros((60, 80), dtype=np.uint16))
    thermal_celsius: np.ndarray = field(default_factory=lambda: np.zeros((60, 80), dtype=np.float32))
    
    # Visual Image
    visual_jpeg: Optional[bytes] = None
    visual_image: Optional[np.ndarray] = None  # 640x480 BGR image
    
    # Metadata & Hardware Telemetry
    metadata: Dict[str, Any] = field(default_factory=dict)
    telemetry: Dict[str, Any] = field(default_factory=dict)
    
    # Status
    shutter_state: str = "unknown"
    shutter_temp_k: float = 0.0
    fpa_temp_k: float = 0.0
    housing_temp_k: float = 0.0
    ffc_state: str = "unknown"
    min_temp_c: float = 0.0
    max_temp_c: float = 0.0
    center_temp_c: float = 0.0
    
    # Optional debug dump: full assembled Rosebud packet bytes (header + payload)
    rosebud_packet_bytes: bytes = b""


class RosebudParser:
    def __init__(self, calibration: Optional[FlirCalibration] = None):
        self.calib = calibration or FlirCalibration()
        self.buffer = bytearray()
        self.sequence = 0
        
    def feed(self, chunk: bytes) -> list[RosebudFrame]:
        """
        Feeds incoming bytes from USB bulk endpoint into the parser buffer.
        Returns a list of all fully assembled, validated RosebudFrame objects.
        """
        self.buffer.extend(chunk)
        frames = []
        
        while len(self.buffer) >= HEADER_SIZE:
            # Search for magic header 0x0000BEEF (0xEF, 0xBE, 0x00, 0x00)
            magic_idx = self.buffer.find(b"\xef\xbe\x00\x00")
            if magic_idx == -1:
                # Discard partial buffer leaving last 3 bytes (in case magic is split across chunks)
                self.buffer = self.buffer[-3:]
                break
            elif magic_idx > 0:
                # Discard bytes before the magic marker
                del self.buffer[:magic_idx]
                
            if len(self.buffer) < HEADER_SIZE:
                break
                
            # Parse the 28-byte header
            header_bytes = bytes(self.buffer[:HEADER_SIZE])
            (
                magic,
                frame_index,
                total_payload_len,
                thermal_len,
                visual_len,
                metadata_len,
                header_crc
            ) = struct.unpack("<IIIIIII", header_bytes)
            
            # Verify Header CRC32 (calculated over the first 24 bytes)
            calculated_crc = zlib.crc32(header_bytes[:24]) & 0xFFFFFFFF
            if calculated_crc != header_crc:
                # Header CRC mismatch, advance 1 byte and resynchronize
                del self.buffer[0:1]
                continue
                
            # Check if entire payload has arrived
            total_frame_size = HEADER_SIZE + total_payload_len
            if len(self.buffer) < total_frame_size:
                # Need more data from USB
                break
                
            # Extract complete payload and also keep the full packet bytes
            raw_packet = bytes(self.buffer[:total_frame_size])
            payload = bytes(self.buffer[HEADER_SIZE:total_frame_size])
            del self.buffer[:total_frame_size]
            
            # Parse payload components
            frame = self._parse_payload(
                frame_index, payload, thermal_len, visual_len, metadata_len
            )
            if frame:
                frame.rosebud_packet_bytes = raw_packet
                self.sequence += 1
                frame.sequence = self.sequence
                frames.append(frame)
                
        return frames

    def _parse_payload(
        self,
        frame_index: int,
        payload: bytes,
        thermal_len: int,
        visual_len: int,
        metadata_len: int
    ) -> Optional[RosebudFrame]:
        """Extracts and decodes thermal matrix, visual image, and metadata from payload."""
        offset = 0
        
        # 1. Extract Thermal Segment (10,332 bytes)
        thermal_raw_bytes = payload[offset : offset + thermal_len]
        offset += thermal_len
        
        # 2. Extract Visual Segment (JPEG)
        visual_bytes = payload[offset : offset + visual_len]
        offset += visual_len
        
        # 3. Extract Metadata String (JSON)
        metadata_raw = payload[offset : offset + metadata_len]
        
        # Unpack VoSPI thermal packets
        thermal_matrix, telemetry = self._unpack_vospi_thermal(thermal_raw_bytes)
        
        # Radiometric conversion to Celsius using calibration
        thermal_celsius = self.calib.raw_to_celsius(thermal_matrix)
        
        # Decode Visual JPEG
        visual_img = None
        if visual_bytes:
            np_arr = np.frombuffer(visual_bytes, dtype=np.uint8)
            visual_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
        # Parse JSON Metadata
        metadata = {}
        if metadata_raw:
            try:
                # Remove trailing null terminator if present
                clean_json_str = metadata_raw.split(b"\x00")[0].decode("utf-8", errors="ignore")
                if clean_json_str:
                    metadata = json.loads(clean_json_str)
            except Exception:
                pass
                
        # Prefer VoSPI Row A temps when present; JSON metadata often omits them on Gen 1.
        row_a = telemetry.get("row_a") or {}
        fpa_temp_k = float(row_a["fpa_temp_k"]) if "fpa_temp_k" in row_a else float(
            metadata.get("fpaTemperature", 0.0)
        )
        housing_temp_k = float(row_a["housing_temp_k"]) if "housing_temp_k" in row_a else float(
            metadata.get("housingTemperature", 0.0)
        )

        # Build frame object
        frame = RosebudFrame(
            frame_index=frame_index,
            thermal_raw=thermal_matrix,
            thermal_celsius=thermal_celsius,
            visual_jpeg=visual_bytes if visual_bytes else None,
            visual_image=visual_img,
            metadata=metadata,
            telemetry=telemetry,
            shutter_state=metadata.get("shutterState", "unknown"),
            shutter_temp_k=float(metadata.get("shutterTemperature", 0.0)),
            fpa_temp_k=fpa_temp_k,
            housing_temp_k=housing_temp_k,
            # Keep the ffc daemon JSON state (FFC_RAD_VALID, …) for shutter calibration.
            ffc_state=metadata.get("ffcState", "unknown"),
            timestamp_notified=float(metadata.get("usbNotifiedTimestamp", 0.0)),
            timestamp_enqueued=float(metadata.get("usbEnqueuedTimestamp", 0.0)),
            min_temp_c=float(np.min(thermal_celsius)),
            max_temp_c=float(np.max(thermal_celsius)),
            center_temp_c=float(thermal_celsius[30, 40])
        )
        return frame

    def _unpack_vospi_thermal(self, raw_thermal_bytes: bytes) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Unpacks 63 VoSPI packets (164 bytes each):
        - Packets 0..59: 60 image rows (80 pixels, uint16 big-endian).
        - Packets 60..62: Telemetry rows.
        """
        image_matrix = np.zeros((60, 80), dtype=np.uint16)
        telemetry = {}
        
        if len(raw_thermal_bytes) < EXPECTED_THERMAL_SIZE:
            return image_matrix, telemetry
            
        for packet_idx in range(VOSPI_PACKET_COUNT):
            p_offset = packet_idx * VOSPI_PACKET_SIZE
            packet = raw_thermal_bytes[p_offset : p_offset + VOSPI_PACKET_SIZE]
            
            # Packet header (2 bytes ID + 2 bytes CRC)
            # VoSPI ID: xFxx marks a discard packet; otherwise the low 12 bits are
            # the packet (line) number: 0..59 image rows, 60..62 telemetry.
            if (packet[0] & 0x0F) == 0x0F:
                continue
            packet_num = ((packet[0] & 0x0F) << 8) | packet[1]
            payload_data = packet[4:164]  # 160 bytes
            
            # Some devices send VoSPI rows in a non-sequential order. Use the embedded packet_num
            # to place each thermal row into the correct location.
            if packet_num < 60 and len(payload_data) == 160:
                # 80 uint16 big-endian pixels
                row_pixels = np.frombuffer(payload_data, dtype=">u2")
                image_matrix[packet_num, :] = row_pixels
            elif packet_num < VOSPI_PACKET_COUNT:
                # Telemetry line (packet_num 60..62): keep raw hex for all three and
                # decode Rows A and B. Row C is all zeros on Gen 1, so it stays raw.
                row_key = f"telemetry_row_{packet_num - 60}"
                telemetry[row_key] = payload_data.hex()
                if packet_num == 60:
                    decoded = decode_telemetry_row_a(payload_data)
                    if decoded:
                        telemetry["row_a"] = decoded
                elif packet_num == 61:
                    decoded = decode_telemetry_row_b(payload_data)
                    if decoded:
                        telemetry["row_b"] = decoded

        return image_matrix, telemetry
