"""
FLIR One Rosebud (0xBEEF) Protocol Parser & Frame Demuxer
Unpacks the 28-byte header, 10,332-byte VoSPI thermal data, visual JPEG, and JSON metadata.
"""

import json
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


@dataclass
class RosebudFrame:
    frame_index: int
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
            fpa_temp_k=float(metadata.get("fpaTemperature", 0.0)),
            housing_temp_k=float(metadata.get("housingTemperature", 0.0)),
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
                # Telemetry line (packet_num 60..62)
                telemetry[f"telemetry_row_{packet_num - 60}"] = payload_data.hex()
                
        return image_matrix, telemetry
