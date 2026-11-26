"""VoSPI Row A/B/C decode, display rows, and charging-state labels."""

import struct

import pytest

from flir_calib import FlirCalibration
from flir_protocol import (
    EXPECTED_THERMAL_SIZE,
    VOSPI_PACKET_SIZE,
    RosebudParser,
    decode_telemetry_row_a,
    decode_telemetry_row_b,
    rbfo_counts,
    format_duration_ms,
    format_temp_k,
    telemetry_display_rows,
)
from iap2 import format_charging_state
from test_protocol import make_packet, make_thermal


def _pack_words(overrides: dict[int, int]) -> bytes:
    words = [0] * 80
    for idx, value in overrides.items():
        words[int(idx)] = int(value) & 0xFFFF
    return struct.pack(">80H", *words)


def _pack_row_a(
    *,
    uptime_ms: int = 0,
    status: int = 0,
    status_hi: int = 0,
    frame_counter: int = 0,
    fpa_ck: int = 0,
    housing_ck: int = 0,
    fpa_counts: int = 0,
    housing_counts: int = 0,
    word0: int = 9,
) -> bytes:
    return _pack_words({
        0: word0,
        1: uptime_ms & 0xFFFF,
        2: (uptime_ms >> 16) & 0xFFFF,
        3: status,
        4: status_hi,
        20: frame_counter & 0xFFFF,
        21: (frame_counter >> 16) & 0xFFFF,
        23: fpa_counts,
        24: fpa_ck,
        25: housing_counts,
        26: housing_ck,
    })


def _thermal_with_rows(row_a: bytes, row_b: bytes | None = None, row_c: bytes | None = None) -> bytes:
    row_b = row_b if row_b is not None else bytes([61]) * 160
    row_c = row_c if row_c is not None else bytes([62]) * 160
    packets = []
    for row in range(60):
        packets.append(struct.pack(">HH", row, 0) + struct.pack(">80H", *([1000 + row] * 80)))
    packets.append(struct.pack(">HH", 60, 0) + row_a)
    packets.append(struct.pack(">HH", 61, 0) + row_b)
    packets.append(struct.pack(">HH", 62, 0) + row_c)
    data = b"".join(packets)
    assert len(data) == EXPECTED_THERMAL_SIZE
    return data


def test_decode_uptime_and_frame_counter_low_word_first():
    row = _pack_row_a(uptime_ms=0x0001_0BFF, frame_counter=0x0000_0053)
    decoded = decode_telemetry_row_a(row)
    assert decoded["uptime_ms"] == 0x00010BFF
    assert decoded["frame_counter"] == 0x53

    words = list(struct.unpack(">80H", row))
    be_uptime = (words[1] << 16) | words[2]
    assert be_uptime != decoded["uptime_ms"]
    assert be_uptime == 0x0BFF0001


def test_decode_temps_counts_and_ffc_status():
    # status 0x0808: bit3 desired, bits4:5 = 00 never_commanded (live dump)
    row = _pack_row_a(
        status=0x0808,
        fpa_ck=29121,
        housing_ck=28714,
        fpa_counts=8434,
        housing_counts=10080,
    )
    decoded = decode_telemetry_row_a(row)
    assert decoded["ffc_desired"] is True
    assert decoded["ffc_state_code"] == 0
    assert decoded["ffc_state"] == "never_commanded"
    assert "FFC never commanded (desired)" in decoded["status_summary"]
    assert "AGC off" in decoded["status_summary"]
    assert decoded["fpa_temp_counts"] == 8434
    assert decoded["housing_temp_counts"] == 10080
    assert decoded["fpa_temp_ck"] == 29121
    assert abs(decoded["fpa_temp_k"] - 291.21) < 1e-9
    assert decoded["housing_temp_ck"] == 28714
    assert abs(decoded["housing_temp_k"] - 287.14) < 1e-9

    assert decode_telemetry_row_a(_pack_row_a(status=0x0010))["ffc_state"] == "imminent"
    assert decode_telemetry_row_a(_pack_row_a(status=0x0020))["ffc_state"] == "in_progress"
    assert decode_telemetry_row_a(_pack_row_a(status=0x0030))["ffc_state"] == "complete"


def test_decode_software_revision_and_video_format():
    # Live dump: w13=0x032f, w14=0, w15=0x032f, w16=0, w72=7 -> RAW14.
    # Two low-word-first uint32 values 0x0000032f -> both 0.3.47 (DSP matches
    # sledInformation versionLepton).
    row = _pack_words({13: 0x032F, 14: 0, 15: 0x032F, 16: 0, 72: 7, 73: 0})
    decoded = decode_telemetry_row_a(row)
    assert decoded["software_revision_gpp"] == "0.3.47"
    assert decoded["software_revision_dsp"] == "0.3.47"
    assert decoded["software_revision"] == "GPP 0.3.47 / DSP 0.3.47"
    assert decoded["module_serial_reported"] is False
    assert decoded["video_output_format"] == 7
    assert decoded["video_output_format_name"] == "RAW14"


def test_format_duration_and_temp_units():
    assert format_duration_ms(4500) == "4.500 s"
    assert format_duration_ms(65_000) == "1 m 5 s"
    assert format_duration_ms(3_726_000) == "1 h 2 m 6 s"
    assert format_temp_k(300.0, use_celsius=True).startswith("26.85 C")
    assert "K)" in format_temp_k(300.0, use_celsius=True)
    assert format_temp_k(300.0, use_celsius=False).startswith("80.33 F")
    assert format_temp_k(0.0) == "—"


def test_decode_status_high_word_overtemp_bit():
    # Bit 20 lives in word 4 (status bits 16-31).
    row = _pack_row_a(status=0, status_hi=0x0010)
    decoded = decode_telemetry_row_a(row)
    assert decoded["overtemp_shutdown_imminent"] is True
    assert decoded["status_bits"] == 0x00100000


# Synthetic Row B words 0-18 in the live Gen 1 layout (values made up, not a real unit's).
SYNTH_ROW_B_HEAD = {
    0: 15275, 1: 3735, 2: 30274,
    3: 6784, 4: 6, 5: 53744, 6: 21, 7: 1000, 8: 0, 9: 18928, 10: 2,
    11: 6784, 12: 6, 13: 53744, 14: 21, 15: 1000, 16: 0, 17: 18928, 18: 2,
}


def test_decode_row_b_rbfo_and_shutter_reference():
    decoded = decode_telemetry_row_b(_pack_words({**SYNTH_ROW_B_HEAD, 39: 30653}))
    assert decoded["rbfo"] == {"R": 400000, "B": 1430.0, "F": 1.0, "O": 150.0}
    assert decoded["rbfo_copy_matches"] is True
    assert decoded["shutter_temp_at_last_ffc_is_placeholder"] is False
    assert abs(decoded["shutter_temp_at_last_ffc_k"] - 302.74) < 1e-9
    assert decoded["shutter_reference_counts"] == 3735
    # On live data word 1 equals floor(RBFO(shutter temp)) at each FFC.
    expected = rbfo_counts(decoded["shutter_temp_at_last_ffc_k"], decoded["rbfo"])
    assert int(expected) == 3735
    assert decoded["word_0"] == 15275 and decoded["word_39"] == 30653


def test_decode_row_b_placeholder_and_missing_rbfo():
    decoded = decode_telemetry_row_b(_pack_words({0: 14294, 2: 30000, 39: 30653}))
    assert decoded["shutter_temp_at_last_ffc_is_placeholder"] is True
    assert decoded["rbfo"] is None
    assert decoded["nonzero_words"] == {0: 14294, 2: 30000, 39: 30653}


def test_decode_row_b_rbfo_offset_is_signed():
    words = dict(SYNTH_ROW_B_HEAD)
    words[9], words[10] = 0xFC18, 0xFFFF  # -1000 -> O = -1.0
    assert decode_telemetry_row_b(_pack_words(words))["rbfo"]["O"] == -1.0
    assert decode_telemetry_row_b(_pack_words(words))["rbfo_copy_matches"] is False


def test_agc_roi_uses_column_row_order():
    decoded = decode_telemetry_row_a(_pack_words({36: 79, 37: 59}))
    assert decoded["agc_roi"] == {"start_col": 0, "start_row": 0, "end_col": 79, "end_row": 59}


def test_parser_sequence_counts_frames_when_header_index_is_constant():
    # Gen 1 sends 1 in the Rosebud header index field for every frame.
    parser = RosebudParser(FlirCalibration())
    frames = parser.feed(make_packet(frame_index=1) + make_packet(frame_index=1))
    frames += parser.feed(make_packet(frame_index=1))
    assert [f.frame_index for f in frames] == [1, 1, 1]
    assert [f.sequence for f in frames] == [1, 2, 3]


def test_decode_accepts_hex_and_rejects_short():
    row = _pack_row_a(uptime_ms=42, fpa_ck=30000)
    assert decode_telemetry_row_a(row.hex())["uptime_ms"] == 42
    assert decode_telemetry_row_a(b"\x00" * 10) == {}
    assert decode_telemetry_row_a("not-hex") == {}


def test_parser_attaches_all_rows_and_fills_frame_temps():
    row_a = _pack_row_a(
        uptime_ms=5000,
        status=0x0808,
        frame_counter=99,
        fpa_ck=30000,
        housing_ck=29500,
        fpa_counts=9000,
        housing_counts=9100,
    )
    row_b = _pack_words({2: 30274})
    row_c = bytes(160)
    meta = b'{"shutterState":"open","ffcState":"FFC_RAD_VALID"}\x00'
    frame = RosebudParser(FlirCalibration()).feed(
        make_packet(thermal=_thermal_with_rows(row_a, row_b, row_c), meta=meta)
    )[0]
    assert "row_a" in frame.telemetry and "row_b" in frame.telemetry
    assert "row_c" not in frame.telemetry and "telemetry_row_2" in frame.telemetry
    assert frame.sequence == 1
    assert frame.telemetry["row_a"]["uptime_ms"] == 5000
    assert frame.telemetry["row_a"]["frame_counter"] == 99
    assert frame.telemetry["row_b"]["shutter_temp_at_last_ffc_ck"] == 30274
    assert abs(frame.fpa_temp_k - 300.0) < 1e-9
    assert abs(frame.housing_temp_k - 295.0) < 1e-9
    assert frame.ffc_state == "FFC_RAD_VALID"
    assert frame.telemetry["row_a"]["ffc_state"] == "never_commanded"


def test_telemetry_display_rows_a_and_b():
    tel = {
        "row_a": decode_telemetry_row_a(
            _pack_row_a(uptime_ms=1000, fpa_ck=30000, housing_ck=29000)
        ),
        "row_b": decode_telemetry_row_b(_pack_words({**SYNTH_ROW_B_HEAD, 39: 30653, 42: 8003})),
    }
    rows_c = telemetry_display_rows(tel, use_celsius=True)
    rows_f = telemetry_display_rows(tel, use_celsius=False)
    assert {g for g, _f, _v in rows_c} == {"A", "B"}
    fields = {f for _g, f, _v in rows_c}
    assert "Uptime" in fields
    assert not any(f.startswith("w[") for f in fields)
    uptime_val = next(v for g, f, v in rows_c if f == "Uptime")
    assert "1 s" in uptime_val and "1000 ms" in uptime_val
    fpa_c = next(v for g, f, v in rows_c if f == "FPA temperature")
    fpa_f = next(v for g, f, v in rows_f if f == "FPA temperature")
    assert "C" in fpa_c and "F" not in fpa_c.split("(")[0]
    assert "F" in fpa_f.split("(")[0]
    shutter = next(v for g, f, v in rows_c if f.startswith("Shutter temp at last FFC"))
    assert shutter.startswith("29.59 C")
    rbfo = next(v for g, f, v in rows_c if f.startswith("RBFO"))
    assert rbfo == "R=400000  B=1430.000  F=1.000  O=150.000"
    ref = next(v for g, f, v in rows_c if f.startswith("Shutter reference counts"))
    assert ref.startswith("3735  (RBFO of shutter temp = 3735.4")
    assert next(v for g, f, v in rows_c if f == "Other non-zero words") == "42=8003"
    assert next(v for g, f, v in rows_c if f == "AGC ROI") == "cols 0–0, rows 0–0"


@pytest.mark.parametrize(
    "raw,label",
    [
        ("stateDeveloper", "Developer"),
        ("stateNoCharging", "Not Charging"),
        ("stateManagedBatteryCharging", "Managed Charging"),
        ("stateChargingSmartPhone", "Charging Phone"),
        ("stateChargingSmartphone", "Charging Phone"),
        ("STATECHARGINGSMARTPHONE", "Charging Phone"),
        ("stateHandleFault", "Handle Fault"),
        ("stateHandleFaultHeat", "Handle Fault (Heat)"),
        ("stateHandleFaultBadCharger", "Handle Fault (Bad Charger)"),
        ("stateChargingSmartphoneFaultHeat", "Phone Charge Fault (Heat)"),
        ("stateManagedBatteryChargingOnly", "Managed Charging Only"),
        ("stateChargingSmartPhoneOnly", "Charging Phone Only"),
        ("invalid", "Invalid"),
        ("stateSomethingNew", "stateSomethingNew"),
        ("", "Unknown"),
        (None, "Unknown"),
    ],
)
def test_format_charging_state(raw, label):
    assert format_charging_state(raw) == label


def test_make_thermal_still_has_three_hex_rows():
    thermal = make_thermal()
    assert len(thermal) == 63 * VOSPI_PACKET_SIZE
