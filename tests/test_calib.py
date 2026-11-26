"""calib.rsc parsing and radiometric conversion (RBFO path and shutter-offset fallback)."""

import os

import numpy as np
import pytest

from flir_calib import MODE_SHUTTER_OFFSET, FlirCalibration
from flir_protocol import rbfo_counts

CALIB_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "calib.rsc")

# Synthetic RBFO in the Gen 1 Row B format (made up, not a real unit's).
SYNTH_RBFO = {"R": 400000.0, "B": 1430.0, "F": 1.0, "O": 150.0}


def _rbfo_calib() -> FlirCalibration:
    c = FlirCalibration()
    assert c.set_rbfo(SYNTH_RBFO)
    return c


@pytest.mark.skipif(
    not os.path.isfile(CALIB_PATH),
    reason="calib.rsc not present (gitignored; copy from device for this check)",
)
def test_calib_rsc_planck_and_fusion_constants():
    c = FlirCalibration(CALIB_PATH)
    # The explicit ExFOL2 doubles win over the rounded "const" text string.
    assert (c.rsc_R, c.rsc_B, c.rsc_F, c.J0, c.J1) == (16727.516, 1431.1, 1.0, 3027.0, 77.408089)
    assert c.fusion_magnification == 1.2822002
    assert c.fusion_c1y == -24.672901
    # RBFO only ever comes from telemetry.
    assert c.rbfo is None and not c.using_rbfo


@pytest.mark.parametrize("use_rbfo", [False, True], ids=["shutter_offset", "rbfo"])
def test_celsius_raw_round_trip(use_rbfo):
    c = _rbfo_calib() if use_rbfo else FlirCalibration()
    assert c.using_rbfo is use_rbfo
    temps = np.array([-20.0, 0.0, 22.0, 37.0, 100.0])
    back = c.raw_to_celsius(c.celsius_to_raw(temps))
    assert np.allclose(back, temps, atol=0.1)


def test_round_trip_with_emissivity():
    c = _rbfo_calib()
    c.emissivity = 0.96
    c.reflected_temp_k = 23.3 + 273.15
    c.atmospheric_temp_k = c.reflected_temp_k
    temps = np.array([0.0, 23.3, 40.0])
    back = c.raw_to_celsius(c.celsius_to_raw(temps))
    assert np.allclose(back, temps, atol=0.1)


def test_set_rbfo_switches_curve_and_rejects_bad_values():
    c = FlirCalibration()
    assert not c.using_rbfo
    for bad in (None, {}, {"R": 1.0}, {"R": "x", "B": 1, "F": 1, "O": 1},
                {"R": 0.0, "B": 1430.0, "F": 1.0, "O": 150.0}):
        assert c.set_rbfo(bad) is False
    assert c.rbfo is None
    assert c.set_rbfo(SYNTH_RBFO) is True
    assert c.using_rbfo and c.rbfo == SYNTH_RBFO
    assert c.set_rbfo(None) is False and c.rbfo == SYNTH_RBFO  # keeps last good values


def test_shutter_offset_mode_ignores_rbfo():
    c = FlirCalibration(mode=MODE_SHUTTER_OFFSET)
    c.set_rbfo(SYNTH_RBFO)
    assert not c.using_rbfo
    with pytest.raises(ValueError):
        FlirCalibration(mode="bogus")


def test_rbfo_closed_shutter_inverts_to_shutter_temp():
    """Closed-shutter raw = RBFO(T) inverts back to T (eps = 1)."""
    c = _rbfo_calib()
    t_k = 302.74
    raw = rbfo_counts(t_k, SYNTH_RBFO)
    pred = float(c.raw_to_celsius(np.array([[raw]]))[0, 0])
    assert abs(pred - (t_k - 273.15)) < 0.01


def test_emissivity_correction_moves_cold_target():
    c = _rbfo_calib()
    c.reflected_temp_k = 23.3 + 273.15
    c.atmospheric_temp_k = c.reflected_temp_k
    raw = np.array([[rbfo_counts(273.15, SYNTH_RBFO)]])
    t_bb = float(c.raw_to_celsius(raw)[0, 0])
    c.emissivity = 0.96
    t_eps = float(c.raw_to_celsius(raw)[0, 0])
    # With eps < 1 and a warmer reflected background, a cold reading drops.
    assert t_eps < t_bb


def test_ffc_recovers_offset_from_shutter():
    truth = FlirCalibration()
    truth.dynamic_offset = 3500.0
    shutter_k = 300.0
    raw = truth.celsius_to_raw(np.full((60, 80), shutter_k - 273.15))

    c = FlirCalibration()  # starts at the default offset of 3200
    assert not c.shutter_offset_calibrated
    c.perform_ffc(raw, shutter_k)
    assert abs(c.dynamic_offset - 3500.0) < 1.0
    assert c.shutter_offset_calibrated


def test_perform_ffc_does_not_touch_rbfo_path():
    c = _rbfo_calib()
    raw = np.full((60, 80), 3700, dtype=np.uint16)
    before = c.raw_to_celsius(raw)
    c.perform_ffc(np.full((60, 80), 3000, dtype=np.uint16), 300.0)
    assert np.array_equal(c.raw_to_celsius(raw), before)
