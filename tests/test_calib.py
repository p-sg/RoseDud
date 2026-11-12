"""calib.rsc parsing and radiometric conversion."""

import os

import numpy as np

from flir_calib import FlirCalibration

CALIB_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "calib.rsc")


def test_calib_rsc_planck_and_fusion_constants():
    c = FlirCalibration(CALIB_PATH)
    assert (c.R, c.B, c.F, c.J0, c.J1) == (16728.0, 1431.1, 1.0, 3027.0, 77.408089)
    assert c.fusion_magnification == 1.2822002
    assert c.fusion_c1y == -24.672901


def test_celsius_raw_round_trip():
    c = FlirCalibration()
    temps = np.array([-20.0, 0.0, 22.0, 37.0, 100.0])
    back = c.raw_to_celsius(c.celsius_to_raw(temps))
    assert np.allclose(back, temps, atol=0.1)


def test_ffc_recovers_offset_from_shutter():
    truth = FlirCalibration()
    truth.dynamic_offset = 3500.0
    shutter_k = 300.0
    raw = truth.celsius_to_raw(np.full((60, 80), shutter_k - 273.15))

    c = FlirCalibration()  # starts at the default offset of 3200
    c.perform_ffc(raw, shutter_k)
    assert abs(c.dynamic_offset - 3500.0) < 1.0
