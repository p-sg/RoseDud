"""
FLIR One calibration and radiometric temperature conversion.

Two conversion paths, sharing one formula:

* RBFO (default once Row B telemetry reports it). The Lepton's raw14 output is
  already on the RBFO curve the sled writes to it (VoSPI Row B words 3-10):
      S = raw - O
      S_obj = (S - (1-eps)*tau*S(T_refl) - (1-tau)*S(T_atm)) / (eps*tau)
      T_K = B / ln(R / S_obj + F),   S(T) = R / (e^(B/T) - F)
  This only holds after the Lepton's first FFC since power-up.
* Shutter offset (fallback). Used until RBFO has been seen, or always when
  ``mode="shutter_offset"``: the calib.rsc Planck constants, an empirical gain,
  and an offset re-estimated from the closed shutter by ``perform_ffc``
  (S = (raw - offset) / gain; the rest is identical).

The RBFO values are per unit and come only from telemetry; none are hardcoded.
calib.rsc also supplies the MSX fusion and lens-correction metadata.
"""

from __future__ import annotations

import math
import os
import re
from typing import Any, Dict, Mapping, Optional, Tuple

import numpy as np

MODE_AUTO = "auto"
MODE_SHUTTER_OFFSET = "shutter_offset"


class FlirCalibration:
    def __init__(
        self,
        calib_rsc_path: Optional[str] = None,
        lcf_path: Optional[str] = None,
        mode: str = MODE_AUTO,
    ):
        if mode not in (MODE_AUTO, MODE_SHUTTER_OFFSET):
            raise ValueError(f"unknown radiometry mode {mode!r}")
        self.mode = mode
        self.raw_dict: Dict[str, Any] = {}

        # Lepton RBFO from Row B; None until telemetry reports it.
        self.rbfo: Optional[Dict[str, float]] = None

        # calib.rsc Planck constants, used only by the shutter-offset fallback.
        self.rsc_R: float = 16728.0
        self.rsc_B: float = 1431.1
        self.rsc_F: float = 1.0
        self.J0: float = 3027.0
        self.J1: float = 77.408089

        # Shutter-offset fallback: empirical gain, offset re-estimated by perform_ffc.
        self.linear_gain: float = 25.0
        self.dynamic_offset: float = 3200.0
        self._ffc_history: list[float] = []

        # Environmental compensation (eps = 1, tau = 1 -> plain inverse).
        self.emissivity: float = 1.0
        self.reflected_temp_k: float = 293.15
        self.atmospheric_temp_k: float = 293.15
        self.tau_atmosphere: float = 1.0

        # Visual/Thermal Fusion Alignment Parameters (from calib.rsc)
        self.fusion_c0x: float = -5.0744071
        self.fusion_c0y: float = 0.0
        self.fusion_c1x: float = -17.791127
        self.fusion_c1y: float = -24.672901
        self.fusion_magnification: float = 1.2822002
        self.fusion_rotation_deg: float = -0.33073297
        self.fusion_fix_distance: float = 3.0  # meters

        # Lens correction metadata from calib.rsc. FLIR's software does not use rotation/zoom
        # (likely the map's generation parameters); do not feed lens_zoom into cv2 rotation.
        self.lens_rotation: float = 0.1657029
        self.lens_zoom: float = 5.7087041
        self.lcf_file_name: Optional[str] = None

        # Optional geometric lens correction (experimental, see flir_lcf.py).
        self._lcf = None
        self.lcf_enabled: bool = True

        if calib_rsc_path and os.path.exists(calib_rsc_path):
            self.load_from_file(calib_rsc_path)

        if lcf_path and os.path.exists(lcf_path):
            self.load_lcf(lcf_path)

    # ------------------------------------------------------------------ RBFO --

    def set_rbfo(self, rbfo: Optional[Mapping[str, Any]]) -> bool:
        """Adopt R, B, F, O from a Row B decode. Returns True if valid values were taken."""
        if not rbfo:
            return False
        try:
            vals = {k: float(rbfo[k]) for k in ("R", "B", "F", "O")}
        except (KeyError, TypeError, ValueError):
            return False
        if vals["R"] <= 0 or vals["B"] <= 0:
            return False
        self.rbfo = vals
        return True

    @property
    def using_rbfo(self) -> bool:
        return self.mode == MODE_AUTO and self.rbfo is not None

    @property
    def shutter_offset_calibrated(self) -> bool:
        """True once perform_ffc has accepted at least one closed-shutter estimate."""
        return bool(self._ffc_history)

    def _curve(self) -> Tuple[float, float, float, float, float]:
        """Active (R, B, F, gain, offset) with S = (raw - offset) / gain."""
        if self.using_rbfo:
            r = self.rbfo
            return r["R"], r["B"], r["F"], 1.0, r["O"]
        return self.rsc_R, self.rsc_B, self.rsc_F, self.linear_gain, self.dynamic_offset

    # --------------------------------------------------------------- loading --

    def load_from_file(self, path: str) -> None:
        """Parses FLIR .rsc format key-value configuration."""
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split(maxsplit=2)
                if len(parts) >= 2:
                    key = parts[0]
                    dtype = parts[1]
                    val_str = parts[2] if len(parts) > 2 else ""
                    try:
                        if dtype == "double":
                            self.raw_dict[key] = float(val_str)
                        elif dtype == "int32":
                            self.raw_dict[key] = int(val_str)
                        elif dtype == "bool":
                            self.raw_dict[key] = (val_str.lower() == "true")
                        elif dtype == "text":
                            self.raw_dict[key] = val_str.strip('"')
                    except ValueError:
                        pass
        self._extract_parameters()

    def _extract_parameters(self) -> None:
        """Extracts specialized calibration parameters from the parsed dictionary."""
        for k, v in self.raw_dict.items():
            if "const" in k and isinstance(v, str) and "R " in v:
                self._parse_planck_const_string(v)

        # Prefer the explicit (unrounded) ExFOL2 doubles when present.
        prefix_rbf = ".calib.extRBF.ds120C_weRB_apNOA_fiNOF_leExFOL2"
        for attr, key in (
            ("rsc_R", f"{prefix_rbf}.R"),
            ("rsc_B", f"{prefix_rbf}.B"),
            ("rsc_F", f"{prefix_rbf}.F"),
            ("J0", f"{prefix_rbf}.J0"),
            ("J1", f"{prefix_rbf}.J1"),
        ):
            if key in self.raw_dict:
                setattr(self, attr, float(self.raw_dict[key]))

        # Visual fusion parameters
        prefix = ".calib.visual.fusion.leExFOL2"
        if f"{prefix}.C0X" in self.raw_dict:
            self.fusion_c0x = float(self.raw_dict[f"{prefix}.C0X"])
        if f"{prefix}.C0Y" in self.raw_dict:
            self.fusion_c0y = float(self.raw_dict[f"{prefix}.C0Y"])
        if f"{prefix}.C1X" in self.raw_dict:
            self.fusion_c1x = float(self.raw_dict[f"{prefix}.C1X"])
        if f"{prefix}.C1Y" in self.raw_dict:
            self.fusion_c1y = float(self.raw_dict[f"{prefix}.C1Y"])
        if f"{prefix}.magnQuote" in self.raw_dict:
            self.fusion_magnification = float(self.raw_dict[f"{prefix}.magnQuote"])
        if f"{prefix}.rotAngle" in self.raw_dict:
            self.fusion_rotation_deg = float(self.raw_dict[f"{prefix}.rotAngle"])
        if f"{prefix}.fixFusionDistance" in self.raw_dict:
            self.fusion_fix_distance = float(self.raw_dict[f"{prefix}.fixFusionDistance"])

        lens = ".calib.lensCorr.ds_we_ap_fi_leExFOL2"
        if isinstance(self.raw_dict.get(f"{lens}.fileName"), str):
            self.lcf_file_name = self.raw_dict[f"{lens}.fileName"]
        if f"{lens}.rotation" in self.raw_dict:
            self.lens_rotation = float(self.raw_dict[f"{lens}.rotation"])
        if f"{lens}.zoom" in self.raw_dict:
            self.lens_zoom = float(self.raw_dict[f"{lens}.zoom"])

    def _parse_planck_const_string(self, s: str) -> None:
        """Parses R, B, F, J0, J1 from a calib.rsc const text string."""
        m_r = re.search(r"R\s+([0-9.]+)", s)
        m_b = re.search(r"B\s+([0-9.]+)", s)
        m_f = re.search(r"F\s+([0-9.]+)", s)
        m_j0 = re.search(r"J0\s+([0-9.]+)", s)
        m_j1 = re.search(r"J1\s+([0-9.]+)", s)

        if m_r:
            self.rsc_R = float(m_r.group(1))
        if m_b:
            self.rsc_B = float(m_b.group(1))
        if m_f:
            self.rsc_F = float(m_f.group(1))
        if m_j0:
            self.J0 = float(m_j0.group(1))
        if m_j1:
            self.J1 = float(m_j1.group(1))

    def load_lcf(self, path: str) -> None:
        from flir_lcf import LensCorrectionMap

        self._lcf = LensCorrectionMap.from_fff(path)

    @property
    def lcf_active(self) -> bool:
        return self._lcf is not None and self.lcf_enabled

    # ------------------------------------------------------------ conversion --

    @staticmethod
    def _planck(temp_k, r: float, b: float, f: float):
        tk = np.asarray(temp_k, dtype=np.float64)
        with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
            return r / (np.exp(b / tk) - f)

    def planck_signal(self, temp_k):
        """S(T) = R / (e^(B/T) - F) on the active curve."""
        r, b, f, _gain, _offset = self._curve()
        s = self._planck(temp_k, r, b, f)
        return float(s) if np.ndim(s) == 0 else s

    def _environment(self) -> Tuple[float, float]:
        eps = max(float(self.emissivity), 1e-3)
        tau = float(self.tau_atmosphere)
        if not 0.0 < tau <= 1.0:
            tau = 1.0
        return eps, tau

    def raw_to_celsius(self, raw_array: np.ndarray) -> np.ndarray:
        """Convert raw14 counts to Celsius on the active curve, with eps / T_refl / tau."""
        r, b, f, gain, offset = self._curve()
        s = (np.asarray(raw_array, dtype=np.float64) - offset) / gain
        eps, tau = self._environment()
        if eps != 1.0 or tau != 1.0:
            s_refl = self._planck(self.reflected_temp_k, r, b, f)
            s_atm = self._planck(self.atmospheric_temp_k, r, b, f)
            s = (s - (1.0 - eps) * tau * s_refl - (1.0 - tau) * s_atm) / (eps * tau)
        s = np.maximum(s, 1e-6)  # keep the logarithm defined
        ratio = np.maximum(r / s + f, 1.0001)
        return (b / np.log(ratio) - 273.15).astype(np.float32)

    def celsius_to_raw(self, temp_c: np.ndarray) -> np.ndarray:
        """Inverse of raw_to_celsius (to the nearest count)."""
        r, b, f, gain, offset = self._curve()
        s = self._planck(np.asarray(temp_c, dtype=np.float64) + 273.15, r, b, f)
        eps, tau = self._environment()
        if eps != 1.0 or tau != 1.0:
            s_refl = self._planck(self.reflected_temp_k, r, b, f)
            s_atm = self._planck(self.atmospheric_temp_k, r, b, f)
            s = eps * tau * s + (1.0 - eps) * tau * s_refl + (1.0 - tau) * s_atm
        raw = s * gain + offset
        return np.clip(np.rint(raw), 0, 0x3FFF).astype(np.uint16)

    def perform_ffc(self, raw_array: np.ndarray, shutter_temp_k: float) -> None:
        """
        Shutter-offset fallback: estimate the offset that maps the closed-shutter
        frame mean onto the shutter's expected signal. No effect on the RBFO path,
        where the Lepton's own FFC does this job.
        """
        if shutter_temp_k <= 0:
            return

        exponent = self.rsc_B / shutter_temp_k
        if exponent > 20:  # Prevent overflow
            s_target = 0.0
        else:
            s_target = self.rsc_R / (math.exp(exponent) - self.rsc_F)

        mean_raw = float(np.mean(raw_array))
        # S_obj = (Raw - Offset) / Gain  =>  Offset = Raw - (S_obj * Gain)
        instant_offset = mean_raw - (s_target * self.linear_gain)

        # Smooth and reject outliers (e.g., shutter opening)
        if len(self._ffc_history) >= 5:
            median_offset = float(np.median(self._ffc_history))
            if abs(instant_offset - median_offset) > 150:
                # Reject this frame; the physical shutter is likely sliding open/closed
                return

        self._ffc_history.append(instant_offset)
        if len(self._ffc_history) > 15:
            self._ffc_history.pop(0)

        self.dynamic_offset = float(np.mean(self._ffc_history))

    def apply_lcf(self, image: np.ndarray) -> np.ndarray:
        """Geometric lens correction of a sensor-oriented (60x80) frame, if enabled."""
        if not self.lcf_active:
            return image
        return self._lcf.remap(image)

    def get_fusion_affine_matrix(self, visual_shape: Tuple[int, int], thermal_shape: Tuple[int, int]) -> np.ndarray:
        """
        Computes the 2x3 affine transformation matrix to align the 640x480 visual camera
        image onto the 80x60 thermal image using factory fusion calibration parameters.
        """
        import cv2
        vis_h, vis_w = visual_shape[:2]
        thm_h, thm_w = thermal_shape[:2]
        
        # Base scaling factor from visual resolution to thermal resolution
        sx = thm_w / float(vis_w) * self.fusion_magnification
        sy = thm_h / float(vis_h) * self.fusion_magnification
        
        # Center of visual image
        center_x = vis_w / 2.0
        center_y = vis_h / 2.0
        
        # Rotation matrix around visual center
        rot_mat = cv2.getRotationMatrix2D((center_x, center_y), self.fusion_rotation_deg, 1.0)
        
        # Apply translation and scaling
        # Offset translation derived from C0X, C1X, C1Y calibration
        tx = (thm_w / 2.0) - (center_x * sx) + self.fusion_c0x
        ty = (thm_h / 2.0) - (center_y * sy) + self.fusion_c1y
        
        m = np.array([
            [rot_mat[0, 0] * sx, rot_mat[0, 1] * sx, tx],
            [rot_mat[1, 0] * sy, rot_mat[1, 1] * sy, ty]
        ], dtype=np.float32)
        
        return m
