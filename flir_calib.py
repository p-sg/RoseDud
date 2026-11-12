"""
FLIR One Calibration Parser and Radiometric Temperature Processor
Extracts factory calibration constants (Planck R, B, F, J0, J1) and visual fusion parameters
from calib.rsc in CameraFiles.
"""

import math
import os
from typing import Dict, Any, Optional, Tuple
import numpy as np


class FlirCalibration:
    def __init__(self, calib_rsc_path: Optional[str] = None):
        self.raw_dict: Dict[str, Any] = {}
        
        # Planck Equation Constants: T_obj = B / ln(R1 / (R2 * (S - O)) + F) - 273.15
        # Factory defaults for FLIR One Gen 1 Lepton sensor
        self.R: float = 16728.0
        self.B: float = 1431.1
        self.F: float = 1.0
        self.J0: float = 3027.0
        self.J1: float = 77.408089
        
        # Radiometric measurement defaults
        self.emissivity: float = 0.98       # Default human/matte object emissivity
        self.reflected_temp_k: float = 293.15  # Reflected ambient temp (20 C)
        self.atmospheric_temp_k: float = 293.15
        self.tau_atmosphere: float = 1.0     # Atmospheric transmission
        
        # Visual/Thermal Fusion Alignment Parameters (from calib.rsc)
        self.fusion_c0x: float = -5.0744071
        self.fusion_c0y: float = 0.0
        self.fusion_c1x: float = -17.791127
        self.fusion_c1y: float = -24.672901
        self.fusion_magnification: float = 1.2822002
        self.fusion_rotation_deg: float = -0.33073297
        self.fusion_fix_distance: float = 3.0  # meters
        
        # Lens distortion correction
        self.lens_rotation: float = 0.1657029
        self.lens_zoom: float = 5.7087041
        
        # Radiometric calculation defaults
        self.linear_gain: float = 25.0
        self.dynamic_offset: float = 3200.0
        self._ffc_history = []
        
        if calib_rsc_path and os.path.exists(calib_rsc_path):
            self.load_from_file(calib_rsc_path)
            
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
        # Default Planck constants (FLIR One Gen 1, Lepton 1.x 80x60)
        self.R = 16728.0
        self.B = 1431.1
        self.F = 1.0
        self.J0 = 3027.0
        self.J1 = 77.408
        
        # Empirical Gain (can be tuned with O/P keys, but 25.0 is a good baseline)
        self.linear_gain = 25.0
        
        # Dynamic Radiometric Offset (automatically calibrated during FFC)
        self.dynamic_offset = 3200.0
        
        # Check for Planck constants string e.g. "R 16728, B 1431.1, F 1.00 (J0 3027, J1 77.408089)"
        for k, v in self.raw_dict.items():
            if "const" in k and isinstance(v, str) and "R " in v:
                self._parse_planck_const_string(v)
                
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

    def _parse_planck_const_string(self, s: str) -> None:
        """Parses R, B, F, J0, J1 from text string."""
        import re
        m_r = re.search(r"R\s+([0-9.]+)", s)
        m_b = re.search(r"B\s+([0-9.]+)", s)
        m_f = re.search(r"F\s+([0-9.]+)", s)
        m_j0 = re.search(r"J0\s+([0-9.]+)", s)
        m_j1 = re.search(r"J1\s+([0-9.]+)", s)
        
        if m_r: self.R = float(m_r.group(1))
        if m_b: self.B = float(m_b.group(1))
        if m_f: self.F = float(m_f.group(1))
        if m_j0: self.J0 = float(m_j0.group(1))
        if m_j1: self.J1 = float(m_j1.group(1))
        
        # Empirical Gain (can be tuned with O/P keys, but 25.0 is a good baseline)
        self.linear_gain = 25.0
        
        # Dynamic Radiometric Offset (automatically calibrated during FFC)
        self.dynamic_offset = 3200.0

    def perform_ffc(self, raw_array: np.ndarray, shutter_temp_k: float) -> None:
        """
        Calculates the theoretical radiometric signal for the shutter's temperature,
        and determines the global offset needed so that the raw image mean matches this signal.
        """
        if shutter_temp_k <= 0:
            return
            
        # Calculate theoretical signal (S_target) of the shutter blackbody
        exponent = self.B / shutter_temp_k
        if exponent > 20: # Prevent overflow
            s_target = 0
        else:
            s_target = self.R / (np.exp(exponent) - self.F)
            
        # Get the mean raw value of the scene (which is currently the shutter)
        mean_raw = np.mean(raw_array)
        
        # S_obj = (Raw - Offset) / Gain  =>  Offset = Raw - (S_obj * Gain)
        instant_offset = mean_raw - (s_target * self.linear_gain)
        
        # Smooth and reject outliers (e.g., shutter opening)
        if len(self._ffc_history) >= 5:
            median_offset = np.median(self._ffc_history)
            if abs(instant_offset - median_offset) > 150:
                # Reject this frame; the physical shutter is likely sliding open/closed
                return
                
        self._ffc_history.append(instant_offset)
        if len(self._ffc_history) > 15:
            self._ffc_history.pop(0)
            
        self.dynamic_offset = float(np.mean(self._ffc_history))

    def raw_to_celsius(self, raw_array: np.ndarray) -> np.ndarray:
        """
        Converts raw 14-bit counts to Celsius using the standard Planck curve
        and our dynamically calibrated FFC offset.
        """
        # 1. Convert Raw to Signal using our dynamic Offset and Gain
        # S_obj = (Raw - Offset) / Gain
        s_obj = (raw_array.astype(np.float32) - self.dynamic_offset) / self.linear_gain
        
        # Avoid division by zero or negative signals
        s_obj = np.maximum(s_obj, 1.0)
        
        # 2. Apply Planck's equation
        # T_kelvin = B / ln(R / S_obj + F)
        ratio = (self.R / s_obj) + self.F
        ratio = np.maximum(ratio, 1.0001)  # Prevent ln(<=0)
        temp_k = self.B / np.log(ratio)
        
        return temp_k - 273.15

    def celsius_to_raw(self, temp_c: np.ndarray) -> np.ndarray:
        """Inverse of raw_to_celsius: Celsius to raw counts with the current offset and gain."""
        temp_k = np.asarray(temp_c, dtype=np.float64) + 273.15
        s_obj = self.R / (np.exp(self.B / temp_k) - self.F)
        raw = s_obj * self.linear_gain + self.dynamic_offset
        return np.clip(raw, 0, 0x3FFF).astype(np.uint16)

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
