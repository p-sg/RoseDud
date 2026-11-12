"""
RoseDud: PyQt6 thermal & visual viewer for the FLIR One (Gen 1) sled.

Usage:
    python flir_app.py [--mock] [--debug] [--calib PATH] [--snapshot-dir PATH]
"""

import argparse
import os
import sys
import time
from typing import Tuple, Optional
import numpy as np
import cv2

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QCheckBox, QFrame,
    QSizePolicy, QSlider, QGroupBox, QScrollArea
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QPoint
from PyQt6.QtGui import QImage, QPixmap, QFont, QPainter, QPen, QColor, QBrush
import qdarkstyle

from flir_calib import FlirCalibration
from flir_protocol import RosebudFrame
from flir_usb import FlirUsbStreamer

APP_DIR = os.path.dirname(os.path.abspath(__file__))

# Color palettes
COLORMAPS = [
    ("Ironbow", "ironbow"),
    ("Inferno", cv2.COLORMAP_INFERNO),
    ("Turbo", cv2.COLORMAP_TURBO),
    ("Jet / Rainbow", cv2.COLORMAP_JET),
    ("Hot", cv2.COLORMAP_HOT),
    ("White-Hot", "whitehot"),
    ("Black-Hot", "blackhot"),
    ("Ocean", cv2.COLORMAP_OCEAN)
]

VIEW_MODES = [
    "Thermal View",
    "Visual View",
    "Dual Side-by-Side",
    "MSX Edge Fusion",
    "Alpha Blend Fusion"
]

def generate_ironbow_lut() -> np.ndarray:
    points = [
        (0.00, (0, 0, 0)),
        (0.15, (80, 0, 40)),
        (0.35, (180, 20, 100)),
        (0.60, (20, 40, 220)),
        (0.80, (0, 180, 255)),
        (0.95, (200, 255, 255)),
        (1.00, (255, 255, 255))
    ]
    lut = np.zeros((256, 1, 3), dtype=np.uint8)
    for i in range(256):
        pos = i / 255.0
        for j in range(len(points) - 1):
            p0, c0 = points[j]
            p1, c1 = points[j + 1]
            if p0 <= pos <= p1:
                t = (pos - p0) / (p1 - p0)
                b = int(c0[0] + t * (c1[0] - c0[0]))
                g = int(c0[1] + t * (c1[1] - c0[1]))
                r = int(c0[2] + t * (c1[2] - c0[2]))
                lut[i, 0] = [b, g, r]
                break
    return lut

IRONBOW_LUT = generate_ironbow_lut()

class XYPadWidget(QWidget):
    positionChanged = pyqtSignal(int, int)

    def __init__(self, parent=None, min_val=-50, max_val=50):
        super().__init__(parent)
        self.setMinimumSize(120, 120)
        self.min_val = min_val
        self.max_val = max_val
        
        self._x = 0
        self._y = 0
        
        self.setCursor(Qt.CursorShape.CrossCursor)

    def value_x(self):
        return self._x

    def value_y(self):
        return self._y

    def setValue(self, x, y):
        self._x = max(self.min_val, min(self.max_val, x))
        self._y = max(self.min_val, min(self.max_val, y))
        self.update()

    def _update_from_mouse(self, pos: QPoint):
        w = self.width() - 1
        h = self.height() - 1
        px = max(0, min(w, pos.x()))
        py = max(0, min(h, pos.y()))
        
        lx = self.min_val + (px / w) * (self.max_val - self.min_val)
        ly = self.min_val + (py / h) * (self.max_val - self.min_val)
        
        self._x = int(lx)
        self._y = int(ly)
        
        self.positionChanged.emit(self._x, self._y)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._update_from_mouse(event.pos())
        if event.button() == Qt.MouseButton.RightButton:
            self._update_from_mouse(QPoint(self.width() // 2, self.height() // 2))
            self.positionChanged.emit(0, 0)

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.MouseButton.LeftButton:
            self._update_from_mouse(event.pos())

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        painter.fillRect(self.rect(), QColor("#1e1e1e"))
        
        painter.setPen(QPen(QColor("#333333"), 1, Qt.PenStyle.DashLine))
        painter.drawLine(self.width() // 2, 0, self.width() // 2, self.height())
        painter.drawLine(0, self.height() // 2, self.width(), self.height() // 2)
        
        w = self.width() - 1
        h = self.height() - 1
        px = int((self._x - self.min_val) / (self.max_val - self.min_val) * w)
        py = int((self._y - self.min_val) / (self.max_val - self.min_val) * h)
        
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.setBrush(QBrush(QColor("#4caf50")))
        painter.drawEllipse(QPoint(px, py), 6, 6)

class VideoLabel(QLabel):
    mouse_moved = pyqtSignal(int, int)
    
    def __init__(self):
        super().__init__()
        self.setMouseTracking(True)
        self.setCursor(Qt.CursorShape.CrossCursor)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(1, 1)
        
    def mouseMoveEvent(self, event):
        pos = event.pos()
        # Map label coordinates to image coordinates if scaled
        self.mouse_moved.emit(pos.x(), pos.y())


class FlirViewerApp(QMainWindow):
    def __init__(self, streamer: FlirUsbStreamer, calib: FlirCalibration,
                 snapshot_dir: Optional[str] = None):
        super().__init__()
        self.streamer = streamer
        self.calib = calib
        self.snapshot_dir = snapshot_dir or os.path.join(APP_DIR, "snapshots")

        self.setWindowTitle("RoseDud - FLIR ONE Gen 1 Thermal Viewer")
        self.resize(1024, 768)
        
        # State
        self.colormap_idx = 0
        self.view_mode_idx = 0
        self.temp_unit_celsius = False
        self.show_crosshairs = True
        self.alpha_blend_weight = 0.55
        self.msx_offset_x = 0
        self.msx_offset_y = 0
        
        self.cursor_x = -1
        self.cursor_y = -1
        self.current_frame: Optional[RosebudFrame] = None
        self._new_frame = False

        self.t_min_smooth = 20.0
        self.t_max_smooth = 35.0
        
        self.fps_timer = time.time()
        self.fps_counter = 0
        self.current_fps = 0.0
        
        self._setup_ui()
        
        # Streaming Timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_frame)
        self.timer.start(33)  # ~30fps poll

    def _setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Left Side: Video Canvas with Overlay Layout
        video_layout = QVBoxLayout()
        video_layout.setContentsMargins(0, 0, 0, 0)
        
        self.lbl_calib_warning = QLabel("")
        self.lbl_calib_warning.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_calib_warning.setFont(QFont("Segoe UI", 12, QFont.Weight.Bold))
        self.lbl_calib_warning.hide()
        video_layout.addWidget(self.lbl_calib_warning)
        
        self.video_label = VideoLabel()
        self.video_label.setStyleSheet("background-color: black; border: 1px solid #444;")
        self.video_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.video_label.mouse_moved.connect(self._on_mouse_move)
        video_layout.addWidget(self.video_label, stretch=1)
        
        main_layout.addLayout(video_layout, stretch=3)
        
        # Right Side: Sidebar Scroll Area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFixedWidth(290)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        
        sidebar_frame = QWidget()
        sidebar_layout = QVBoxLayout(sidebar_frame)
        sidebar_layout.setContentsMargins(5, 5, 5, 5)
        sidebar_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        scroll_area.setWidget(sidebar_frame)
        main_layout.addWidget(scroll_area, stretch=0)
        
        # Settings
        lbl_font = QFont("Segoe UI", 10, QFont.Weight.Bold)
        
        lbl_controls = QLabel("CONTROLS")
        lbl_controls.setFont(lbl_font)
        sidebar_layout.addWidget(lbl_controls)
        
        self.cb_view_mode = QComboBox()
        self.cb_view_mode.addItems(VIEW_MODES)
        self.cb_view_mode.currentIndexChanged.connect(self._on_view_mode_changed)
        sidebar_layout.addWidget(QLabel("View Mode:"))
        sidebar_layout.addWidget(self.cb_view_mode)
        
        self.cb_colormap = QComboBox()
        self.cb_colormap.addItems([c[0] for c in COLORMAPS])
        self.cb_colormap.currentIndexChanged.connect(self._on_colormap_changed)
        sidebar_layout.addWidget(QLabel("Colormap:"))
        sidebar_layout.addWidget(self.cb_colormap)
        
        self.chk_celsius = QCheckBox("Celsius Units")
        self.chk_celsius.setChecked(False)
        self.chk_celsius.toggled.connect(self._on_celsius_toggled)
        sidebar_layout.addWidget(self.chk_celsius)
        
        self.chk_crosshairs = QCheckBox("Show Spot Crosshairs")
        self.chk_crosshairs.setChecked(True)
        self.chk_crosshairs.toggled.connect(self._on_crosshairs_toggled)
        sidebar_layout.addWidget(self.chk_crosshairs)
        
        btn_snapshot = QPushButton("Take Snapshot")
        btn_snapshot.clicked.connect(self._save_snapshot)
        sidebar_layout.addWidget(btn_snapshot)
        
        sidebar_layout.addSpacing(15)
        lbl_telemetry = QLabel("TELEMETRY")
        lbl_telemetry.setFont(lbl_font)
        sidebar_layout.addWidget(lbl_telemetry)
        
        # Dynamic Labels
        self.lbl_fps = QLabel("FPS: 0.0")
        self.lbl_frame_idx = QLabel("Frame: #0")
        self.lbl_calib_info = QLabel("Calib: Offset=-- Gain=--")
        self.lbl_shutter = QLabel("Shutter Temp: -- K")
        self.lbl_batt = QLabel("Battery: -- V (--%)")
        self.lbl_shutter_state = QLabel("Shutter State: --")
        
        for lbl in (self.lbl_fps, self.lbl_frame_idx, self.lbl_calib_info, self.lbl_shutter, self.lbl_batt, self.lbl_shutter_state):
            lbl.setFont(QFont("Segoe UI", 9))
            lbl.setWordWrap(True)
            sidebar_layout.addWidget(lbl)
            
        # Sled / Device Information Group (hidden until sledInformation is received)
        self.grp_sled_info = QGroupBox("Device Information")
        sled_layout = QVBoxLayout()
        sled_layout.setSpacing(4)
        
        self.lbl_sled_board = QLabel("Board: --")
        self.lbl_sled_lepton = QLabel("Lepton: --")
        self.lbl_sled_fw = QLabel("Firmware: --")
        self.lbl_sled_mode = QLabel("Mode: --")
        
        for lbl in (self.lbl_sled_board, self.lbl_sled_lepton, self.lbl_sled_fw, self.lbl_sled_mode):
            lbl.setFont(QFont("Segoe UI", 9))
            lbl.setWordWrap(True)
            sled_layout.addWidget(lbl)
            
        self.grp_sled_info.setLayout(sled_layout)
        self.grp_sled_info.hide()
        sidebar_layout.addWidget(self.grp_sled_info)
            
        # Interactive Tuning Controls
        tuning_group = QGroupBox("Tuning & Blending")
        tuning_layout = QVBoxLayout()
        
        self.lbl_msx = QLabel(f"MSX Alignment (X / Y): {self.msx_offset_x} / {self.msx_offset_y}")
        tuning_layout.addWidget(self.lbl_msx)
        self.pad_msx = XYPadWidget(min_val=-100, max_val=100)
        self.pad_msx.positionChanged.connect(self._on_msx_pad_changed)
        tuning_layout.addWidget(self.pad_msx, alignment=Qt.AlignmentFlag.AlignHCenter)
        
        self.lbl_alpha = QLabel(f"Alpha Blend Weight: {self.alpha_blend_weight}")
        tuning_layout.addWidget(self.lbl_alpha)
        self.slider_alpha = QSlider(Qt.Orientation.Horizontal)
        self.slider_alpha.setRange(0, 100)
        self.slider_alpha.setValue(int(self.alpha_blend_weight * 100))
        self.slider_alpha.valueChanged.connect(self._on_alpha_changed)
        tuning_layout.addWidget(self.slider_alpha)
        
        self.lbl_offset = QLabel(f"Temp Offset: {getattr(self.calib, 'dynamic_offset', 5928)}")
        tuning_layout.addWidget(self.lbl_offset)
        self.slider_offset = QSlider(Qt.Orientation.Horizontal)
        self.slider_offset.setRange(-1000, 7000)
        self.slider_offset.setValue(int(getattr(self.calib, 'dynamic_offset', 5928)))
        self.slider_offset.valueChanged.connect(self._on_offset_changed)
        tuning_layout.addWidget(self.slider_offset)
        
        self.lbl_gain = QLabel(f"Temp Gain: {getattr(self.calib, 'linear_gain', 64)}")
        tuning_layout.addWidget(self.lbl_gain)
        self.slider_gain = QSlider(Qt.Orientation.Horizontal)
        self.slider_gain.setRange(10, 150)
        self.slider_gain.setValue(int(getattr(self.calib, 'linear_gain', 64)))
        self.slider_gain.valueChanged.connect(self._on_gain_changed)
        tuning_layout.addWidget(self.slider_gain)
        
        tuning_group.setLayout(tuning_layout)
        sidebar_layout.addWidget(tuning_group)
        
        sidebar_layout.addStretch()

    def _on_msx_pad_changed(self, x, y):
        self.msx_offset_x = x
        self.msx_offset_y = y
        self.lbl_msx.setText(f"MSX Alignment (X / Y): {self.msx_offset_x} / {self.msx_offset_y}")
        
    def _on_alpha_changed(self, val):
        self.alpha_blend_weight = val / 100.0
        self.lbl_alpha.setText(f"Alpha Blend Weight: {self.alpha_blend_weight:.2f}")
        
    def _on_offset_changed(self, val):
        if hasattr(self.calib, 'dynamic_offset'):
            self.calib.dynamic_offset = float(val)
            self.lbl_offset.setText(f"Temp Offset: {self.calib.dynamic_offset}")
            
    def _on_gain_changed(self, val):
        if hasattr(self.calib, 'linear_gain'):
            self.calib.linear_gain = float(val)
            self.lbl_gain.setText(f"Temp Gain: {self.calib.linear_gain}")

    def _on_view_mode_changed(self, idx):
        self.view_mode_idx = idx
        
    def _on_colormap_changed(self, idx):
        self.colormap_idx = idx
        
    def _on_celsius_toggled(self, checked):
        self.temp_unit_celsius = checked
        
    def _on_crosshairs_toggled(self, checked):
        self.show_crosshairs = checked

    def _on_mouse_move(self, x, y):
        # We need to map UI label coordinates to the actual image coordinates
        if self.video_label.pixmap():
            pix_w = self.video_label.pixmap().width()
            pix_h = self.video_label.pixmap().height()
            
            lbl_w = self.video_label.width()
            lbl_h = self.video_label.height()
            
            x_off = (lbl_w - pix_w) // 2
            y_off = (lbl_h - pix_h) // 2
            
            scaled_x = x - x_off
            scaled_y = y - y_off
            
            mode = self.view_mode_idx
            orig_w = 960 if mode == 2 else 480
            orig_h = 640
            
            if pix_w > 0 and pix_h > 0:
                self.cursor_x = int(scaled_x * (orig_w / pix_w))
                self.cursor_y = int(scaled_y * (orig_h / pix_h))

    def _format_temp(self, temp_c: float) -> str:
        if self.temp_unit_celsius:
            return f"{temp_c:.1f} C"
        else:
            temp_f = (temp_c * 1.8) + 32.0
            return f"{temp_f:.1f} F"

    def _apply_colormap(self, temp_c_matrix: np.ndarray, t_min: float, t_max: float) -> np.ndarray:
        norm = np.clip((temp_c_matrix - t_min) / max(0.5, (t_max - t_min)), 0.0, 1.0)
        u8_gray = (norm * 255.0).astype(np.uint8)
        
        palette_type = COLORMAPS[self.colormap_idx][1]
        if palette_type == "ironbow":
            colored = cv2.applyColorMap(u8_gray, IRONBOW_LUT)
        elif palette_type == "whitehot":
            colored = cv2.cvtColor(u8_gray, cv2.COLOR_GRAY2BGR)
        elif palette_type == "blackhot":
            colored = cv2.cvtColor(255 - u8_gray, cv2.COLOR_GRAY2BGR)
        else:
            colored = cv2.applyColorMap(u8_gray, palette_type)
        return colored

    def _generate_msx_fusion(self, thermal_color: np.ndarray, visual_bgr: np.ndarray) -> np.ndarray:
        vis_resized = cv2.resize(visual_bgr, (thermal_color.shape[1], thermal_color.shape[0]))
        vis_gray = cv2.cvtColor(vis_resized, cv2.COLOR_BGR2GRAY)
        edges = cv2.Laplacian(vis_gray, cv2.CV_16S, ksize=3)
        edges = cv2.convertScaleAbs(edges, alpha=1.5)
        edges = cv2.GaussianBlur(edges, (3, 3), 0)
        edge_3c = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR) / 255.0
        fused = np.clip(thermal_color.astype(np.float32) * (1.0 - 0.5 * edge_3c) + (edge_3c * 255.0 * 0.4), 0, 255).astype(np.uint8)
        return fused

    def _update_frame(self):
        frame = self.streamer.get_latest_frame(timeout=0.01)
        self._new_frame = frame is not None
        if frame is not None:
            self.current_frame = frame

            # FPS Calculation
            self.fps_counter += 1
            now = time.time()
            if now - self.fps_timer >= 1.0:
                self.current_fps = self.fps_counter / (now - self.fps_timer)
                self.fps_counter = 0
                self.fps_timer = now
                self.lbl_fps.setText(f"FPS: {self.current_fps:.1f}")

        if self.current_frame is not None:
            rendered_view = self._render_cv2(self.current_frame)
            # Convert BGR to RGB for PyQt
            rgb_image = cv2.cvtColor(rendered_view, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            q_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pix = QPixmap.fromImage(q_img)
            # Scale pixmap to fit the video_label while keeping aspect ratio
            scaled_pix = pix.scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.video_label.setPixmap(scaled_pix)
            
        self._update_telemetry(self.current_frame)

    def _render_cv2(self, frame: RosebudFrame) -> np.ndarray:
        temp_c = cv2.rotate(frame.thermal_celsius, cv2.ROTATE_90_CLOCKWISE)
        raw_thm = cv2.rotate(frame.thermal_raw, cv2.ROTATE_90_CLOCKWISE)
        
        frame_min = float(np.min(temp_c))
        frame_max = float(np.max(temp_c))
        self.t_min_smooth = 0.85 * self.t_min_smooth + 0.15 * frame_min
        self.t_max_smooth = 0.85 * self.t_max_smooth + 0.15 * frame_max
        
        thermal_80x60 = self._apply_colormap(temp_c, self.t_min_smooth, self.t_max_smooth)
        thermal_upscaled = cv2.resize(thermal_80x60, (480, 640), interpolation=cv2.INTER_CUBIC)
        
        visual_img = frame.visual_image
        if visual_img is None:
            visual_img = np.zeros((640, 480, 3), dtype=np.uint8)
            cv2.putText(visual_img, "No Visual Stream", (130, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (150, 150, 150), 2)
        else:
            visual_img = cv2.rotate(visual_img, cv2.ROTATE_90_CLOCKWISE)
            if visual_img.shape[:2] != (640, 480):
                visual_img = cv2.resize(visual_img, (480, 640))
                
        if self.msx_offset_x != 0 or self.msx_offset_y != 0:
            M = np.float32([[1, 0, self.msx_offset_x], [0, 1, self.msx_offset_y]])
            visual_img = cv2.warpAffine(visual_img, M, (480, 640))
            
        mode = self.view_mode_idx
        if mode == 0:
            main_canvas = thermal_upscaled.copy()
            active_w, active_h = 480, 640
        elif mode == 1:
            main_canvas = visual_img.copy()
            active_w, active_h = 480, 640
        elif mode == 2:
            main_canvas = np.hstack([thermal_upscaled, visual_img])
            active_w, active_h = 960, 640
        elif mode == 3:
            main_canvas = self._generate_msx_fusion(thermal_upscaled, visual_img)
            active_w, active_h = 480, 640
        else:
            main_canvas = cv2.addWeighted(
                thermal_upscaled, self.alpha_blend_weight,
                visual_img, 1.0 - self.alpha_blend_weight, 0
            )
            active_w, active_h = 480, 640
            
        if self.show_crosshairs and mode != 1:
            min_loc = np.unravel_index(np.argmin(temp_c), temp_c.shape)
            max_loc = np.unravel_index(np.argmax(temp_c), temp_c.shape)
            
            scale_x = 480.0 / 60.0
            scale_y = 640.0 / 80.0
            
            max_pt = (int((max_loc[1] + 0.5) * scale_x), int((max_loc[0] + 0.5) * scale_y))
            min_pt = (int((min_loc[1] + 0.5) * scale_x), int((min_loc[0] + 0.5) * scale_y))
            center_pt = (240, 320)
            
            self._draw_crosshair(main_canvas, max_pt, (0, 0, 255), f"MAX: {self._format_temp(frame_max)}")
            self._draw_crosshair(main_canvas, min_pt, (255, 255, 0), f"MIN: {self._format_temp(frame_min)}")
            self._draw_crosshair(main_canvas, center_pt, (255, 255, 255), f"CTR: {self._format_temp(frame.center_temp_c)}")

        is_closed = frame.shutter_state.lower() in ("closed", "ffc")
        if is_closed:
            if "VALID" in frame.ffc_state.upper() or "APPROX" in frame.ffc_state.upper():
                # The UI redraws faster than frames arrive; feed each frame to FFC once.
                if self._new_frame:
                    self.calib.perform_ffc(frame.thermal_raw, frame.shutter_temp_k)
                self.lbl_calib_warning.setText("CALIBRATION COMPLETE! YOU MAY OPEN SHUTTER")
                self.lbl_calib_warning.setStyleSheet("color: white; background-color: #2e7d32; padding: 5px; border-radius: 5px;")
                self.lbl_calib_warning.show()
                self.slider_offset.setValue(int(getattr(self.calib, 'dynamic_offset', 5928)))
            else:
                self.lbl_calib_warning.setText("CALIBRATING... KEEP SHUTTER CLOSED")
                self.lbl_calib_warning.setStyleSheet("color: white; background-color: #c62828; padding: 5px; border-radius: 5px;")
                self.lbl_calib_warning.show()
        else:
            if "VALID" not in frame.ffc_state.upper() and "APPROX" not in frame.ffc_state.upper():
                self.lbl_calib_warning.setText("CALIBRATION NEEDED: PLEASE SLIDE SHUTTER CLOSED")
                self.lbl_calib_warning.setStyleSheet("color: white; background-color: #ef6c00; padding: 5px; border-radius: 5px;")
                self.lbl_calib_warning.show()
            else:
                self.lbl_calib_warning.hide()

        if 0 <= self.cursor_x < active_w and 0 <= self.cursor_y < active_h:
            # In side-by-side mode the right half is the visual image; map it
            # back onto the same thermal pixel as the left half.
            img_x = self.cursor_x % 480
            thm_px_x = int(np.clip(img_x / (480.0 / 60.0), 0, 59))
            thm_px_y = int(np.clip(self.cursor_y / (640.0 / 80.0), 0, 79))
            spot_temp = temp_c[thm_px_y, thm_px_x]
            
            cv2.drawMarker(main_canvas, (self.cursor_x, self.cursor_y), (0, 255, 0), cv2.MARKER_TILTED_CROSS, 20, 2)
            cv2.circle(main_canvas, (self.cursor_x, self.cursor_y), 8, (0, 255, 0), 1)
            
            txt = f"{self._format_temp(spot_temp)} (raw: {raw_thm[thm_px_y, thm_px_x]})"
            (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            bx, by = self.cursor_x + 12, max(25, self.cursor_y - 12)
            cv2.rectangle(main_canvas, (bx - 4, by - th - 4), (bx + tw + 4, by + 6), (20, 20, 20), -1)
            cv2.rectangle(main_canvas, (bx - 4, by - th - 4), (bx + tw + 4, by + 6), (0, 255, 0), 1)
            cv2.putText(main_canvas, txt, (bx, by), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        return main_canvas

    def _draw_crosshair(self, img: np.ndarray, pt: Tuple[int, int], color: Tuple[int, int, int], label: str) -> None:
        x, y = pt
        cv2.drawMarker(img, (x, y), color, cv2.MARKER_CROSS, 16, 2)
        cv2.putText(img, label, (x + 10, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, label, (x + 10, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

    def _update_telemetry(self, frame: Optional[RosebudFrame]):
        if frame is not None:
            self.lbl_frame_idx.setText(f"Frame: #{frame.frame_index}")
            
            if hasattr(self.calib, 'dynamic_offset'):
                self.lbl_calib_info.setText(
                    f"Calib: Offset={int(self.calib.dynamic_offset)} Gain={int(self.calib.linear_gain)}\n"
                    f"Planck: R={int(getattr(self.calib, 'R', 0))} B={int(getattr(self.calib, 'B', 0))} F={int(getattr(self.calib, 'F', 0))}"
                )
            
            tk = frame.shutter_temp_k
            tc = tk - 273.15
            if self.temp_unit_celsius:
                self.lbl_shutter.setText(f"Shutter Temp: {tk:.2f} K ({tc:.1f} C)")
            else:
                tf = (tc * 1.8) + 32.0
                self.lbl_shutter.setText(f"Shutter Temp: {tk:.2f} K ({tf:.1f} F)")
                
            self.lbl_shutter_state.setText(f"Shutter State: {frame.shutter_state} / {frame.ffc_state}")
        
        # Battery telemetry update
        if hasattr(self.streamer, 'transport') and self.streamer.transport is not None:
            if hasattr(self.streamer.transport, 'link_mgr') and self.streamer.transport.link_mgr is not None:
                link = self.streamer.transport.link_mgr
                v = getattr(link, 'battery_voltage', 0.0)
                p = getattr(link, 'battery_percentage', 0.0)
                raw_s = getattr(link, 'battery_charging_state', "Unknown")
                c = getattr(link, 'battery_charging_current', 0.0)
                
                state_map = {
                    "stateNoCharging": "Not Charging",
                    "stateManagedBatteryCharging": "Charging"
                }
                s = state_map.get(raw_s, raw_s.replace("state", ""))
                
                if hasattr(self.streamer, 'debug') and self.streamer.debug:
                    print(f"[UI] Read battery from link_mgr: {v:.2f}V {p}% - {s} at {c}mA")
                
                if v > 0 or p > 0 or s != "Unknown":
                    if s == "Charging":
                        self.lbl_batt.setText(f"Battery: {v:.2f} V ({p:.0f}%) - {s} ({c} mA)")
                    else:
                        self.lbl_batt.setText(f"Battery: {v:.2f} V ({p:.0f}%) - {s}")

        # Sled / Device Information update
        sled = getattr(self.streamer, 'sled_info', None)
        if not sled and hasattr(self.streamer, 'transport') and self.streamer.transport:
            if hasattr(self.streamer.transport, 'link_mgr') and self.streamer.transport.link_mgr is not None:
                sled = getattr(self.streamer.transport.link_mgr, 'sled_info', None)

        if sled:
            if self.grp_sled_info.isHidden():
                self.grp_sled_info.show()

            board_sn = sled.get("serialNumberBoard", "")
            board_rev = sled.get("versionBoard", "")
            board_pn = sled.get("partNumberBoard", "")
            board_parts = []
            if board_sn:
                board_parts.append(f"Board: {board_sn}" + (f" (Rev {board_rev})" if board_rev else ""))
            if board_pn:
                board_parts.append(f"P/N: {board_pn}")
            self.lbl_sled_board.setText("\n".join(board_parts) if board_parts else "Board: --")

            lep_sn = sled.get("serialNumberLepton", "")
            lep_ver = sled.get("versionLepton", "")
            lep_qr = sled.get("leptonQR", "")
            lep_parts = []
            if lep_sn:
                lep_parts.append(f"Lepton: S/N {lep_sn}" + (f" (v{lep_ver})" if lep_ver else ""))
            if lep_qr and lep_qr != lep_sn:
                lep_parts.append(f"QR: {lep_qr}")
            self.lbl_sled_lepton.setText("\n".join(lep_parts) if lep_parts else "Lepton: --")

            esw_op = sled.get("versionRosebudOperationalESW", "")
            esw_fac = sled.get("versionRosebudFactoryESW", "")
            api_rev = sled.get("versionRosebudAPI", "") or sled.get("gitRevision", "")
            fw_parts = []
            if esw_op:
                fw_parts.append(f"Rosebud: v{esw_op}" + (f" (Fac: {esw_fac})" if esw_fac else ""))
            if api_rev:
                fw_parts.append(f"API: {api_rev}")
            self.lbl_sled_fw.setText("\n".join(fw_parts) if fw_parts else "Firmware: --")

            mode = sled.get("operatingMode", "")
            self.lbl_sled_mode.setText(f"Mode: {mode}" if mode else "Mode: --")
        else:
            if not self.grp_sled_info.isHidden():
                self.grp_sled_info.hide()

    def _save_snapshot(self) -> None:
        if self.current_frame is None:
            return
        os.makedirs(self.snapshot_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        
        temp_c = cv2.rotate(self.current_frame.thermal_celsius, cv2.ROTATE_90_CLOCKWISE)
        csv_path = os.path.join(self.snapshot_dir, f"thermal_{ts}.csv")
        np.savetxt(csv_path, temp_c, delimiter=",", fmt="%.2f")
        
        thm_color = self._apply_colormap(temp_c, self.t_min_smooth, self.t_max_smooth)
        thm_path = os.path.join(self.snapshot_dir, f"thermal_{ts}.png")
        cv2.imwrite(thm_path, cv2.resize(thm_color, (480, 640), interpolation=cv2.INTER_CUBIC))
        
        if self.current_frame.visual_image is not None:
            vis = cv2.rotate(self.current_frame.visual_image, cv2.ROTATE_90_CLOCKWISE)
            vis_path = os.path.join(self.snapshot_dir, f"visual_{ts}.jpg")
            cv2.imwrite(vis_path, vis)
            
        print(f"\n[Snapshot Saved]: {csv_path}, {thm_path}")

    def closeEvent(self, event):
        self.streamer.stop()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="RoseDud: FLIR One (Gen 1) thermal & visual viewer")
    parser.add_argument("--mock", action="store_true", help="Run with synthetic thermal stream (for offline testing)")
    parser.add_argument("--calib", type=str, default=os.path.join(APP_DIR, "calib.rsc"),
                        help="Path to calib.rsc (default: next to this script)")
    parser.add_argument("--snapshot-dir", type=str, default=os.path.join(APP_DIR, "snapshots"),
                        help="Where snapshots are saved (default: snapshots/ next to this script)")
    parser.add_argument("--debug", action="store_true", help="Enable verbose debug logging")
    args = parser.parse_args()
    
    if not os.path.exists(args.calib):
        print(f"[Notice]: calibration file not found at {args.calib}; using built-in defaults.")
    calib = FlirCalibration(args.calib if os.path.exists(args.calib) else None)
    streamer = FlirUsbStreamer(calib=calib, mock_mode=args.mock, full_startup=True, debug=args.debug)
    
    if not args.mock:
        if not streamer.connect():
            print(f"\n[Notice]: {streamer.error_message}")
            print("Launching in synthetic MOCK mode for preview...\n")
            streamer.mock_mode = True
            streamer.connect()
            
    streamer.start()
            
    app = QApplication(sys.argv)
    
    # Apply a nice modern dark theme using qdarkstyle
    app.setStyleSheet(qdarkstyle.load_stylesheet(qt_api='pyqt6'))
    
    viewer = FlirViewerApp(streamer=streamer, calib=calib, snapshot_dir=args.snapshot_dir)
    viewer.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
