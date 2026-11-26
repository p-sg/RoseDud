"""Headless checks of the PyQt6 viewer: sled info panel and frame rendering."""

import sys

import pytest

pytest.importorskip("PyQt6")
from PyQt6.QtWidgets import QApplication  # noqa: E402

from conftest import SAMPLE_SLED_INFO  # noqa: E402
from flir_app import VIEW_MODES, FlirViewerApp  # noqa: E402
from flir_calib import FlirCalibration  # noqa: E402
from flir_usb import FlirUsbStreamer  # noqa: E402


@pytest.fixture
def viewer(tmp_path):
    app = QApplication.instance() or QApplication(sys.argv)
    calib = FlirCalibration()
    streamer = FlirUsbStreamer(calib=calib, mock_mode=True)
    v = FlirViewerApp(streamer=streamer, calib=calib, snapshot_dir=str(tmp_path))
    v.timer.stop()
    v.show()
    yield v
    v.close()
    app.processEvents()


def test_vospi_telemetry_dialog_opens(viewer):
    assert viewer._vospi_dialog is None
    viewer._open_vospi_telemetry()
    assert viewer._vospi_dialog is not None
    assert viewer._vospi_dialog.isVisible()
    viewer._vospi_dialog.close()


def test_sled_info_panel_visibility(viewer):
    assert viewer.grp_sled_info.isHidden()
    viewer._update_telemetry(None)
    assert viewer.grp_sled_info.isHidden()

    viewer.streamer._cached_sled_info = SAMPLE_SLED_INFO
    viewer._update_telemetry(None)
    assert not viewer.grp_sled_info.isHidden()
    assert "TEST00001" in viewer.lbl_sled_board.text()
    assert "Rev N" in viewer.lbl_sled_board.text()
    assert "00000000000" in viewer.lbl_sled_board.text()
    assert "0.3.47" in viewer.lbl_sled_lepton.text()
    assert "1.0.11" in viewer.lbl_sled_fw.text()
    assert "operational" in viewer.lbl_sled_mode.text()

    viewer.streamer._cached_sled_info = None
    viewer._update_telemetry(None)
    assert viewer.grp_sled_info.isHidden()


def test_render_all_modes_and_snapshot(viewer, tmp_path):
    streamer = viewer.streamer
    streamer.connect()
    streamer.start()
    try:
        frame = streamer.get_latest_frame(timeout=2.0)
    finally:
        streamer.stop()
    assert frame is not None

    for mode, _name in enumerate(VIEW_MODES):
        viewer.view_mode_idx = mode
        # Hover the right half in side-by-side mode to cover the visual-side readout.
        viewer.cursor_x, viewer.cursor_y = (700 if mode == 2 else 200), 300
        canvas = viewer._render_cv2(frame)
        assert canvas.shape == ((640, 960, 3) if mode == 2 else (640, 480, 3))

    viewer.current_frame = frame
    viewer._save_snapshot()
    names = sorted(p.suffix for p in tmp_path.iterdir())
    assert names == [".csv", ".jpg", ".png"]
