"""FlirUsbStreamer mock mode and thread lifecycle."""

import time

from flir_calib import FlirCalibration
from flir_usb import FlirUsbStreamer


def test_mock_stream_produces_plausible_temperatures():
    streamer = FlirUsbStreamer(calib=FlirCalibration(), mock_mode=True)
    assert streamer.connect()
    streamer.start()
    try:
        frame = streamer.get_latest_frame(timeout=2.0)
    finally:
        streamer.stop()
    assert frame is not None
    assert frame.thermal_raw.shape == (60, 80)
    assert frame.visual_image.shape == (480, 640, 3)
    assert 15.0 < frame.min_temp_c < frame.max_temp_c < 45.0


def test_restart_leaves_no_stale_threads():
    streamer = FlirUsbStreamer(calib=FlirCalibration(), mock_mode=True)
    streamer.connect()
    streamer.start()
    first = streamer.worker_thread
    streamer.stop()
    streamer.start()
    try:
        time.sleep(0.3)
        assert not first.is_alive()
        assert streamer.worker_thread.is_alive()
    finally:
        streamer.stop()
    assert not streamer.worker_thread.is_alive()
