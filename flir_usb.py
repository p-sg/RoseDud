"""
FLIR One USB Device Interface & Worker Streamer
Pure-USB (no UART required) implementation.
Uses native WinUSB on Windows; PyUSB/libusb fallback elsewhere.
"""

import queue
import sys
import threading
import time
from typing import Optional, Protocol

import cv2
import numpy as np

from flir_calib import FlirCalibration
from flir_protocol import RosebudParser, RosebudFrame


class UsbTransport(Protocol):
    def connect(self, timeout_sec: float = 15.0) -> bool: ...
    def read_fileio(self, size: int = 4096, timeout_ms: int = 50) -> bytes: ...
    def read_frame(self, size: int = 65536, timeout_ms: int = 500) -> bytes: ...
    def close(self) -> None: ...


def _create_transport(log_fn=None, full_startup: bool = True, debug: bool = False) -> Optional[UsbTransport]:
    if sys.platform == "win32":
        try:
            from winusb_backend import WinUsbTransport
            t = WinUsbTransport(log_fn=log_fn, full_startup=full_startup, debug=debug)
            if t.is_available():
                return t
        except Exception:
            pass
        return None
    try:
        from libusb_backend import LibUsbTransport
        t = LibUsbTransport(log_fn=log_fn, full_startup=full_startup)
        if t.is_available():
            return t
    except Exception:
        pass
    return None


class FlirUsbStreamer:
    def __init__(
        self,
        calib: Optional[FlirCalibration] = None,
        mock_mode: bool = False,
        full_startup: bool = False,
        debug: bool = False,
    ):
        self.calib = calib or FlirCalibration()
        self.mock_mode = mock_mode
        self.full_startup = full_startup
        self.debug = debug
        self.parser = RosebudParser(self.calib)

        self.frame_queue: queue.Queue[RosebudFrame] = queue.Queue(maxsize=30)
        self.is_running = False
        self._run_id = 0
        self.worker_thread: Optional[threading.Thread] = None
        self.fileio_thread: Optional[threading.Thread] = None

        self.transport: Optional[UsbTransport] = None
        self.device_connected = False
        self.error_message = ""
        self._cached_sled_info: Optional[dict] = None

        # No-new-frames watchdog: if we haven't received a parsed frame for a while,
        # the EP0x85 producer is likely stalled. Recover by reconnecting.
        self.no_frame_timeout_sec = 5.0
        self._last_frame_time: float = 0.0
        self._watchdog_thread: Optional[threading.Thread] = None
        self._reconnect_lock = threading.Lock()

    def connect(self, timeout_sec: float = 15.0) -> bool:
        if self.mock_mode:
            self.device_connected = True
            return True

        self.transport = _create_transport(
            log_fn=lambda msg: print(f"[USB] {msg}"),
            full_startup=self.full_startup,
            debug=self.debug
        )
        if self.transport is None:
            self.error_message = (
                "No USB backend available. On Windows install WinUSB via Zadig; "
                "else: pip install pyusb libusb-package"
            )
            return False

        print("[USB] Connecting to FLIR One Camera (0x09CB:0x1996)...")
        if not self.transport.connect(timeout_sec=timeout_sec):
            self.error_message = (
                "Failed to connect to FLIR One. Ensure the camera is powered on, "
                "WinUSB is installed on all 3 interfaces, and no other app holds the device."
            )
            self.transport.close()
            self.transport = None
            return False

        self.device_connected = True
        return True

    def start(self) -> None:
        if self.is_running:
            return

        self.is_running = True
        # Each start() gets a new run id so threads left over from a previous
        # run (e.g. across a watchdog reconnect) exit instead of running twice.
        self._run_id += 1
        run_id = self._run_id
        self._last_frame_time = time.time()

        if self.mock_mode or self.transport is None:
            self.worker_thread = threading.Thread(target=self._mock_stream_loop, args=(run_id,), daemon=True)
            self.worker_thread.start()
            return

        # We MUST slowly drain FileIO (EP 0x83) to prevent the camera's usbiap2_poc
        # daemon from deadlocking on write(), while pacing it to prevent CPU starvation.
        self.fileio_thread = threading.Thread(target=self._fileio_drain_loop, args=(run_id,), daemon=True)
        self.fileio_thread.start()

        self.worker_thread = threading.Thread(target=self._usb_stream_loop, args=(run_id,), daemon=True)
        self.worker_thread.start()

        # Start watchdog only for real USB streaming.
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, args=(run_id,), daemon=True)
        self._watchdog_thread.start()

    def _active(self, run_id: int) -> bool:
        return self.is_running and self._run_id == run_id

    def stop(self) -> None:
        self.is_running = False
        current = threading.current_thread()
        # Join every thread that may touch the transport before closing it.
        # The watchdog calls stop() itself, so never join the calling thread.
        for thread, timeout in (
            (self.fileio_thread, 1.0),
            (self.worker_thread, 3.0),
            (self._watchdog_thread, 2.0),
        ):
            if thread and thread is not current and thread.is_alive():
                thread.join(timeout=timeout)

        if self.transport:
            self.transport.close()
            self.transport = None
        self.device_connected = False

    def _fileio_drain_loop(self, run_id: int) -> None:
        while self._active(run_id):
            transport = self.transport
            if self.device_connected and transport:
                try:
                    transport.read_fileio(size=4096, timeout_ms=10)
                except Exception:
                    pass
            time.sleep(0.1)

    def _watchdog_loop(self, run_id: int) -> None:
        # Runs alongside the USB streaming threads.
        while self._active(run_id):
            time.sleep(1.0)
            if self._last_frame_time <= 0:
                continue
            if time.time() - self._last_frame_time < self.no_frame_timeout_sec:
                continue

            # Stall detected; attempt a reconnect once.
            if not self._reconnect_lock.acquire(blocking=False):
                continue

            try:
                print(f"[USB] Watchdog: no frames for {self.no_frame_timeout_sec:.1f}s; reconnecting...")

                # Stop threads/transports cleanly.
                self.stop()

                # Clear any partial/old frames.
                self.frame_queue = queue.Queue(maxsize=30)
                self._last_frame_time = 0.0

                # Reconnect and resume streaming.
                if self.connect(timeout_sec=15.0):
                    self.start()
                    if self.debug:
                        print(f"[USB] Watchdog reconnected; {threading.active_count()} threads alive")
                else:
                    print("[USB] Watchdog reconnect failed.")

            finally:
                try:
                    self._reconnect_lock.release()
                except Exception:
                    pass
            return

    @property
    def sled_info(self) -> Optional[dict]:
        if self.transport and hasattr(self.transport, 'link_mgr') and self.transport.link_mgr:
            info = getattr(self.transport.link_mgr, 'sled_info', None)
            if info:
                self._cached_sled_info = info
                return info
        return self._cached_sled_info

    def get_latest_frame(self, timeout: float = 0.5) -> Optional[RosebudFrame]:
        try:
            return self.frame_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _usb_stream_loop(self, run_id: int) -> None:
        while self._active(run_id):
            transport = self.transport
            if transport is None:
                break
            # Use a longer timeout to avoid repeatedly aborting WinUSB pipes.
            # This improves long-run stability of the streaming pipeline.
            chunk = transport.read_frame(size=65536, timeout_ms=2000)
            if chunk:
                for frame in self.parser.feed(chunk):
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                    self.frame_queue.put(frame)
                    self._last_frame_time = time.time()
            else:
                time.sleep(0.01)

    def _mock_stream_loop(self, run_id: int) -> None:
        frame_idx = 0
        t0 = time.time()
        # Synthesize raw counts with a fixed default calibration so the UI's
        # offset/gain sliders still visibly affect the mock readings.
        mock_calib = FlirCalibration()

        while self._active(run_id):
            now = time.time()
            elapsed = now - t0

            y, x = np.mgrid[0:60, 0:80]

            cx = 40 + 20 * np.sin(elapsed * 0.8)
            cy = 30 + 12 * np.cos(elapsed * 0.8)
            dist_sq = (x - cx) ** 2 + (y - cy) ** 2
            warm_spot = np.exp(-dist_sq / 120.0) * (37.0 - 22.0)

            gradient = (y / 60.0) * 3.0
            noise = np.random.normal(0, 0.15, (60, 80))

            scene_c = 22.0 + warm_spot + gradient + noise
            thermal_raw = mock_calib.celsius_to_raw(scene_c)
            thermal_celsius = self.calib.raw_to_celsius(thermal_raw)

            vis_img = np.zeros((480, 640, 3), dtype=np.uint8)
            vis_img[:] = (60, 50, 45)
            vis_cx = int(cx * 8)
            vis_cy = int(cy * 8)
            cv2.circle(vis_img, (vis_cx, vis_cy), 70, (180, 190, 210), -1)
            cv2.circle(vis_img, (vis_cx, vis_cy), 70, (40, 40, 50), 3)
            cv2.putText(
                vis_img, "FLIR ONE Visual Stream (Mock)", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
            )

            frame = RosebudFrame(
                frame_index=frame_idx,
                thermal_raw=thermal_raw,
                thermal_celsius=thermal_celsius,
                visual_image=vis_img,
                metadata={
                    "shutterState": "open",
                    "shutterTemperature": 298.15,
                    "ffcState": "FFC_RAD_VALID",
                },
                shutter_state="open",
                shutter_temp_k=298.15,
                ffc_state="FFC_RAD_VALID",
                min_temp_c=float(np.min(thermal_celsius)),
                max_temp_c=float(np.max(thermal_celsius)),
                center_temp_c=float(thermal_celsius[30, 40]),
                timestamp_notified=now,
                timestamp_enqueued=now,
            )

            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            self.frame_queue.put(frame)

            frame_idx += 1
            time.sleep(0.11)
