"""
FLIR One USB Device Interface & Worker Streamer
Pure-USB (no UART required) implementation.
Uses native WinUSB on Windows; PyUSB/libusb fallback elsewhere.
"""

import queue
import sys
import threading
import time
from typing import List, Optional, Protocol

import cv2
import numpy as np

from flir_calib import FlirCalibration
from flir_protocol import RosebudParser, RosebudFrame
from iap2 import (
    FILEIO_HEADER_SIZE,
    FILEIO_MAX_PAYLOAD,
    FileIOPacket,
    FileIOPacketParser,
    parse_open_file_status,
)


class UsbTransport(Protocol):
    def connect(self, timeout_sec: float = 15.0) -> bool: ...
    def read_fileio(self, size: int = 4096, timeout_ms: int = 50) -> bytes: ...
    def read_frame(self, size: int = 65536, timeout_ms: int = 500) -> bytes: ...
    def close(self) -> None: ...


class FileIOFetchError(RuntimeError):
    """Raised when open/read/close FileIO sequence fails or times out."""


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

        # FileIO (EP 0x83) demuxer — always drained; payloads routed while a fetch is active.
        self._fileio_parser = FileIOPacketParser()
        self._fileio_lock = threading.Lock()
        self._fileio_active_stream: Optional[int] = None
        self._fileio_chunks: queue.Queue = queue.Queue()
        self._fileio_unsolicited: List[bytes] = []
        self._fileio_logged_headers: List[bytes] = []

        # No-new-frames watchdog: if we haven't received a parsed frame for a while,
        # the EP0x85 producer is likely stalled. Recover by reconnecting.
        self.no_frame_timeout_sec = 5.0
        self._last_frame_time: float = 0.0
        self._watchdog_thread: Optional[threading.Thread] = None
        self._reconnect_lock = threading.Lock()
        # Held while fetch_file runs — watchdog must not tear down the transport.
        self._fileio_fetch_active = threading.Event()
        self._fetch_lock = threading.Lock()

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

        # Drain FileIO (EP 0x83) or the camera stalls on its IN write and frames stop.
        # Pace reads so this thread does not starve the frame loop.
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
                    with self._fileio_lock:
                        active = self._fileio_active_stream is not None
                    # Larger reads while a fetch is in flight so 128 KiB chunks reassemble promptly.
                    size = (FILEIO_HEADER_SIZE + FILEIO_MAX_PAYLOAD) if active else 4096
                    data = transport.read_fileio(size=size, timeout_ms=10)
                    if data:
                        self._handle_fileio_bytes(data)
                except Exception:
                    pass
            time.sleep(0.1)

    def _handle_fileio_bytes(self, data: bytes) -> None:
        with self._fileio_lock:
            active = self._fileio_active_stream
            if active is None:
                # Log a little of anything that arrives before/without an active readFile.
                if len(self._fileio_unsolicited) < 16:
                    self._fileio_unsolicited.append(bytes(data[:64]))
                # Still parse so the buffer cannot grow without bound on framed noise.
            packets = self._fileio_parser.feed(data)
            for pkt in packets:
                if len(self._fileio_logged_headers) < 8:
                    self._fileio_logged_headers.append(pkt.header_raw)
                if active is not None and pkt.stream_identifier == active:
                    self._fileio_chunks.put(pkt)

    def take_fileio_unsolicited(self) -> List[bytes]:
        with self._fileio_lock:
            out = list(self._fileio_unsolicited)
            self._fileio_unsolicited.clear()
            return out

    def take_fileio_logged_headers(self) -> List[bytes]:
        with self._fileio_lock:
            out = list(self._fileio_logged_headers)
            self._fileio_logged_headers.clear()
            return out

    def _arm_fileio_fetch(self, stream_id: int) -> None:
        with self._fileio_lock:
            self._fileio_active_stream = stream_id
            self._fileio_parser.reset()
            self._fileio_logged_headers.clear()
        while True:
            try:
                self._fileio_chunks.get_nowait()
            except queue.Empty:
                break

    def _disarm_fileio_fetch(self) -> None:
        with self._fileio_lock:
            self._fileio_active_stream = None

    def _reclaim_late_opens(self, link, timeout_s: float) -> None:
        """If openFileStatus arrives after we timed out, close those streams (FD leak)."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            msg = link.wait_for_control(
                "openFileStatus", timeout_s=min(0.2, max(0.0, deadline - time.monotonic()))
            )
            if msg is None:
                continue
            try:
                opened = parse_open_file_status(msg)
            except Exception:
                continue
            if not opened["ok"]:
                continue
            sid = opened["stream_identifier"]
            print(f"[USB] Reclaiming late openFileStatus stream={sid}")
            try:
                link.send_close_file(sid)
                link.wait_for_control("fileClosed", timeout_s=1.0)
            except Exception:
                pass

    def _wait_fileio_chunk(self, timeout_s: float) -> Optional[FileIOPacket]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                return self._fileio_chunks.get(timeout=min(0.1, remaining))
            except queue.Empty:
                continue
        return None

    def _link_mgr(self):
        transport = self.transport
        if transport is None:
            return None
        return getattr(transport, "link_mgr", None)

    def fetch_file(self, name: str, timeout_s: float = 30.0) -> bytes:
        """
        Read-only FileIO fetch: openFile -> readFile loop until zero-length chunk -> bytes.
        Cleans up with closeFile on error/timeout. Refuses concurrent fetches.
        """
        if not self.device_connected or self.transport is None:
            raise FileIOFetchError("device not connected")
        link = self._link_mgr()
        if link is None:
            raise FileIOFetchError("iAP2 link manager not available")
        if not link.link_ready.is_set():
            raise FileIOFetchError("iAP2 link not ready")

        if not self._fetch_lock.acquire(blocking=False):
            raise FileIOFetchError("another FileIO fetch is already in progress")

        stream_id: Optional[int] = None
        deadline = time.monotonic() + timeout_s
        parts: List[bytes] = []
        self._fileio_fetch_active.set()
        # Refresh frame watchdog grace so a slow open does not look like a stall.
        self._last_frame_time = time.time()

        def remaining() -> float:
            return max(0.0, deadline - time.monotonic())

        try:
            # Prefer waiting for sledInformation (signals FileIO is ready). If a prior
            # session left the channel up, openFile can still succeed without it.
            if link.sled_info is None:
                print("[USB] Waiting for sledInformation...")
                info = link.wait_for_sled_information(
                    timeout_s=min(8.0, max(1.0, remaining()))
                )
                if info is None:
                    print(
                        "[USB] No sledInformation yet — trying openFile anyway "
                        "(FileIO may still be ready from a prior session)"
                    )
                else:
                    print(f"[USB] sledInformation OK (mode={info.get('operatingMode')})")
                    time.sleep(0.3)

            # Short per-attempt open wait: missing files used to burn the full timeout,
            # and a coalesced openFileStatus drop looks identical to a hang.
            open_msg = None
            open_attempts = 2
            open_wait_s = 5.0
            for attempt in range(open_attempts):
                if remaining() <= 0:
                    break
                link.drain_control_messages()
                link.send_open_file(name, mode="r")
                open_msg = link.wait_for_control(
                    "openFileStatus", timeout_s=min(open_wait_s, remaining())
                )
                if open_msg is not None:
                    break
                if attempt + 1 < open_attempts:
                    print(f"[USB] openFileStatus timeout for {name!r}; retrying open...")

            if open_msg is None:
                self._reclaim_late_opens(link, min(1.5, remaining()))
                raise FileIOFetchError(f"timed out waiting for openFileStatus ({name})")
            opened = parse_open_file_status(open_msg)
            if not opened["ok"]:
                raise FileIOFetchError(f"{name}: {opened['error']}")
            stream_id = opened["stream_identifier"]

            self._arm_fileio_fetch(stream_id)
            while True:
                if remaining() <= 0:
                    raise FileIOFetchError(f"timed out reading {name}")
                link.send_read_file(stream_id)
                pkt = self._wait_fileio_chunk(timeout_s=min(5.0, remaining()))
                if pkt is None:
                    raise FileIOFetchError(f"timed out waiting for FileIO chunk ({name})")
                if pkt.payload_length == 0:
                    break
                parts.append(pkt.payload)

            return b"".join(parts)
        except Exception:
            if stream_id is not None:
                try:
                    link.send_close_file(stream_id)
                    link.wait_for_control("fileClosed", timeout_s=min(2.0, max(0.5, remaining())))
                except Exception:
                    pass
            raise
        finally:
            self._disarm_fileio_fetch()
            self._fileio_fetch_active.clear()
            self._fetch_lock.release()

    def _watchdog_loop(self, run_id: int) -> None:
        # Runs alongside the USB streaming threads.
        while self._active(run_id):
            time.sleep(1.0)
            if self._fileio_fetch_active.is_set():
                # Never reconnect under an active FileIO fetch (would orphan the wait).
                self._last_frame_time = time.time()
                continue
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
                sequence=frame_idx,
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
