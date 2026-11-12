"""
PyUSB / libusb backend for FLIR ONE (Linux fallback and Windows alternative).
"""

import time
from typing import Optional

from iap2 import (
    EP_FILEIO_IN,
    EP_FRAME_IN,
    EP_IAP2_IN,
    EP_IAP2_OUT,
    IAP2LinkManager,
)

FLIR_VID = 0x09CB
FLIR_PID = 0x1996


def get_usb_backend():
    try:
        import libusb_package
        return libusb_package.get_libusb1_backend()
    except Exception:
        return None


class LibUsbTransport:
    def __init__(self, log_fn=None, full_startup: bool = True):
        self.log_fn = log_fn or print
        self.full_startup = full_startup
        self.dev = None
        self.link_mgr: Optional[IAP2LinkManager] = None

    def is_available(self) -> bool:
        try:
            import usb.core  # noqa: F401
            return True
        except ImportError:
            return False

    def probe_interfaces(self) -> dict:
        import usb.core
        backend = get_usb_backend()
        dev = usb.core.find(idVendor=FLIR_VID, idProduct=FLIR_PID, backend=backend)
        return {"device": dev is not None}

    def connect(self, timeout_sec: float = 15.0) -> bool:
        import usb.core
        import usb.util

        backend = get_usb_backend()
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            try:
                self.dev = usb.core.find(
                    idVendor=FLIR_VID, idProduct=FLIR_PID, backend=backend
                )
            except Exception:
                self.dev = None
            if self.dev is not None:
                break
            time.sleep(0.2)

        if self.dev is None:
            return False

        self.log_fn(
            f"[libusb] Found FLIR One (bus={self.dev.bus}, addr={self.dev.address})"
        )

        for intf in (0, 1, 2):
            try:
                usb.util.claim_interface(self.dev, intf)
            except Exception as e:
                self.log_fn(f"[libusb] claim_interface({intf}): {e}")

        for ep in (EP_IAP2_IN, EP_IAP2_OUT, EP_FILEIO_IN, EP_FRAME_IN):
            try:
                self.dev.clear_halt(ep)
            except Exception:
                pass

        def write_fn(data: bytes) -> None:
            self.dev.write(EP_IAP2_OUT, data, timeout=500)

        def read_fn(pipe: int, size: int) -> bytes:
            try:
                return bytes(self.dev.read(pipe, size, timeout=100))
            except Exception:
                return b""

        self.link_mgr = IAP2LinkManager(
            write_fn=write_fn,
            read_fn=read_fn,
            log_fn=self.log_fn,
            full_startup=self.full_startup,
        )
        self.link_mgr.start()

        if not self.link_mgr.link_ready.wait(timeout=10.0):
            self.log_fn("[libusb] iAP2 link handshake timed out")
            return False

        try:
            self.dev.set_interface_altsetting(interface=1, alternate_setting=1)
            self.log_fn("[libusb] Interface 1 AltSetting 1 active")
        except Exception as e:
            self.log_fn(f"[libusb] SET_INTERFACE(1,1): {e}")

        time.sleep(0.1)

        try:
            self.dev.set_interface_altsetting(interface=2, alternate_setting=1)
            self.log_fn("[libusb] Interface 2 AltSetting 1 active")
        except Exception as e:
            self.log_fn(f"[libusb] SET_INTERFACE(2,1): {e}")

        return True

    def read_fileio(self, size: int = 4096, timeout_ms: int = 50) -> bytes:
        if self.dev is None:
            return b""
        try:
            return bytes(self.dev.read(EP_FILEIO_IN, size, timeout=timeout_ms))
        except Exception:
            return b""

    def read_frame(self, size: int = 65536, timeout_ms: int = 500) -> bytes:
        if self.dev is None:
            return b""
        try:
            return bytes(self.dev.read(EP_FRAME_IN, size, timeout=timeout_ms))
        except Exception:
            return b""

    def close(self) -> None:
        if self.link_mgr:
            self.link_mgr.running = False
            # Ensure background iAP2 thread stops before disposing the device.
            try:
                self.link_mgr.join(timeout=1.0)
            except Exception:
                pass
            self.link_mgr = None
        if self.dev:
            try:
                import usb.util
                for intf in (0, 1, 2):
                    try:
                        usb.util.release_interface(self.dev, intf)
                    except Exception:
                        pass
                usb.util.dispose_resources(self.dev)
            except Exception:
                pass
            self.dev = None
