"""
Native Windows WinUSB backend for FLIR ONE composite device.
Uses overlapped I/O (required for WinUsb_Initialize on Zadig-bound interfaces).
"""

import ctypes
import logging
import sys
import time
from ctypes import wintypes
from typing import Optional, Callable

from iap2 import (
    EP_FILEIO_IN,
    EP_FRAME_IN,
    EP_IAP2_IN,
    EP_IAP2_OUT,
    IAP2LinkManager,
)

logger = logging.getLogger("rosedud.winusb")

FLIR_VID = 0x09CB
FLIR_PID = 0x1996

GUID_INTF0 = "{6CE7CB69-66E4-4333-9B2C-55CAF3B6DD9C}"
GUID_INTF1 = "{CC2D2161-A10F-43DB-8E4B-1F210840EF2B}"
GUID_INTF2 = "{2B51B20E-23A3-43AD-B1B6-69AD1C1F6A44}"

DIGCF_PRESENT = 0x00000002
DIGCF_DEVICEINTERFACE = 0x00000010
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value
GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 0x00000001
FILE_SHARE_WRITE = 0x00000002
OPEN_EXISTING = 3
FILE_ATTRIBUTE_NORMAL = 0x00000080
FILE_FLAG_OVERLAPPED = 0x40000000
PIPE_TRANSFER_TIMEOUT = 3


class GUID(ctypes.Structure):
    _fields_ = [
        ("Data1", wintypes.DWORD),
        ("Data2", wintypes.WORD),
        ("Data3", wintypes.WORD),
        ("Data4", wintypes.BYTE * 8),
    ]


class SP_DEVICE_INTERFACE_DATA(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("InterfaceClassGuid", GUID),
        ("Flags", wintypes.DWORD),
        ("Reserved", ctypes.c_void_p),
    ]


class OVERLAPPED(ctypes.Structure):
    _fields_ = [
        ("Internal", ctypes.c_void_p),
        ("InternalHigh", ctypes.c_void_p),
        ("Offset", wintypes.DWORD),
        ("OffsetHigh", wintypes.DWORD),
        ("hEvent", ctypes.c_void_p),
    ]


class WINUSB_SETUP_PACKET(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("RequestType", ctypes.c_ubyte),
        ("Request", ctypes.c_ubyte),
        ("Value", wintypes.WORD),
        ("Index", wintypes.WORD),
        ("Length", wintypes.WORD),
    ]


def _parse_guid(guid_str: str) -> GUID:
    g = GUID()
    ctypes.windll.ole32.CLSIDFromString(guid_str, ctypes.byref(g))
    return g


def get_interface_path(guid_str: str) -> Optional[str]:
    setupapi = ctypes.windll.setupapi
    setupapi.SetupDiGetClassDevsW.restype = ctypes.c_void_p
    setupapi.SetupDiGetClassDevsW.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD,
    ]
    setupapi.SetupDiEnumDeviceInterfaces.restype = wintypes.BOOL
    setupapi.SetupDiEnumDeviceInterfaces.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD, ctypes.c_void_p,
    ]
    setupapi.SetupDiGetDeviceInterfaceDetailW.restype = wintypes.BOOL
    setupapi.SetupDiGetDeviceInterfaceDetailW.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD,
        ctypes.c_void_p, ctypes.c_void_p,
    ]
    setupapi.SetupDiDestroyDeviceInfoList.restype = wintypes.BOOL
    setupapi.SetupDiDestroyDeviceInfoList.argtypes = [ctypes.c_void_p]

    guid = _parse_guid(guid_str)
    hdevinfo = setupapi.SetupDiGetClassDevsW(
        ctypes.byref(guid), None, None, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE
    )
    if hdevinfo is None or hdevinfo == ctypes.c_void_p(-1).value:
        return None

    intf_data = SP_DEVICE_INTERFACE_DATA()
    intf_data.cbSize = ctypes.sizeof(SP_DEVICE_INTERFACE_DATA)

    path = None
    if setupapi.SetupDiEnumDeviceInterfaces(
        hdevinfo, None, ctypes.byref(guid), 0, ctypes.byref(intf_data)
    ):
        req_size = wintypes.DWORD(0)
        setupapi.SetupDiGetDeviceInterfaceDetailW(
            hdevinfo, ctypes.byref(intf_data), None, 0, ctypes.byref(req_size), None
        )
        if req_size.value > 0:
            buf = ctypes.create_string_buffer(req_size.value)
            # sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W): DWORD + WCHAR[1], padded on x64.
            cb_size = 8 if ctypes.sizeof(ctypes.c_void_p) == 8 else 6
            ctypes.memmove(buf, ctypes.byref(wintypes.DWORD(cb_size)), 4)
            if setupapi.SetupDiGetDeviceInterfaceDetailW(
                hdevinfo, ctypes.byref(intf_data), buf, req_size.value, None, None
            ):
                path = ctypes.wstring_at(ctypes.addressof(buf) + 4)

    setupapi.SetupDiDestroyDeviceInfoList(hdevinfo)
    return path


class WinUsbInterface:
    def __init__(self, guid_str: str, name: str = "Interface", debug: bool = False):
        self.guid_str = guid_str
        self.name = name
        self.debug = debug
        self.path: Optional[str] = None
        self.file_handle = None
        self.winusb_handle = ctypes.c_void_p()
        self._event = None
        # Avoid flooding stdout on normal "no data yet" reads.
        self._timeout_log_remaining = 20
        self._fail_log_remaining = 10

    @property
    def is_open(self) -> bool:
        return bool(self.winusb_handle)

    def open(self) -> bool:
        self.path = get_interface_path(self.guid_str)
        if not self.path:
            return False

        kernel32 = ctypes.windll.kernel32
        kernel32.CreateFileW.restype = ctypes.c_void_p
        kernel32.CreateFileW.argtypes = [
            wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
            ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p,
        ]
        kernel32.CreateEventW.restype = ctypes.c_void_p
        kernel32.CreateEventW.argtypes = [
            ctypes.c_void_p, wintypes.BOOL, wintypes.BOOL, wintypes.LPCWSTR,
        ]

        self.file_handle = kernel32.CreateFileW(
            self.path,
            GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            None,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
            None,
        )
        if self.file_handle == INVALID_HANDLE_VALUE:
            return False

        winusb = ctypes.windll.winusb
        winusb.WinUsb_Initialize.restype = wintypes.BOOL
        winusb.WinUsb_Initialize.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        if not winusb.WinUsb_Initialize(self.file_handle, ctypes.byref(self.winusb_handle)):
            kernel32.CloseHandle(self.file_handle)
            self.file_handle = None
            return False

        self._event = kernel32.CreateEventW(None, True, False, None)
        return self._event is not None

    def set_alt_setting(self, alt_setting: int) -> bool:
        winusb = ctypes.windll.winusb
        winusb.WinUsb_SetCurrentAlternateSetting.restype = wintypes.BOOL
        winusb.WinUsb_SetCurrentAlternateSetting.argtypes = [ctypes.c_void_p, wintypes.BYTE]
        return bool(
            winusb.WinUsb_SetCurrentAlternateSetting(
                self.winusb_handle, wintypes.BYTE(alt_setting)
            )
        )

    def control_transfer(
        self, req_type: int, req: int, val: int, idx: int, data: bytes = b""
    ) -> bool:
        winusb = ctypes.windll.winusb
        winusb.WinUsb_ControlTransfer.restype = wintypes.BOOL
        winusb.WinUsb_ControlTransfer.argtypes = [
            ctypes.c_void_p,
            WINUSB_SETUP_PACKET,
            ctypes.c_char_p,
            wintypes.ULONG,
            ctypes.POINTER(wintypes.ULONG),
            ctypes.c_void_p,
        ]
        pkt = WINUSB_SETUP_PACKET(req_type, req, val, idx, len(data))
        buf = ctypes.create_string_buffer(data) if data else None
        transferred = wintypes.ULONG(0)
        return bool(
            winusb.WinUsb_ControlTransfer(
                self.winusb_handle,
                pkt,
                buf,
                len(data) if data else 0,
                ctypes.byref(transferred),
                None,
            )
        )

    def _do_transfer(
        self, pipe_fn_name: str, pipe_id: int, data: Optional[bytes], size: int, timeout_ms: int
    ) -> tuple[int, bytes]:
        winusb = ctypes.windll.winusb
        kernel32 = ctypes.windll.kernel32

        timeout_dw = wintypes.DWORD(timeout_ms)
        winusb.WinUsb_SetPipePolicy(
            self.winusb_handle, wintypes.BYTE(pipe_id),
            PIPE_TRANSFER_TIMEOUT, ctypes.sizeof(timeout_dw), ctypes.byref(timeout_dw),
        )

        ov = OVERLAPPED()
        ov.hEvent = self._event
        kernel32.ResetEvent(self._event)

        transferred = wintypes.DWORD(0)
        pipe_fn = getattr(winusb, pipe_fn_name)

        if data is not None:
            buf = ctypes.create_string_buffer(data)
            length = len(data)
        else:
            buf = ctypes.create_string_buffer(size)
            length = size

        res = pipe_fn(
            self.winusb_handle, wintypes.BYTE(pipe_id),
            buf, length, ctypes.byref(transferred), ctypes.byref(ov),
        )

        if not res:
            err = kernel32.GetLastError()
            if err == 997:  # ERROR_IO_PENDING
                wait = kernel32.WaitForSingleObject(self._event, wintypes.DWORD(timeout_ms))
                if wait == 0:
                    winusb.WinUsb_GetOverlappedResult(
                        self.winusb_handle, ctypes.byref(ov),
                        ctypes.byref(transferred), False,
                    )
                else:
                    winusb.WinUsb_AbortPipe(self.winusb_handle, wintypes.BYTE(pipe_id))
                    # Wait for the aborted request to complete so the kernel is done
                    # with `buf` and `ov` before they are freed.
                    winusb.WinUsb_GetOverlappedResult(
                        self.winusb_handle, ctypes.byref(ov),
                        ctypes.byref(transferred), True,
                    )
                    # Helpful when the camera isn't producing frames.
                    if self._timeout_log_remaining > 0:
                        self._timeout_log_remaining -= 1
                        if self.debug:
                            logger.debug(f"[WinUSB] {pipe_fn_name} pipe=0x{pipe_id:02X} wait-timeout")
                    return -1, b""
            else:
                if self._fail_log_remaining > 0:
                    self._fail_log_remaining -= 1
                    if self.debug:
                        logger.debug(f"[WinUSB] {pipe_fn_name} pipe=0x{pipe_id:02X} transfer-failed err={err}")
                return -1, b""

        n = transferred.value
        return n, bytes(buf.raw[:n]) if n > 0 else b""

    def write_pipe(self, pipe_id: int, data: bytes, timeout_ms: int = 1000) -> int:
        n, _ = self._do_transfer("WinUsb_WritePipe", pipe_id, data, 0, timeout_ms)
        return n if n >= 0 else -1

    def read_pipe(self, pipe_id: int, size: int, timeout_ms: int = 1000) -> bytes:
        if size <= 0:
            return b""
        n, data = self._do_transfer("WinUsb_ReadPipe", pipe_id, None, size, timeout_ms)
        return data if n > 0 else b""

    def reset_pipe(self, pipe_id: int) -> None:
        try:
            ctypes.windll.winusb.WinUsb_ResetPipe(
                self.winusb_handle, wintypes.BYTE(pipe_id)
            )
        except Exception:
            pass

    def close(self) -> None:
        if self._event:
            ctypes.windll.kernel32.CloseHandle(self._event)
            self._event = None
        if self.winusb_handle:
            ctypes.windll.winusb.WinUsb_Free(self.winusb_handle)
            self.winusb_handle = ctypes.c_void_p()
        if self.file_handle and self.file_handle != INVALID_HANDLE_VALUE:
            ctypes.windll.kernel32.CloseHandle(self.file_handle)
            self.file_handle = None


class WinUsbTransport:
    """Manages all three WinUSB interface handles and iAP2 link thread."""

    def __init__(self, log_fn: Optional[Callable] = None, full_startup: bool = False, debug: bool = False):
        self.log_fn = log_fn
        self.full_startup = full_startup
        self.debug = debug
        self.intf0 = WinUsbInterface(GUID_INTF0, "iAP2", debug=self.debug)
        self.intf1 = WinUsbInterface(GUID_INTF1, "FileIO", debug=self.debug)
        self.intf2 = WinUsbInterface(GUID_INTF2, "Frame", debug=self.debug)
        self.link_mgr: Optional[IAP2LinkManager] = None

    def _log(self, level: int, msg: str) -> None:
        if self.log_fn is not None:
            self.log_fn(msg)
        else:
            logger.log(level, msg)

    def is_available(self) -> bool:
        return sys.platform == "win32"

    def probe_interfaces(self) -> dict:
        result = {}
        for label, guid in [
            ("iap", GUID_INTF0),
            ("fileio", GUID_INTF1),
            ("frame", GUID_INTF2),
        ]:
            result[label] = get_interface_path(guid) is not None
        return result

    def _set_interface(self, index: int, alt: int, delay_sec: float = 0.2) -> bool:
        """USB SET_INTERFACE (bmRequestType=0x01, bRequest=0x0B)."""
        ok = self.intf0.control_transfer(0x01, 0x0B, alt, index)
        self._log(logging.DEBUG, f"[WinUSB] SET_INTERFACE({index},{alt})={ok}")
        if ok and delay_sec > 0:
            time.sleep(delay_sec)
        return ok

    def _activate_interface(self, index: int, delay_sec: float = 0.2) -> bool:
        return self._set_interface(index, 1, delay_sec=delay_sec)

    def _wait_for_sled_information(self, timeout_s: float) -> bool:
        if self.link_mgr is None:
            return False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.link_mgr.sled_info is not None:
                return True
            time.sleep(0.05)
        return self.link_mgr.sled_info is not None

    def connect(self, timeout_sec: float = 15.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            if self.intf0.open():
                self._log(logging.DEBUG, f"[WinUSB] Interface 0 open: {self.intf0.path}")
                break
            time.sleep(0.1)
        else:
            return False
            
        self.intf0.reset_pipe(EP_IAP2_IN)
        self.intf0.reset_pipe(EP_IAP2_OUT)

        def write_fn(data: bytes) -> None:
            self.intf0.write_pipe(EP_IAP2_OUT, data, timeout_ms=500)

        def read_fn(pipe: int, size: int) -> bytes:
            return self.intf0.read_pipe(pipe, size, timeout_ms=100)

        self.link_mgr = IAP2LinkManager(
            write_fn=write_fn,
            read_fn=read_fn,
            log_fn=self.log_fn,
            full_startup=self.full_startup,
        )
        self.link_mgr.start()

        if not self.link_mgr.link_ready.wait(timeout=20.0):
            self._log(logging.WARNING, "[WinUSB] iAP2 link handshake timed out")
            return False

        # Cycle bulk interfaces through alt 0 then 1 so a re-connect still gets
        # sledInformation (SET_INTERFACE(1,1) while already active is a no-op).
        self._set_interface(2, 0, delay_sec=0.1)
        self._set_interface(1, 0, delay_sec=0.2)

        if not self._activate_interface(1, delay_sec=0.5):
            self._log(logging.ERROR, "[WinUSB] Failed to activate Interface 1 (FileIO)")
            return False

        if not self.intf1.open():
            self._log(logging.ERROR, "[WinUSB] Failed to open Interface 1 (FileIO)")
            return False
        # Ensure WinUSB handle is bound to Active AltSetting 1.
        self.intf1.set_alt_setting(1)
        self.intf1.reset_pipe(EP_FILEIO_IN)
        self._log(logging.DEBUG, f"[WinUSB] Interface 1 active: {self.intf1.path}")

        if not self._activate_interface(2, delay_sec=0.2):
            self._log(logging.ERROR, "[WinUSB] Failed to activate Interface 2 (Frame)")
            return False

        if not self.intf2.open():
            self._log(logging.ERROR, "[WinUSB] Failed to open Interface 2 (Frame)")
            return False
        # Ensure WinUSB handle is bound to Active AltSetting 1.
        self.intf2.set_alt_setting(1)
        self.intf2.reset_pipe(EP_FRAME_IN)
        self._log(logging.DEBUG, f"[WinUSB] Interface 2 active: {self.intf2.path}")

        if not self._wait_for_sled_information(5.0):
            self._log(
                logging.WARNING,
                "[WinUSB] No sledInformation after UP; re-cycling FileIO interface"
            )
            self._set_interface(1, 0, delay_sec=0.3)
            self._set_interface(1, 1, delay_sec=0.5)
            if self.intf1.is_open:
                self.intf1.set_alt_setting(1)
                self.intf1.reset_pipe(EP_FILEIO_IN)
            if not self._wait_for_sled_information(5.0):
                self._log(
                    logging.WARNING,
                    "[WinUSB] WARNING: still no sledInformation — FileIO may fail"
                )
        else:
            mode = self.link_mgr.sled_info.get("operatingMode")
            self._log(logging.INFO, f"[WinUSB] sledInformation OK (mode={mode})")

        return True

    def read_fileio(self, size: int = 4096, timeout_ms: int = 500) -> bytes:
        if self.intf1.is_open:
            return self.intf1.read_pipe(EP_FILEIO_IN, size, timeout_ms=timeout_ms)
        return b""

    def read_frame(self, size: int = 65536, timeout_ms: int = 500) -> bytes:
        if self.intf2.is_open:
            return self.intf2.read_pipe(EP_FRAME_IN, size, timeout_ms=timeout_ms)
        return b""

    def close(self) -> None:
        if self.link_mgr:
            # Do not send 0xEA01 here — stopping the command session while USB stays
            # attached leaves the sled silent (no SYN) until a power cycle. Stale
            # EA recovery on the next connect sends EA01 then EA00 instead.
            self.link_mgr.running = False
            # Ensure background iAP2 thread stops before freeing WinUSB handles.
            try:
                self.link_mgr.join(timeout=1.0)
            except Exception:
                pass
            self.link_mgr = None
        # Idle bulk interfaces; command EA session remains for the next host.
        if self.intf0.is_open:
            try:
                self._set_interface(2, 0, delay_sec=0.05)
                self._set_interface(1, 0, delay_sec=0.1)
            except Exception:
                pass
        self.intf2.close()
        self.intf1.close()
        self.intf0.close()
