"""Windows USB backend: preserve PySerial read semantics and driver errors."""
from collections import Counter
import ctypes
from serial.serialutil import PortNotOpenError, SerialException


def read_windows(port, size, api):
    """Keep bytes authoritative; report the flags that PySerial 3.5 normally clears."""
    if not port.is_open:
        raise PortNotOpenError()
    if size <= 0:
        return b""
    api.ResetEvent(port._overlapped_read.hEvent)
    flags, comstat = api.DWORD(), api.COMSTAT()
    if not api.ClearCommError(port._port_handle, ctypes.byref(flags), ctypes.byref(comstat)):
        raise SerialException(f"ClearCommError failed ({ctypes.WinError()!r})")
    counts = port.driver_error_counts
    counts["checks"] += 1
    if flags.value:
        counts["error_checks"] += 1
        counts["flags_or"] |= flags.value
        for name in ("CE_RXOVER", "CE_OVERRUN", "CE_RXPARITY", "CE_FRAME", "CE_BREAK"):
            if flags.value & getattr(api, name):
                counts[name] += 1
    size = min(comstat.cbInQue, size) if port.timeout == 0 else size
    if size <= 0:
        return b""
    buf, received = ctypes.create_string_buffer(size), api.DWORD()
    ok = api.ReadFile(port._port_handle, buf, size, ctypes.byref(received), ctypes.byref(port._overlapped_read))
    if not ok and api.GetLastError() not in (api.ERROR_SUCCESS, api.ERROR_IO_PENDING):
        raise SerialException(f"ReadFile failed ({ctypes.WinError()!r})")
    ok = api.GetOverlappedResult(port._port_handle, ctypes.byref(port._overlapped_read), ctypes.byref(received), True)
    if not ok and api.GetLastError() != api.ERROR_OPERATION_ABORTED:
        raise SerialException(f"GetOverlappedResult failed ({ctypes.WinError()!r})")
    return bytes(buf.raw[:received.value])


def create_serial():
    """Construct unopened: DTR/RTS, timeouts and port are still set by ReadSerial."""
    from serial import serialwin32, win32

    class MonitoredSerial(serialwin32.Serial):
        def __init__(self):
            self.driver_error_counts = Counter()
            super().__init__()

        def read(self, size=1):
            return read_windows(self, size, win32)

    return MonitoredSerial()
