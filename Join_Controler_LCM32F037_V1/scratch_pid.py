import ctypes
import sys

target_pid = int(sys.argv[1]) if len(sys.argv) > 1 else 0

def enum_cb(hwnd, lparam):
    pid = ctypes.c_ulong()
    ctypes.windll.user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
    if pid.value == target_pid or target_pid == 0:
        vis = ctypes.windll.user32.IsWindowVisible(hwnd)
        length = ctypes.windll.user32.GetWindowTextLengthW(hwnd)
        buff = ctypes.create_unicode_buffer(length + 1)
        ctypes.windll.user32.GetWindowTextW(hwnd, buff, length + 1)
        if buff.value or vis:
            print(f'PID {pid.value} HWND {hwnd} [Visible={vis}]: "{buff.value}"')
    return True

cb = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)(enum_cb)
ctypes.windll.user32.EnumWindows(cb, 0)
