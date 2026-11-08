import ctypes
import subprocess
import time

def print_all_windows():
    def enum_cb(hwnd, lparam):
        length = ctypes.windll.user32.GetWindowTextLengthW(hwnd)
        if length > 0:
            buff = ctypes.create_unicode_buffer(length + 1)
            ctypes.windll.user32.GetWindowTextW(hwnd, buff, length + 1)
            pid = ctypes.c_ulong()
            ctypes.windll.user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
            title = buff.value
            if any(k in title.lower() for k in ['j-link', 'segger', 'clone', 'defective', 'warning', 'error', 'keil', 'uv4', 'cortex']):
                print(f"PID {pid.value} HWND {hwnd}: {title}")
        return True
    cb = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)(enum_cb)
    ctypes.windll.user32.EnumWindows(cb, 0)

print_all_windows()
