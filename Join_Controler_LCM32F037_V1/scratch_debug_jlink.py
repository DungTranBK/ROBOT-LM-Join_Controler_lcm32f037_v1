import subprocess
import time
# import ctypes
import ctypes

def get_windows():
    titles = []
    def enum_cb(hwnd, lparam):
        length = ctypes.windll.user32.GetWindowTextLengthW(hwnd)
        if length > 0:
            buff = ctypes.create_unicode_buffer(length + 1)
            ctypes.windll.user32.GetWindowTextW(hwnd, buff, length + 1)
            pid = ctypes.c_ulong()
            ctypes.windll.user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
            titles.append((pid.value, buff.value))
        return True
    
    cb = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)(enum_cb)
    ctypes.windll.user32.EnumWindows(cb, 0)
    return titles

proc = subprocess.Popen([r'D:\Program Files\Kei_v5\ARM\Segger\JLink.exe', '-CommanderScript', 'scratch_test.jlink'])
time.sleep(1.0)
wins = get_windows()
for pid, title in wins:
    if pid == proc.pid or any(k in title.lower() for k in ['j-link', 'segger', 'uv4', 'keil', 'error', 'warning']):
        print(f"PID {pid}: {title}")
proc.kill()
