import subprocess
import time

script = "si SWD\nspeed 4000\nconnect\nmem32 0x20000000, 4\nq\n"
with open("scratch_test_usb.jlink", "w") as f:
    f.write(script)

start = time.time()
try:
    p = subprocess.run([
        r"C:\Program Files\SEGGER\JLink\JLink.exe",
        "-USB", "69404531",
        "-Device", "LCM32F037",
        "-CommanderScript", "scratch_test_usb.jlink",
        "-ExitOnError", "1",
        "-NoGui", "1"
    ], capture_output=True, text=True, timeout=15)
    print(f"Elapsed: {time.time() - start:.2f}s")
    print("STDOUT:", p.stdout)
    print("STDERR:", p.stderr)
except subprocess.TimeoutExpired:
    print(f"TIMEOUT after {time.time() - start:.2f}s")
