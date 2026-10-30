import subprocess, re
JLINK_EXE = r"D:\Program Files\Kei_v5\ARM\Segger\JLink.exe"
script = "device LCM32F037\nsi SWD\nspeed 4000\nconnect\nmem32 0x20000098, 1\nmem32 0x20000170, 2\nmem32 0x2000017c, 1\nmem32 0x20000294, 1\nq\n"
with open("scratch/check_target.jlink", "w") as f: f.write(script)
res = subprocess.run([JLINK_EXE, "-CommanderScript", "scratch/check_target.jlink"], capture_output=True, text=True)
print(res.stdout)
