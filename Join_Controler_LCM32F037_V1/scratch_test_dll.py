import ctypes
import os

dll_path = r'C:\Program Files\SEGGER\JLink\JLink_x64.dll'
if not os.path.exists(dll_path):
    print("DLL not found:", dll_path)
    exit(1)

jlink = ctypes.cdll.LoadLibrary(dll_path)
print("JLinkARM.dll loaded successfully!")

# Set error and out handlers to stdout
CMPFUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p)
def err_handler(msg):
    print("JLINK ERROR:", msg.decode('utf-8', errors='ignore'))
def out_handler(msg):
    print("JLINK OUT:", msg.decode('utf-8', errors='ignore'))

c_err = CMPFUNC(err_handler)
c_out = CMPFUNC(out_handler)

try:
    jlink.JLINKARM_SetErrorOutHandler(c_err)
    jlink.JLINKARM_SetWarnOutHandler(c_out)
except Exception as e:
    print("Handler set exc:", e)

print("Calling JLINKARM_Open()...")
res = jlink.JLINKARM_Open()
print("JLINKARM_Open() returned:", res)
if res == 0:
    sn = jlink.JLINKARM_GetSN()
    print(f"Connected to J-Link SN: {sn}")
    jlink.JLINKARM_Close()
else:
    print("Failed to open J-Link!")
