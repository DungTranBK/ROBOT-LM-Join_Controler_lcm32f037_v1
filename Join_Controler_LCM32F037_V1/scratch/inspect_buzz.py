import subprocess
import time
import struct
import re

JLINK_EXE = r"D:\Program Files\Kei_v5\ARM\Segger\JLink.exe"

def run_jlink(commands):
    script_path = "scratch/temp_buzz.jlink"
    with open(script_path, "w") as f:
        f.write("device LCM32F037\n")
        f.write("si SWD\n")
        f.write("speed 4000\n")
        f.write("connect\n")
        for cmd in commands:
            f.write(cmd + "\n")
        f.write("q\n")
    res = subprocess.run([JLINK_EXE, "-CommanderScript", script_path], capture_output=True, text=True)
    return res.stdout

def parse_mem32(output):
    mem = {}
    for line in output.splitlines():
        m = re.match(r"^([0-9A-Fa-f]{8})\s*=\s*(.*)", line.strip())
        if m:
            base_addr = int(m.group(1), 16)
            vals = m.group(2).split()
            for i, v in enumerate(vals):
                if len(v) == 8:
                    mem[base_addr + i * 4] = int(v, 16)
    return mem

print("Sampling FOC variables while holding...")
for i in range(10):
    out = run_jlink([
        "mem32 0x20000070, 4", # f_duty[0..2] (0x70..0x78), f_elec (0x7c)
        "mem32 0x20000080, 4", # f_id (0x80), f_integ_d (0x84), f_integ_q (0x88), f_integ_v (0x8c)
        "mem32 0x20000090, 4", # f_iq (0x90), f_iq_ref (0x94), f_pos_prev, f_pos_target (0x98)
        "mem32 0x200000a4, 2", # f_vd (0xa4), f_vq (0xa8)
        "mem32 0x20000290, 4", # pos10 (0x292), raw_angle (0x298)
        "mem32 0x200004a8, 1", # s_speed
    ])
    mem = parse_mem32(out)
    duty0 = mem.get(0x20000070, 0)
    duty1 = mem.get(0x20000074, 0)
    duty2 = mem.get(0x20000078, 0)
    elec = mem.get(0x2000007c, 0)
    id_val = struct.unpack("<i", struct.pack("<I", mem.get(0x20000080, 0)))[0]
    integ_d = struct.unpack("<i", struct.pack("<I", mem.get(0x20000084, 0)))[0]
    integ_q = struct.unpack("<i", struct.pack("<I", mem.get(0x20000088, 0)))[0]
    iq_val = struct.unpack("<i", struct.pack("<I", mem.get(0x20000090, 0)))[0]
    iq_ref = struct.unpack("<h", struct.pack("<H", mem.get(0x20000094, 0) & 0xFFFF))[0]
    vd = struct.unpack("<i", struct.pack("<I", mem.get(0x200000a4, 0)))[0]
    vq = struct.unpack("<i", struct.pack("<I", mem.get(0x200000a8, 0)))[0]
    raw = mem.get(0x20000298, 0) & 0x3FFF
    pos10 = (mem.get(0x20000290, 0) >> 16) & 0xFFFF
    spd = struct.unpack("<i", struct.pack("<I", mem.get(0x200004a8, 0)))[0]
    print(f"[{i}] Pos10={pos10}, Raw={raw}, Spd={spd:2d}, IqRef={iq_ref:2d}, Id={id_val:3d}, Iq={iq_val:3d}, Vd={vd:4d}, Vq={vq:4d}, Duty=({duty0},{duty1},{duty2})")
    time.sleep(0.05)
