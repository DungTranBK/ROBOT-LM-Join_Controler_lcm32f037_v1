import subprocess
import time
import struct
import re

JLINK_EXE = r"D:\Program Files\Kei_v5\ARM\Segger\JLink.exe"

def run_jlink(commands):
    script_path = "scratch/temp_align.jlink"
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

# Read current pos10
out = run_jlink(["mem32 0x20000290, 4"])
mem = parse_mem32(out)
cur_pos = (mem.get(0x20000290, 0) >> 16) & 0xFFFF
raw = mem.get(0x20000298, 0) & 0x3FFF

print(f"Current motor physical position: Pos10={cur_pos}, Raw14={raw}")
print(f"Aligning goal_position and s_goal_position_cmd to {cur_pos}...")

run_jlink([
    "w1 0x200000c4, 0x01",                 # torque_enable = 1
    f"w2 0x200000ca, 0x{cur_pos:04X}",     # goal_position = cur_pos
    f"w2 0x20000172, 0x{cur_pos:04X}",     # s_goal_position_cmd = cur_pos
])

time.sleep(0.1)

print("\nSampling state after aligning goal to current position:")
for i in range(10):
    out = run_jlink([
        "mem32 0x20000170, 2", # goal (0x172)
        "mem32 0x20000290, 4", # pos10 (0x292), hold (0x294), raw (0x298)
        "mem32 0x20000094, 1", # iq_ref (0x94)
        "mem32 0x200004a8, 1", # speed (0x4a8)
        "mem32 0x200000a4, 2", # vd, vq
    ])
    mem = parse_mem32(out)
    goal = (mem.get(0x20000170, 0) >> 16) & 0xFFFF
    pos10 = (mem.get(0x20000290, 0) >> 16) & 0xFFFF
    hold = mem.get(0x20000294, 0) & 0xFF
    raw = mem.get(0x20000298, 0) & 0x3FFF
    iq_ref = struct.unpack("<h", struct.pack("<H", mem.get(0x20000094, 0) & 0xFFFF))[0]
    spd = struct.unpack("<i", struct.pack("<I", mem.get(0x200004a8, 0)))[0]
    vd = struct.unpack("<i", struct.pack("<I", mem.get(0x200000a4, 0)))[0]
    vq = struct.unpack("<i", struct.pack("<I", mem.get(0x200000a8, 0)))[0]
    print(f"[{i}] Goal={goal}, Pos10={pos10}, Raw={raw}, Hold={hold}, Spd={spd:2d}, IqRef={iq_ref:2d}, Vd={vd}, Vq={vq}")
    time.sleep(0.05)
