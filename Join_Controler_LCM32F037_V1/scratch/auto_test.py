import subprocess
import time
import struct
import re

JLINK_EXE = r"D:\Program Files\Kei_v5\ARM\Segger\JLink.exe"

def run_jlink_cmd(commands):
    script_path = "scratch/temp_cmd.jlink"
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
    # Parses '200000AC = 011C000C 0000FA01 ...' into dict of addr -> uint32 values
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

def sample_motor_status():
    cmds = [
        "mem32 0x20000090, 4", # f_iq_ref (0x94), f_pos_target (0x98)
        "mem32 0x200000ac, 12",# g_control_table
        "mem32 0x20000170, 4", # goal_cmd (0x172), speed_cmd (0x174), multiturn (0x17c)
        "mem32 0x20000290, 4", # pos10 (0x292), pos_hold (0x294), raw_angle (0x298)
        "mem32 0x200004a8, 1", # s_speed
    ]
    out = run_jlink_cmd(cmds)
    mem = parse_mem32(out)
    
    # Extract values
    # 0x20000094: f_iq_ref (int16), f_pos_prev (int16)
    w_90 = mem.get(0x20000094, 0)
    iq_ref = struct.unpack("<h", struct.pack("<H", w_90 & 0xFFFF))[0]
    
    # 0x20000098: f_pos_target (int32)
    pos_target = struct.unpack("<i", struct.pack("<I", mem.get(0x20000098, 0)))[0]
    
    # 0x20000170: [0x170]=0, [0x172]=goal_cmd, [0x174]=speed_cmd
    w_170 = mem.get(0x20000170, 0)
    goal_cmd = (w_170 >> 16) & 0xFFFF
    
    # 0x2000017c: s_multiturn (int32)
    multiturn = struct.unpack("<i", struct.pack("<I", mem.get(0x2000017c, 0)))[0]
    
    # 0x20000290: [0x290]=baud_valid, [0x292]=s_pos10
    w_290 = mem.get(0x20000290, 0)
    pos10 = (w_290 >> 16) & 0xFFFF
    
    # 0x20000294: [0x294]=pos_hold (uint8), [0x295]=prev_mode (uint8)
    w_294 = mem.get(0x20000294, 0)
    pos_hold = w_294 & 0xFF
    
    # 0x20000298: s_raw_angle (int32)
    raw_angle = mem.get(0x20000298, 0) & 0x3FFF
    
    # 0x200004a8: s_speed (int32)
    speed = struct.unpack("<i", struct.pack("<I", mem.get(0x200004a8, 0)))[0]
    
    # Control table
    # 0x200000c4: [0x200000c6]=torque_enable
    w_c4 = mem.get(0x200000c4, 0)
    torque_enable = (w_c4 >> 16) & 0xFF
    calib_done = (w_c4 >> 24) & 0xFF
    
    return {
        "torque_enable": torque_enable,
        "calib_done": calib_done,
        "pos_hold": pos_hold,
        "goal_cmd": goal_cmd,
        "pos10": pos10,
        "raw_angle": raw_angle,
        "multiturn": multiturn,
        "pos_target": pos_target,
        "iq_ref": iq_ref,
        "speed": speed,
    }

def main():
    print("=== MOTOR AUTOMATED STATUS & STABILITY CHECK ===")
    samples = []
    for i in range(5):
        st = sample_motor_status()
        samples.append(st)
        time.sleep(0.1)
    
    last = samples[-1]
    raw_vals = [s["raw_angle"] for s in samples]
    speed_vals = [s["speed"] for s in samples]
    
    raw_span = max(raw_vals) - min(raw_vals)
    speed_max = max(abs(s) for s in speed_vals)
    
    err_counts_10 = last["goal_cmd"] - last["pos10"]
    err_deg_10 = err_counts_10 * (360.0 / 1024.0)
    
    # Fine error in 14-bit (0..16383)
    target_14 = (last["goal_cmd"] << 4)
    raw_14 = last["raw_angle"]
    err_14 = target_14 - raw_14
    if err_14 > 8192: err_14 -= 16384
    elif err_14 < -8192: err_14 += 16384
    err_deg_14 = err_14 * (360.0 / 16384.0)
    
    print(f"Torque Enable: {last['torque_enable']}, Calib Done: {last['calib_done']}, Hold Mode: {last['pos_hold']}")
    print(f"Goal Position: {last['goal_cmd']} counts ({last['goal_cmd']*360/1024:.2f} deg)")
    print(f"Current Pos10: {last['pos10']} counts ({last['pos10']*360/1024:.2f} deg)")
    print(f"Raw 14-bit   : {last['raw_angle']} (span over 0.5s: {raw_span} counts = {raw_span*360/16384:.3f} deg)")
    print(f"Holding Iq   : {last['iq_ref']} / 300")
    print(f"Speed (RPM)  : {last['speed']} (max observed: {speed_max} RPM)")
    print(f"10-bit Error : {err_counts_10} counts = {err_deg_10:+.3f} deg")
    print(f"14-bit Error : {err_14} counts = {err_deg_14:+.3f} deg")
    
    # Check vibration criteria
    if speed_max == 0 and raw_span <= 3:
        print("\n>>> RESULT: COMPLETELY STABLE (ZERO VIBRATION) <<<")
    elif speed_max <= 8 and raw_span <= 10:
        print("\n>>> RESULT: MINIMAL NOISE JITTER (VERY SMOOTH) <<<")
    else:
        print("\n>>> WARNING: MOTOR IS OSCILLATING / VIBRATING <<<")

if __name__ == "__main__":
    main()
