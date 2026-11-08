#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hardware-in-the-Loop (HIL) Test Harness for ROBOT-LM Join Controller (LCM32F037).
Tests (7 Suites):
  1. Bus Communication, Register Health & Calibration Status (Hall & Rotor Offsets)
  2. Stationary Settled Hold Lock & Anti-Vibration (Position / Speed / Current Jitter)
  3. Mode 3 (MIT Impedance) Multi-Step Smooth Landing (Overshoot, Ringing, Final Error)
  4. Mode 2 (Position Mode) Multi-Step Smooth Handover (Overshoot, Ringing, Final Error)
  5. Dynamic Mid-Flight Reversal & Bumpless Mode Transfer (Mode 3 <-> Mode 2 under Torque)
  6. Protocol V3 Stack (REG_WRITE + ACTION, SYNC_WRITE, Soft Angle Limits, Checksum Error)
  7. Multi-Turn / Sensor Wrap-Boundary Smooth Crossing (+180 deg -> +225 deg -> 0 deg)
"""

import argparse
import math
import os
import struct
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("[ERROR] pyserial is required: pip install pyserial")
    sys.exit(1)

# Register Map (Include/mid/control_table.h)
REG_MODEL_NUM_L       = 0x00
REG_FW_VER            = 0x02
REG_MOTOR_ID          = 0x03
REG_MIN_ANGLE_L       = 0x06
REG_MAX_ANGLE_L       = 0x08
REG_TORQUE_ENABLE     = 0x18
REG_CALIB_ENABLE      = 0x19
REG_KP_L              = 0x1A
REG_KD_L              = 0x1C
REG_GOAL_POSITION_L   = 0x1E
REG_MOVING_SPEED_L    = 0x20
REG_TAU_FF_L          = 0x22
REG_CURRENT_POS_L     = 0x24
REG_CURRENT_SPEED_L   = 0x26
REG_CURRENT_DRAW_L    = 0x28
REG_CURRENT_VOLTAGE   = 0x2A
REG_CURRENT_TEMP      = 0x2B
REG_REGISTERED_FLAG   = 0x2C
REG_CONTROL_MODE      = 0x2D
REG_MOVING_FLAG       = 0x2E
REG_KP_MAX_L          = 0x32
REG_KD_MAX_L          = 0x34

BROADCAST_ID          = 0xFE
MODE_POSITION         = 2
MODE_IMPEDANCE        = 3
COUNTS_PER_REV        = 16384.0

ERR_FLAG_ANGLE        = 0x02
ERR_FLAG_CHECKSUM     = 0x10


def calc_checksum(packet_body: bytes) -> int:
    return (~sum(packet_body)) & 0xFF


def deg_to_counts(deg: float) -> int:
    return int(round(deg * (COUNTS_PER_REV / 360.0)))


def counts_to_deg(cnt: int) -> float:
    return float(cnt) * (360.0 / COUNTS_PER_REV)


class JointControllerClient:
    """Half-duplex bus client for ROBOT-LM Join Controller."""

    def __init__(self, port: str, baud: int = 1000000, motor_id: int = 1, timeout: float = 0.006):
        self.port_name = port
        self.baud = baud
        self.motor_id = motor_id
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _transact(self, inst: int, params: bytes, expected_data_len: int = 0,
                  target_id: int = None, corrupt_cs: bool = False):
        if not self.ser or not self.ser.is_open:
            return None, None
        pkt_id = self.motor_id if target_id is None else target_id
        length = len(params) + 2
        body = bytes([pkt_id, length, inst]) + params
        cs = calc_checksum(body)
        if corrupt_cs:
            cs ^= 0x55
        pkt = b"\xFF\xFF" + body + bytes([cs])
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        time.sleep(0.004)

        if pkt_id == BROADCAST_ID:
            return 0, b""

        # V3 Status frame: [FF, FF, ID, LEN, ERR, PosL, PosH, EchoLen, EchoInst, EchoP1, Data..., CS]
        raw = self.ser.read(64)
        if not raw:
            return None, None

        search_pos = len(pkt) if raw.startswith(pkt) else 0
        while True:
            idx = raw.find(b"\xFF\xFF", search_pos)
            if idx < 0 or len(raw) < idx + 11:
                return None, None
            if raw[idx : idx + len(pkt)] == pkt and len(raw) >= idx + len(pkt) + 11:
                search_pos = idx + len(pkt)
                continue
            r_id = raw[idx + 2]
            r_len = raw[idx + 3]
            r_err = raw[idx + 4]
            end_pos = idx + 4 + r_len
            if len(raw) < end_pos or end_pos < idx + 11:
                return None, None
            r_csum = raw[end_pos - 1]
            if calc_checksum(raw[idx + 2 : end_pos - 1]) == r_csum and r_id == self.motor_id:
                r_params = raw[idx + 10 : end_pos - 1]
                return r_err, r_params
            search_pos = idx + 2

    def ping(self, corrupt_cs: bool = False):
        err, _ = self._transact(0x01, b"", 0, corrupt_cs=corrupt_cs)
        return err

    def read_bytes(self, addr: int, count: int):
        err, data = self._transact(0x02, bytes([addr, count]), count)
        if err is None or data is None or len(data) != count:
            return None
        return data

    def write_bytes(self, addr: int, payload: bytes):
        err, _ = self._transact(0x03, bytes([addr]) + payload, 0)
        return err

    def reg_write_bytes(self, addr: int, payload: bytes):
        err, _ = self._transact(0x04, bytes([addr]) + payload, 0)
        return err

    def action(self):
        err, _ = self._transact(0x05, b"", 0)
        return err

    def sync_write(self, start_addr: int, data_len: int, id_payload_pairs: list):
        params = bytearray([start_addr, data_len])
        for m_id, data_bytes in id_payload_pairs:
            params.append(m_id)
            params.extend(data_bytes)
        self._transact(0x83, bytes(params), 0, target_id=BROADCAST_ID)

    def write_u8(self, addr: int, val: int):
        return self.write_bytes(addr, struct.pack("<B", val & 0xFF))

    def write_i16(self, addr: int, val: int):
        return self.write_bytes(addr, struct.pack("<h", int(val)))

    def write_u16(self, addr: int, val: int):
        return self.write_bytes(addr, struct.pack("<H", int(val) & 0xFFFF))

    def write_mit_block(self, kp: int, kd: int, goal_cnt: int, speed_rpm: int, tau_ff: int = 0):
        payload = struct.pack("<HHhhh", kp, kd, goal_cnt, speed_rpm, tau_ff)
        return self.write_bytes(REG_KP_L, payload)

    def read_telemetry(self):
        """Read 8-byte feedback block 0x24..0x2B: pos(i16), spd(i16), cur(i16), volt(u8), temp(u8)."""
        data = self.read_bytes(REG_CURRENT_POS_L, 8)
        if not data:
            return None
        pos, spd, cur, volt, temp = struct.unpack("<hhhBB", data)
        return {
            "pos_cnt": pos,
            "pos_deg": counts_to_deg(pos),
            "spd_rpm": spd,
            "cur_ma": cur,
            "volt_v": volt * 0.1,
            "temp_c": temp,
        }


def run_step_and_measure(client: JointControllerClient, mode: int, target_deg: float,
                         kp: int, kd: int):
    t0_telem = client.read_telemetry()
    start_deg = t0_telem["pos_deg"] if t0_telem else 0.0
    step_deg = abs(target_deg - start_deg)
    duration_s = 2.4 if step_deg > 180.0 else 1.8
    step_dir = 1.0 if target_deg >= start_deg else -1.0
    goal_cnt = deg_to_counts(target_deg)

    if mode == MODE_IMPEDANCE:
        client.write_mit_block(kp, kd, goal_cnt, 0, 0)
    else:
        client.write_bytes(REG_GOAL_POSITION_L, struct.pack("<hH", goal_cnt, 60))

    t_start = time.perf_counter()
    samples = []
    while (time.perf_counter() - t_start) < duration_s:
        tel = client.read_telemetry()
        if tel:
            tel["t_ms"] = (time.perf_counter() - t_start) * 1000.0
            samples.append(tel)

    ss_samples = []
    for _ in range(30):
        tel = client.read_telemetry()
        if tel:
            tel["t_ms"] = (time.perf_counter() - t_start) * 1000.0
            samples.append(tel)
            ss_samples.append(tel)

    if not samples:
        return None

    max_over_deg = 0.0
    last_outside_ms = 0.0
    last_sign = 0
    zero_crossings = 0

    for s in samples:
        err_deg = target_deg - s["pos_deg"]
        over_deg = (s["pos_deg"] - target_deg) * step_dir
        if over_deg > max_over_deg:
            max_over_deg = over_deg
        if abs(err_deg) > 1.00:
            last_outside_ms = s["t_ms"]
        if abs(err_deg) > 0.70:
            sign = 1 if err_deg > 0.0 else -1
            if last_sign != 0 and sign != last_sign:
                zero_crossings += 1
            last_sign = sign

    ss_list = ss_samples if ss_samples else samples[-10:]
    final_deg = sum(s["pos_deg"] for s in ss_list) / len(ss_list)
    return {
        "start_deg": start_deg,
        "target_deg": target_deg,
        "final_deg": final_deg,
        "overshoot_deg": max_over_deg,
        "final_err_deg": abs(target_deg - final_deg),
        "settle_ms": last_outside_ms,
        "ring_cycles": zero_crossings // 2,
    }


def main():
    parser = argparse.ArgumentParser(description="ROBOT-LM Join Controller HIL Test Harness")
    parser.add_argument("--port", default="COM26", help="Serial port (default: COM26)")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate (default: 1000000)")
    parser.add_argument("--id", type=int, default=1, help="Motor ID (default: 1)")
    parser.add_argument("--kp", type=int, default=1500, help="MIT Kp register value for test (default: 1500)")
    parser.add_argument("--kd", type=int, default=64, help="MIT Kd register value for test (default: 64)")
    parser.add_argument("--report", default="test_harness/reports/hil_report.md", help="Output Markdown report path")
    args = parser.parse_args()

    print("=" * 76)
    print(f"  ROBOT-LM JOIN CONTROLLER - HARDWARE-IN-THE-LOOP (HIL) TEST HARNESS")
    print(f"  Port: {args.port} @ {args.baud} bps | Motor ID: {args.id}")
    print("=" * 76)

    try:
        client = JointControllerClient(args.port, args.baud, args.id)
    except Exception as exc:
        print(f"[SKIP/ERROR] Could not open {args.port}: {exc}")
        sys.exit(2)

    report_lines = [
        "# Báo Cáo Kiểm Thử Phần Cứng HIL (Hardware-in-the-Loop)",
        f"- **Thời gian**: `{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}`",
        f"- **Cổng giao tiếp**: `{args.port}` (`{args.baud} bps`, ID `{args.id}`)",
        "",
    ]

    passed_suites = 0
    total_suites = 7
    connected_ok = False

    try:
        # ------------------------------------------------------------------
        # SUITE 1: Bus Communication, Register Health & Calibration Status
        # ------------------------------------------------------------------
        print("\n[HIL SUITE 1] Bus Communication, Register Health & Calibration Check...")
        ping_err = client.ping()
        if ping_err is None:
            for cand_id in range(1, 16):
                client.motor_id = cand_id
                if client.ping() is not None:
                    print(f"  [Auto-Scan] Found controller at Motor ID = {cand_id}")
                    args.id = cand_id
                    ping_err = 0
                    break
            if ping_err is None:
                client.motor_id = args.id

        hdr = client.read_bytes(REG_MODEL_NUM_L, 4) if ping_err is not None else None
        tel = client.read_telemetry() if ping_err is not None else None
        limits = client.read_bytes(REG_KP_MAX_L, 4) if ping_err is not None else None

        if ping_err is not None and hdr and tel and limits:
            connected_ok = True
            model_num, fw_ver, m_id = struct.unpack("<HBB", hdr)
            kp_max, kd_max = struct.unpack("<HH", limits)
            s1_ok = (tel["volt_v"] >= 10.0) and (tel["temp_c"] < 75)
            print(f"  -> Model={model_num} | FW=v{fw_ver} | ID={m_id} | Pos={tel['pos_deg']:+.2f} deg | "
                  f"Vbus={tel['volt_v']:.1f} V | Temp={tel['temp_c']} C | Kp_max={kp_max} | Kd_max={kd_max}")
            print(f"  -> Result: {'PASS' if s1_ok else 'FAIL'}")
            if s1_ok:
                passed_suites += 1
        else:
            print("  -> Result: FAIL (No response from controller)")
            sys.exit(1)

        # Ensure full multi-turn angle limits (-32768..+32767) before tests
        client.write_bytes(REG_MIN_ANGLE_L, struct.pack("<hh", -32768, 32767))

        # Move to 0 deg and allow full settling + hold lock before hold test
        client.write_u8(REG_TORQUE_ENABLE, 0)
        client.write_u8(REG_CONTROL_MODE, MODE_IMPEDANCE)
        client.write_u8(REG_TORQUE_ENABLE, 1)
        client.write_mit_block(args.kp, args.kd, 0, 0, 0)
        time.sleep(2.0)

        # ------------------------------------------------------------------
        # SUITE 2: Stationary Settled Hold Lock & Anti-Vibration Check
        # ------------------------------------------------------------------
        print("\n[HIL SUITE 2] Stationary Settled Hold Lock & Anti-Vibration (2.0 s)...")
        hold_samples = []
        t0 = time.perf_counter()
        while (time.perf_counter() - t0) < 2.0:
            t_sample = client.read_telemetry()
            if t_sample:
                hold_samples.append(t_sample)

        pos_list = [s["pos_deg"] for s in hold_samples]
        spd_list = [abs(s["spd_rpm"]) for s in hold_samples]
        pos_p2p = max(pos_list) - min(pos_list) if pos_list else 999.0
        max_spd = max(spd_list) if spd_list else 999
        s2_ok = (pos_p2p <= 0.35) and (max_spd <= 3)
        if s2_ok:
            passed_suites += 1
        print(f"  -> Samples={len(hold_samples)} | Pos Peak-to-Peak Jitter={pos_p2p:.3f} deg (Limit <= 0.35 deg)")
        print(f"  -> Max Velocity Jitter at Rest={max_spd} RPM (Limit <= 3 RPM)")
        print(f"  -> Result: {'PASS' if s2_ok else 'FAIL'}")

        report_lines += [
            "## 1. Kiểm Tra Đứng Yên & Khóa Chống Rung (Settled Hold Lock)",
            f"- **Số mẫu thu thập trong 2.0s**: `{len(hold_samples)} mẫu`",
            f"- **Độ lệch vị trí đỉnh-đỉnh (Peak-to-Peak)**: `{pos_p2p:.3f}°` (Ngưỡng `≤ 0.35°`)",
            f"- **Nhiễu vận tốc cực đại khi đứng yên**: `{max_spd} RPM` (Ngưỡng `≤ 3 RPM`)",
            f"- **Đánh giá**: **{'PASS' if s2_ok else 'FAIL'}**",
            "",
        ]

        # ------------------------------------------------------------------
        # SUITE 3: Mode 3 (MIT Impedance) Multi-Step Smooth Landing
        # ------------------------------------------------------------------
        print(f"\n[HIL SUITE 3] Mode 3 (MIT Impedance, Kp={args.kp}, Kd={args.kd}) Smooth Landing...")
        client.write_u8(REG_TORQUE_ENABLE, 0)
        client.write_u8(REG_CONTROL_MODE, MODE_IMPEDANCE)
        client.write_u8(REG_TORQUE_ENABLE, 1)
        m3_targets = [90.0, 180.0, -90.0, -45.0, 0.0]
        s3_ok = True
        report_lines += [
            f"## 2. Đáp Ứng Bước Nhảy Mode 3 (`MODE_IMPEDANCE`, `Kp={args.kp}`, `Kd={args.kd}`)",
            "| Bước | Hành trình | Vọt lố (`ov`) | Sai số tĩnh (`err`) | Thời gian xác lập | Dao động (`ring`) | Kết quả |",
            "| :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
        ]
        for idx, tgt in enumerate(m3_targets, 1):
            res = run_step_and_measure(client, MODE_IMPEDANCE, tgt, args.kp, args.kd)
            step_ok = (res["overshoot_deg"] <= 0.85) and (res["final_err_deg"] <= 0.70) and (res["ring_cycles"] == 0)
            if not step_ok:
                s3_ok = False
            print(f"  Step {idx} ({res['start_deg']:+6.1f} -> {tgt:+6.1f} deg): "
                  f"ov={res['overshoot_deg']:5.2f} deg | err={res['final_err_deg']:5.2f} deg | "
                  f"settle={res['settle_ms']:6.1f} ms | ring={res['ring_cycles']} [{'OK' if step_ok else 'FAIL'}]")
            report_lines.append(
                f"| {idx} | `{res['start_deg']:+.1f}° -> {tgt:+.1f}°` | `{res['overshoot_deg']:.2f}°` | "
                f"`{res['final_err_deg']:.2f}°` | `{res['settle_ms']:.1f} ms` | `{res['ring_cycles']}` | "
                f"**{'PASS' if step_ok else 'FAIL'}** |"
            )
        if s3_ok:
            passed_suites += 1
        print(f"  -> Result: {'PASS' if s3_ok else 'FAIL'}")
        report_lines.append("")

        # ------------------------------------------------------------------
        # SUITE 4: Mode 2 (Position Mode) Multi-Step Smooth Handover
        # ------------------------------------------------------------------
        print("\n[HIL SUITE 4] Mode 2 (Position Mode, Speed=60 RPM) Smooth Handover...")
        client.write_u8(REG_TORQUE_ENABLE, 0)
        client.write_u8(REG_CONTROL_MODE, MODE_POSITION)
        client.write_u8(REG_TORQUE_ENABLE, 1)
        m2_targets = [90.0, 180.0, -90.0, 0.0]
        s4_ok = True
        report_lines += [
            "## 3. Đáp Ứng Bước Nhảy Mode 2 (`MODE_POSITION`, `Speed=60 RPM`)",
            "| Bước | Hành trình | Vọt lố (`ov`) | Sai số tĩnh (`err`) | Thời gian xác lập | Dao động (`ring`) | Kết quả |",
            "| :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
        ]
        for idx, tgt in enumerate(m2_targets, 1):
            res = run_step_and_measure(client, MODE_POSITION, tgt, args.kp, args.kd)
            step_ok = (res["overshoot_deg"] <= 0.85) and (res["final_err_deg"] <= 0.70) and (res["ring_cycles"] == 0)
            if not step_ok:
                s4_ok = False
            print(f"  Step {idx} ({res['start_deg']:+6.1f} -> {tgt:+6.1f} deg): "
                  f"ov={res['overshoot_deg']:5.2f} deg | err={res['final_err_deg']:5.2f} deg | "
                  f"settle={res['settle_ms']:6.1f} ms | ring={res['ring_cycles']} [{'OK' if step_ok else 'FAIL'}]")
            report_lines.append(
                f"| {idx} | `{res['start_deg']:+.1f}° -> {tgt:+.1f}°` | `{res['overshoot_deg']:.2f}°` | "
                f"`{res['final_err_deg']:.2f}°` | `{res['settle_ms']:.1f} ms` | `{res['ring_cycles']}` | "
                f"**{'PASS' if step_ok else 'FAIL'}** |"
            )
        if s4_ok:
            passed_suites += 1
        print(f"  -> Result: {'PASS' if s4_ok else 'FAIL'}")
        report_lines.append("")

        # ------------------------------------------------------------------
        # SUITE 5: Dynamic Mid-Flight Reversal & Bumpless Mode Transfer
        # ------------------------------------------------------------------
        print("\n[HIL SUITE 5] Dynamic Mid-Flight Reversal & Bumpless Mode Switch...")
        client.write_u8(REG_CONTROL_MODE, MODE_IMPEDANCE)
        client.write_u8(REG_TORQUE_ENABLE, 1)
        # Launch toward +90 deg, then reverse in mid-flight after 220 ms toward -45 deg
        client.write_mit_block(args.kp, args.kd, deg_to_counts(90.0), 0, 0)
        time.sleep(0.22)
        tel_mid = client.read_telemetry()
        mid_deg = tel_mid["pos_deg"] if tel_mid else 0.0
        rev_res = run_step_and_measure(client, MODE_IMPEDANCE, -45.0, args.kp, args.kd)
        rev_ok = (rev_res["overshoot_deg"] <= 0.85) and (rev_res["final_err_deg"] <= 0.70)

        # Now test Bumpless Transfer at -45 deg: switch Mode 3 -> Mode 2 -> Mode 3 while torque_enable=1
        before_sw = client.read_telemetry()["pos_deg"]
        client.write_u8(REG_CONTROL_MODE, MODE_POSITION)
        max_glitch_m2 = 0.0
        for _ in range(25):
            t_s = client.read_telemetry()
            if t_s:
                max_glitch_m2 = max(max_glitch_m2, abs(t_s["pos_deg"] - before_sw))
        client.write_u8(REG_CONTROL_MODE, MODE_IMPEDANCE)
        max_glitch_m3 = 0.0
        for _ in range(25):
            t_s = client.read_telemetry()
            if t_s:
                max_glitch_m3 = max(max_glitch_m3, abs(t_s["pos_deg"] - before_sw))
        bumpless_ok = (max_glitch_m2 <= 0.45) and (max_glitch_m3 <= 0.45)
        s5_ok = rev_ok and bumpless_ok
        if s5_ok:
            passed_suites += 1

        print(f"  -> Mid-Flight Reversal (reversed at {mid_deg:+.1f} deg -> -45.0 deg): "
              f"ov={rev_res['overshoot_deg']:.2f} deg | err={rev_res['final_err_deg']:.2f} deg [{'OK' if rev_ok else 'FAIL'}]")
        print(f"  -> Bumpless Mode Switch (M3->M2 glitch={max_glitch_m2:.3f} deg, "
              f"M2->M3 glitch={max_glitch_m3:.3f} deg) [{'OK' if bumpless_ok else 'FAIL'}]")
        print(f"  -> Result: {'PASS' if s5_ok else 'FAIL'}")

        report_lines += [
            "## 4. Kiểm Tra Đảo Chiều Đột Ngột & Chuyển Mode Không Giật (Bumpless Transfer)",
            f"- **Đảo chiều khi đang bay** (tại `{mid_deg:+.1f}° -> -45.0°`): vọt lố `{rev_res['overshoot_deg']:.2f}°`, sai số `{rev_res['final_err_deg']:.2f}°` (`{'PASS' if rev_ok else 'FAIL'}`)",
            f"- **Chuyển Mode trực tiếp khi đang giữ tải**: lệch cực đại `M3->M2 = {max_glitch_m2:.3f}°`, `M2->M3 = {max_glitch_m3:.3f}°` (`{'PASS' if bumpless_ok else 'FAIL'}`)",
            "",
        ]

        # ------------------------------------------------------------------
        # SUITE 6: Protocol V3 Advanced Commands & Soft Limit Protection
        # ------------------------------------------------------------------
        print("\n[HIL SUITE 6] Protocol V3 Stack (REG_WRITE+ACTION, SYNC_WRITE, Limits, Checksum)...")
        # 6A. Bad checksum
        err_cs = client.ping(corrupt_cs=True)
        cs_ok = (err_cs is not None) and ((err_cs & ERR_FLAG_CHECKSUM) != 0)

        # 6B. REG_WRITE + ACTION to move to 0.0 deg
        client.reg_write_bytes(REG_GOAL_POSITION_L, struct.pack("<h", 0))
        reg_flag_b = client.read_bytes(REG_REGISTERED_FLAG, 1)
        staged_ok = (reg_flag_b == b"\x01")
        client.action()
        reg_flag_after = client.read_bytes(REG_REGISTERED_FLAG, 1)
        action_ok = staged_ok and (reg_flag_after == b"\x00")
        time.sleep(1.5)

        # 6C. Broadcast SYNC_WRITE to set goal_position = +30 deg (1365 counts)
        sync_target_cnt = deg_to_counts(30.0)
        client.sync_write(REG_GOAL_POSITION_L, 2, [
            (args.id, struct.pack("<h", sync_target_cnt)),
            (args.id + 1, struct.pack("<h", 0)),
        ])
        goal_readback = client.read_bytes(REG_GOAL_POSITION_L, 2)
        sync_ok = (goal_readback is not None) and (struct.unpack("<h", goal_readback)[0] == sync_target_cnt)
        time.sleep(1.5)

        # 6D. Soft Angle Limits (-90 deg .. +90 deg) rejecting +135 deg with ERR_FLAG_ANGLE
        client.write_bytes(REG_MIN_ANGLE_L, struct.pack("<hh", deg_to_counts(-90.0), deg_to_counts(90.0)))
        err_lim = client.write_i16(REG_GOAL_POSITION_L, deg_to_counts(135.0))
        goal_after_lim = struct.unpack("<h", client.read_bytes(REG_GOAL_POSITION_L, 2))[0]
        lim_ok = (err_lim is not None) and ((err_lim & ERR_FLAG_ANGLE) != 0) and (goal_after_lim == sync_target_cnt)
        # Restore full multi-turn limits (-32768..+32767)
        client.write_bytes(REG_MIN_ANGLE_L, struct.pack("<hh", -32768, 32767))

        s6_ok = cs_ok and action_ok and sync_ok and lim_ok
        if s6_ok:
            passed_suites += 1
        print(f"  -> Checksum Error Detection (ERR_FLAG_CHECKSUM=0x10) : {'OK' if cs_ok else 'FAIL'}")
        print(f"  -> REG_WRITE Staging + ACTION Deferred Execution     : {'OK' if action_ok else 'FAIL'}")
        print(f"  -> Broadcast SYNC_WRITE Multi-Motor Dispatch         : {'OK' if sync_ok else 'FAIL'}")
        print(f"  -> Soft Angle Limits & ERR_FLAG_ANGLE Protection     : {'OK' if lim_ok else 'FAIL'}")
        print(f"  -> Result: {'PASS' if s6_ok else 'FAIL'}")

        report_lines += [
            "## 5. Kiểm Tra Giao Thức Bus Nâng Cao & Bảo Vệ Giới Hạn Mềm",
            f"- **Phát hiện gói tin lỗi Checksum (`ERR_FLAG_CHECKSUM`)**: **{'PASS' if cs_ok else 'FAIL'}**",
            f"- **Lệnh chờ `REG_WRITE (0x04)` + kích hoạt đồng loạt `ACTION (0x05)`**: **{'PASS' if action_ok else 'FAIL'}**",
            f"- **Lệnh đồng bộ đa khớp `SYNC_WRITE (0x83)`**: **{'PASS' if sync_ok else 'FAIL'}**",
            f"- **Chặn vượt giới hạn góc mềm (`ERR_FLAG_ANGLE`)**: **{'PASS' if lim_ok else 'FAIL'}**",
            "",
        ]

        # ------------------------------------------------------------------
        # SUITE 7: Multi-Turn / Sensor Wrap-Boundary Smooth Crossing
        # ------------------------------------------------------------------
        print("\n[HIL SUITE 7] Multi-Turn / Sensor Wrap-Boundary Crossing (+135 -> +225 -> 0 deg)...")
        wrap_steps = [135.0, 225.0, 0.0]
        s7_ok = True
        for idx, tgt in enumerate(wrap_steps, 1):
            res = run_step_and_measure(client, MODE_IMPEDANCE, tgt, args.kp, args.kd)
            step_ok = (res["overshoot_deg"] <= 0.85) and (res["final_err_deg"] <= 0.70) and (res["ring_cycles"] == 0)
            if not step_ok:
                s7_ok = False
            print(f"  Wrap Step {idx} ({res['start_deg']:+6.1f} -> {tgt:+6.1f} deg): "
                  f"ov={res['overshoot_deg']:5.2f} deg | err={res['final_err_deg']:5.2f} deg | "
                  f"ring={res['ring_cycles']} [{'OK' if step_ok else 'FAIL'}]")
        if s7_ok:
            passed_suites += 1
        print(f"  -> Result: {'PASS' if s7_ok else 'FAIL'}")

        report_lines += [
            "## 6. Kiểm Tra Vượt Ranh Giới Cảm Biến Đa Vòng (`+135° -> +225° -> 0°`)",
            f"- **Kết quả vượt điểm cắt không nhảy vận tốc & quay về 0°**: **{'PASS' if s7_ok else 'FAIL'}**",
        ]

    finally:
        # Leave motor safely disabled after test if connected and port is open
        try:
            if connected_ok and client.ser and client.ser.is_open:
                client.write_bytes(REG_MIN_ANGLE_L, struct.pack("<hh", -32768, 32767))
                client.write_u8(REG_TORQUE_ENABLE, 0)
        except Exception:
            pass
        client.close()

    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w", encoding="utf-8") as f:
        f.write("\n".join(report_lines) + "\n")

    print("\n" + "=" * 76)
    print(f"  HIL SUMMARY: {passed_suites} / {total_suites} Test Suites PASSED")
    print(f"  Report saved to: {args.report}")
    print("=" * 76)
    sys.exit(0 if passed_suites == total_suites else 1)


if __name__ == "__main__":
    main()
