import sys, time, struct
sys.path.insert(0, 'test_harness/hil')
import hil_harness as h

c = h.JointControllerClient('COM26')
c.write_u8(h.REG_TORQUE_ENABLE, 0)
c.write_u8(h.REG_CONTROL_MODE, h.MODE_POSITION)
c.write_u8(h.REG_TORQUE_ENABLE, 1)

# Step 0: move to 0 deg
c.write_bytes(h.REG_GOAL_POSITION_L, struct.pack("<hH", 0, 60))
time.sleep(1.0)
t0 = c.read_telemetry()
print('At 0 deg cmd:', t0)

# Step 1: move to 90 deg (4096 counts)
c.write_bytes(h.REG_GOAL_POSITION_L, struct.pack("<hH", 4096, 60))
for i in range(25):
    time.sleep(0.1)
    t = c.read_telemetry()
    print(f"t={i*100:4d}ms: pos={t['pos_deg']:6.2f} deg (cnt={t['pos_cnt']:5d}), cur={t['cur_ma']:4d} mA, spd={t['spd_rpm']:3d} rpm")

c.write_u8(h.REG_TORQUE_ENABLE, 0)
c.close()
