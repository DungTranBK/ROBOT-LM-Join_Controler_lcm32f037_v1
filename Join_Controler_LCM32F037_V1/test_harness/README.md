# Hệ Thống Kiểm Thử Tự Động (Test Harness — SIL & HIL)

Thư mục `test_harness/` cung cấp kiến trúc kiểm thử tự động 2 tầng cho bộ điều khiển khớp Robot **LCM32F037 (`JOINT_TYPE_NECK` / `HIP` / `KNEE`)**:

1. **Tầng 1 — PC Host Simulation & Unit Test Harness (SIL)** (`test_harness/sim/sim_harness.c`):
   - Biên dịch **trực tiếp mã nguồn C thực tế của dự án** (`Source/mid/motor_ctrl.c`, `Source/mid/motor_monitor.c`, `Source/mid/protocol.c`, `Source/mid/foc_math.c`, `Source/mid/trig_table.c`, `Source/mid/control_table.c`, `Source/bsp/bsp_mt9102.c`) bằng trình biên dịch GCC trên máy tính.
   - Ghép nối trực tiếp hàm `motor_ctrl_fast_loop()` với **Mô hình vật lý động cơ BLDC 7 cặp cực + Cảm biến Hall tuyến tính kép MT9102 + Cầu nghịch lưu 3 pha + Mô-men cogging + Ma sát tĩnh Coulomb + Nhiễu nhiệt ADC** ở tần số thực `20 kHz` (`50 us/tick`).
   - Tự động kiểm chứng **9 kịch bản cốt lõi (9/9 Suites)**:
     - `[TEST 1]` Nội suy LUT 32x `MT9102` toàn vòng $360^\circ$ (sai số `< 0.15°`) & Khử nhiễu Hysteresis khi đứng yên (`0 counts_14` nhảy số dưới nhiễu ADC `±2 LSB`).
     - `[TEST 2]` Đáp ứng bước nhảy đa điểm ở **Mode 3 (`MODE_IMPEDANCE`)**: vọt lố `≤ 0.80°`, sai số tĩnh `≤ 0.70°`, số chu kỳ rung khi dừng `ring == 0`.
     - `[TEST 3]` Đáp ứng bước nhảy đa điểm ở **Mode 2 (`MODE_POSITION`)**: bàn giao mượt từ vòng vận tốc sang giữ vị trí 14-bit, vọt lố `≤ 0.85°`, `ring == 0`.
     - `[TEST 4]` Cơ chế **Settled Hold Lock (`s_ctrl.imp_hold`)**:
       - Giữ tải tĩnh (`-0.015 N.m`) dưới nhiễu ADC liên tục trong `2000 ticks` (`20 kHz`): **độ biến thiên xung PWM `CCR1/2/3 == 0`**, **vẫn duy trì mô-men giữ tải khác 0**.
       - Khi có ngoại lực lớn (`-0.055 N.m`) tác động: tự động mở khóa trong `50 us`, kháng lực với độ cứng `5x Kp` (độ lệch `< 2.50°`), và tự động khóa êm trở lại (`0` biến thiên PWM) sau khi bỏ ngoại lực.
     - `[TEST 5]` Máy trạng thái **Tự động Hiệu chuẩn 5 bước (`CAL_STATE_ZERO_TOTAL -> ZERO_PHASE -> HALL_SWEEP -> ALIGN -> IDLE`)**: kiểm tra tự động đo điểm không dòng pha (`U, V, Total`), quét vòng hở $360^\circ$ tìm tâm/biên độ Hall `SIN/COS`, và khóa cực từ 2 chiều xác định `angle_offset`.
     - `[TEST 6]` Chuyển trạng thái động: **Đảo chiều đột ngột khi đang bay (`Mid-Flight Reversal`)**, **Chuyển Mode không giật (`Bumpless Transfer Mode 3 <-> Mode 2`)** dưới tải, và **Chế độ Damping / Mô-men thuần (`Kp = 0`)**.
     - `[TEST 7]` Vượt ranh giới cảm biến đa vòng **`0 ↔ 16383` (`Multi-Turn Wrap-Around`)**: quay vượt qua $+360^\circ$ sang vòng thứ 2 và quay về $0^\circ$ không bị gai vận tốc.
     - `[TEST 8]` Bảo vệ an toàn phần cứng (**Quá dòng `> 600 mA` trong `2000 ms`**, **Quá nhiệt NTC `> 70°C` -> tự ngắt `torque_enable = 0`**) & **Tự động bám điểm không Op-Amp khi nghỉ (`Idle Auto-Zero`)**.
     - `[TEST 9]` Toàn bộ ngăn xếp giao thức **Protocol V3 (`INST_SYNC_WRITE`, `INST_REG_WRITE` + `INST_ACTION`, Giới hạn góc mềm `ERR_FLAG_ANGLE`, Kẹp trần `Kp_max`, Khóa EEPROM `ERR_FLAG_RANGE_LOCK`, và Phát hiện lỗi `ERR_FLAG_CHECKSUM`)**.

2. **Tầng 2 — Hardware-in-the-Loop (HIL) Test Harness** (`test_harness/hil/hil_harness.py`):
   - Giao tiếp trực tiếp với mạch thật qua bus nối tiếp `1 Mbps` (mặc định `COM26`).
   - Thực thi **7 bộ kiểm thử phần cứng (7/7 Suites)**:
     - `[HIL SUITE 1]` Kiểm tra giao tiếp Bus, phiên bản Firmware, điện áp `Vbus`, nhiệt độ `Temp` và giới hạn `Kp_max / Kd_max`.
     - `[HIL SUITE 2]` Đo độ tĩnh lặng khi giữ vị trí (`Stationary Settled Hold Lock`) trong `2.0 s`.
     - `[HIL SUITE 3]` Đo đáp ứng bước nhảy đa chiều ở `MODE_IMPEDANCE` (`+90°`, `+180°`, `-90°`, `-45°`, `0°`).
     - `[HIL SUITE 4]` Đo đáp ứng bước nhảy đa chiều ở `MODE_POSITION` (`+90°`, `+180°`, `-90°`, `0°`).
     - `[HIL SUITE 5]` Kiểm tra đảo chiều đột ngột khi đang bay (`Mid-Flight Reversal`) & Chuyển Mode không giật (`Bumpless Transfer M3 <-> M2`) khi đang bật `torque_enable = 1`.
     - `[HIL SUITE 6]` Kiểm tra tập lệnh `REG_WRITE` + `ACTION`, `SYNC_WRITE`, bắt lỗi `ERR_FLAG_CHECKSUM` và chặn vượt giới hạn góc mềm `ERR_FLAG_ANGLE`.
     - `[HIL SUITE 7]` Kiểm tra vượt ranh giới cảm biến đa vòng (`+135° -> +225° -> 0°`) và tự động xuất báo cáo ra `test_harness/reports/hil_report.md`.

---

## Cách Chạy Nhanh (1 Lệnh Duy Nhất)

Mở PowerShell tại thư mục gốc dự án và chạy:

```powershell
# 1. Chạy toàn bộ (Keil Build + PC SIL Simulation + HIL trên mạch thật nếu đang cắm COM26)
powershell -ExecutionPolicy Bypass -File test_harness/run_harness.ps1

# 2. Chỉ chạy Keil Build + PC SIL Simulation (không cần cắm phần cứng)
powershell -ExecutionPolicy Bypass -File test_harness/run_harness.ps1 -Mode sim

# 3. Nạp Firmware mới nhất qua J-Link rồi chạy HIL Test trên cổng COM26
powershell -ExecutionPolicy Bypass -File test_harness/run_harness.ps1 -Mode all -Port COM26 -Flash
```
