# Báo Cáo Kiểm Thử Phần Cứng HIL (Hardware-in-the-Loop)
- **Thời gian**: `2026-11-09 00:24:37`
- **Cổng giao tiếp**: `COM26` (`1000000 bps`, ID `1`)

## 1. Kiểm Tra Đứng Yên & Khóa Chống Rung (Settled Hold Lock)
- **Số mẫu thu thập trong 2.0s**: `129 mẫu`
- **Độ lệch vị trí đỉnh-đỉnh (Peak-to-Peak)**: `0.154°` (Ngưỡng `≤ 0.35°`)
- **Nhiễu vận tốc cực đại khi đứng yên**: `0 RPM` (Ngưỡng `≤ 3 RPM`)
- **Đánh giá**: **PASS**

## 2. Đáp Ứng Bước Nhảy Mode 3 (`MODE_IMPEDANCE`, `Kp=1500`, `Kd=64`)
| Bước | Hành trình | Vọt lố (`ov`) | Sai số tĩnh (`err`) | Thời gian xác lập | Dao động (`ring`) | Kết quả |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | `-1.1° -> +90.0°` | `1.08°` | `0.21°` | `1043.2 ms` | `0` | **FAIL** |
| 2 | `+90.2° -> +180.0°` | `0.00°` | `0.31°` | `968.9 ms` | `0` | **PASS** |
| 3 | `+179.7° -> -90.0°` | `0.40°` | `0.28°` | `2301.1 ms` | `0` | **PASS** |
| 4 | `-90.3° -> -45.0°` | `0.00°` | `0.24°` | `717.0 ms` | `0` | **PASS** |
| 5 | `-45.2° -> +0.0°` | `0.22°` | `0.13°` | `499.1 ms` | `0` | **PASS** |

## 3. Đáp Ứng Bước Nhảy Mode 2 (`MODE_POSITION`, `Speed=60 RPM`)
| Bước | Hành trình | Vọt lố (`ov`) | Sai số tĩnh (`err`) | Thời gian xác lập | Dao động (`ring`) | Kết quả |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | `-0.9° -> +90.0°` | `2.42°` | `0.23°` | `667.6 ms` | `0` | **FAIL** |
| 2 | `+90.2° -> +180.0°` | `0.00°` | `0.27°` | `699.8 ms` | `0` | **PASS** |
| 3 | `+179.7° -> -90.0°` | `1.49°` | `0.01°` | `2882.4 ms` | `16` | **FAIL** |
| 4 | `-89.3° -> +0.0°` | `0.00°` | `0.22°` | `485.4 ms` | `0` | **PASS** |

## 4. Kiểm Tra Đảo Chiều Đột Ngột & Chuyển Mode Không Giật (Bumpless Transfer)
- **Đảo chiều khi đang bay** (tại `+30.0° -> -45.0°`): vọt lố `0.44°`, sai số `0.28°` (`PASS`)
- **Chuyển Mode trực tiếp khi đang giữ tải**: lệch cực đại `M3->M2 = 0.242°`, `M2->M3 = 0.110°` (`PASS`)

## 5. Kiểm Tra Giao Thức Bus Nâng Cao & Bảo Vệ Giới Hạn Mềm
- **Phát hiện gói tin lỗi Checksum (`ERR_FLAG_CHECKSUM`)**: **PASS**
- **Lệnh chờ `REG_WRITE (0x04)` + kích hoạt đồng loạt `ACTION (0x05)`**: **PASS**
- **Lệnh đồng bộ đa khớp `SYNC_WRITE (0x83)`**: **PASS**
- **Chặn vượt giới hạn góc mềm (`ERR_FLAG_ANGLE`)**: **PASS**

## 6. Kiểm Tra Vượt Ranh Giới Cảm Biến Đa Vòng (`+135° -> +225° -> 0°`)
- **Kết quả vượt điểm cắt không nhảy vận tốc & quay về 0°**: **FAIL**
