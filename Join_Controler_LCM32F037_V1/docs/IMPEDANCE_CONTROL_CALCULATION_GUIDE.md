# HUONG DAN TINH TOAN THONG SO DIEU KHIEN TRO KHANG (IMPEDANCE CONTROL)
## Ap dung cho Actuator BLDC K-Power EC4012_HIP (Hop so hanh tinh 7.5:1) — Chuan hoa don vi SI theo Master

---

## 1. TONG QUAN & DON VI CHUAN CUA MASTER (OUTPUT SHAFT SI UNITS)

Trong che do Impedance Mode (MIT-Cheetah style, INST=0x90, Mode=3), dong co BLDC `EC4012_HIP` nhan cac tham so `Kp`, `Kd`, `tau_ff` theo **cung he don vi vat ly SI tai truc ra (Output Shaft - sau hop so 7.5:1)** voi bo dieu khien cap cao (Master `Lumi-Tanker`) va dong co `GIM6010-8`:

| Dai luong | Don vi tren Master / Web UI | Don vi thanh ghi UART (16-bit int) | Cong thuc |
|---|---|---|---|
| **Vi tri (`p_goal`)** | `counts` truc ra (`1024 counts = 1 vong = 2*pi rad`) | `int16_t` (`1 LSB = 1 count`) | Giu nguyen `1024 counts/vong` |
| **Van toc (`v_goal`)** | `RPM` | `int16_t` (`1 LSB = 1 RPM`) | Van toc dat (`= 0` khi giu goc) |
| **Do cung (`Kp`)** | **`N.m / rad`** (tai truc ra) | `uint16_t` (`1 LSB = 0.001 N.m/rad`) | `kp_reg = Kp [N.m/rad] * 1000` |
| **Giam chan (`Kd`)** | **`N.m.s / rad`** (tai truc ra) | `uint16_t` (`1 LSB = 0.001 N.m.s/rad`) | `kd_reg = Kd [N.m.s/rad] * 1000` |
| **Mo-men bu (`tau_ff`)**| **`N.m`** (tai truc ra) | `int16_t` (`1 LSB = 0.001 N.m = 1 mN.m`) | `tau_reg = tau_ff [N.m] * 1000` |

---

## 2. THONG SO CO - DIEN CUA DONG CO KHOP EC4012_HIP (HOP SO 7.5 : 1)

Dua tren Datasheet dong co outrunner K-Power EC4012, cum hop so hanh tinh 7.5:1 (`15/2`) va bo dieu khien LCM32F037:

1. **Dien ap hoat dong dinh muc:** `12 V`
2. **Ti so truyen hop so hanh tinh (`foc_config.h`):** `N = 15 / 2 = 7.5 : 1`
3. **Dong dien dinh muc:** `I_rated = 0.95 A` (Dong dinh: `I_peak = 2.0 A`)
4. **Mo-men xoan dinh muc:**
   - Tai rotor (truoc hop so): `tau_rotor_rated = 0.13 N.m`
   - Tai truc khop (sau hop so 7.5:1): `tau_out_rated ≈ 1.00 N.m` (Peak: `~1.50 N.m`)
5. **Hang so mo-men Kt (tai rotor):**
   ```text
   Kt = tau_rotor_rated / I_rated = 0.13 N.m / 0.95 A = 0.13684 N.m/A
   ```
6. **Mo-men quan tinh quy doi ve truc rotor (`J_eq_rotor`):**
   ```text
   J_eq_rotor = J_rotor + J_gearbox_reflected ≈ 5.0 * 10^-5 kg.m^2
   ```
7. **Do phan giai cam bien goc (`control_table.h`):**
   - `POSITION_COUNTS_PER_REV = 1024 counts / vong`
   - Khi truc ra quay `1 vong (2*pi rad)`, rotor quay `7.5 * 1024 = 7680 rotor_counts`.
8. **Thang do dong dien & mo-men trong Firmware (`motor_monitor.c` & `motor_ctrl.c`):**
   ```text
   CURRENT_SCALE_MA = 3.256 mA / count = 0.003256 A / Iq_count
   IQ_LIMIT         = 300 counts (≈ 0.977 A)
   K_tau_rotor      = Kt * 0.003256 = 0.0004454 N.m / Iq_count
   K_tau_out        = N * K_tau_rotor = 7.5 * 0.0004454 = 0.0033405 N.m / Iq_count
   ===> 1.000 N.m tai truc ra = 1 / 0.0033405 ≈ 300 Iq_counts (IQ_LIMIT)
   ```

---

## 3. CONG THUC QUY DOI TU DON VI MASTER (SI TRUC RA) SANG SO NGUYEN ROTOR TRONG `motor_ctrl.c`

Ben trong `motor_ctrl.c`, sai so vi tri `p_err` duoc tinh bang `rotor_counts` va sai so van toc `v_err` duoc tinh bang `rotor_RPM` de giu do phan giai cao nhat. Cac ham inline `si_kp_to_rotor()`, `si_kd_to_rotor()`, `si_tau_to_iq()` quy doi tu don vi SI tai truc ra sang he so rotor:

1. **Ba hang so trung gian tai truc ra:**
   - **`300 Iq_counts / N.m`**: `1 N.m` tai truc ra tuong ung `300 Iq_counts`.
   - **`1222.31 rotor_counts / rad`**: `1 rad` tai truc ra `= (7.5 * 1024) / (2 * pi) = 1222.31 rotor_counts`.
   - **`71.6197 rotor_RPM / (rad/s)`**: `1 rad/s` tai truc ra `= (7.5 * 60) / (2 * pi) = 71.6197 rotor_RPM`.

2. **Quy doi `Kp` (`N.m/rad` tai truc ra -> `Iq_count / rotor_count`):**
   ```text
   Kp_rotor = Kp_SI * (300 / 1222.31) = Kp_SI * 0.245437 ≈ (Kp_SI * 2454) / 10000
   ```
   - Vi du: `Kp = 10.0 N.m/rad` (`kp_reg = 10000`) -> `s_imp_kp = 2454` (`2.454 Iq/rotor_count`).
   - Vi du: `Kp = 20.0 N.m/rad` (`kp_reg = 20000`) -> `s_imp_kp = 4908` (`4.908 Iq/rotor_count`).

3. **Quy doi `Kd` (`N.m.s/rad` tai truc ra -> `Iq_count / rotor_RPM`):**
   ```text
   Kd_rotor = Kd_SI * (300 / 71.6197) = Kd_SI * 4.18879 ≈ (Kd_SI * 4189) / 1000
   ```
   - Vi du: `Kd = 0.036 N.m.s/rad` (`kd_reg = 36`) -> `s_imp_kd = 151` (`0.151 Iq/rotor_RPM`).
   - Vi du: `Kd = 0.060 N.m.s/rad` (`kd_reg = 60`) -> `s_imp_kd = 251` (`0.251 Iq/rotor_RPM`).

4. **Quy doi `tau_ff` (`mN.m` tai truc ra -> `Iq_count`):**
   ```text
   tau_iq = (tau_reg_mNm * 300) / 1000
   ```
   - Vi du: `tau_ff = 0.500 N.m` (`tau_reg = 500`) -> `s_imp_tau_ff = 150 Iq_counts`.

---

## 4. BANG THONG SO CHUAN CHO EC4012_HIP (DON VI SI TRUC RA CUA MASTER)

`DEFAULT_KP = 10000` (`10.000 N.m/rad`), `DEFAULT_KD = 36` (`0.036 N.m.s/rad`):

| Che do (Preset) | **Kp (`N.m/rad`)** | `kp_reg` (`0x1A`) | **Kd (`N.m.s/rad`)** | `kd_reg` (`0x1C`) | Tuong duong tai rotor (`Kp_r / Kd_r`) | Dac tinh thuc te |
|---|---|---|---|---|---|---|
| **Free (Tha long)** | **0.0** | `0` | **0.000** | `0` | `0.00 / 0.00` | Truc quay tu do |
| **Damping only** | **0.0** | `0` | **0.025** | `25` | `0.00 / 0.10` | Can nhot van toc muot |
| **Soft (Khop mem)** | **4.0** | `4000` | **0.015** | `15` | `0.98 / 0.06` | Mem deo, dan hoi, ve dich dung em ngay |
| **Medium (Vua phai)**| **6.0** | `6000` | **0.022** | `22` | `1.47 / 0.09` | Giu vi tri em ai, khong vot lo |
| **Stiff (Mac dinh)** | **10.0** | `10000` | **0.036** | `36` | `2.45 / 0.15` | **Bam goc cung chac, phanh dut khoat, 0 rung** |
| **High Stiff (Chiu tai)**| **14.0** | `14000` | **0.048** | `48` | `3.44 / 0.20` | Giu khop vung duoi tai nang |
| **Rigid (`Kp_max`)** | **20.0** | `20000` | **0.060** | `60` | `4.91 / 0.25` | Do cung cuc dai theo dong dinh muc (`1 N.m`) |

> **Luu y:** Khi dat `Kp >= 8.0 N.m/rad`, **khong giam `Kd` xuong duoi `0.015 N.m.s/rad`** (`kd_reg < 15`) de tranh thieu giam chan (underdamped oscillation) khi ve dich. Nguong tran `Kd_max` an toan khong gay rung nhieu luong tu la **`0.120 N.m.s/rad`** (`kd_reg = 120`).
