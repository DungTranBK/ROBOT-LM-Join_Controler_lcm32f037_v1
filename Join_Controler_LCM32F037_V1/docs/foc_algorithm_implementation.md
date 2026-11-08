# Tài Liệu Kỹ Thuật: Triển Khai Thuật Toán Điều Khiển FOC (Field Oriented Control)

Tài liệu này mô tả chi tiết kiến trúc, nguyên lý toán học, luồng thực thi thời gian thực và các kỹ thuật tối ưu hóa trong việc triển khai thuật toán **Điều khiển Định hướng Từ trường (FOC - Field Oriented Control)** cho động cơ BLDC EC4012 trên vi điều khiển **LCM32F037** (ARM Cortex-M0 @ 96 MHz).

---

## 1. Tổng Quan Kiến Trúc Điều Khiển FOC

Hệ thống FOC trong firmware được tổ chức theo mô hình phân tầng nghiêm ngặt:
- **Tầng Ứng Dụng (App - [app_isr.c](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/app/app_isr.c))**: Tiếp nhận ngắt đồng bộ phần cứng từ Timer TIM1 và phân phối luồng.
- **Tầng Trung Gian (Middleware - [motor_ctrl.c](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/mid/motor_ctrl.c), [foc_math.c](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/mid/foc_math.c), [trig_table.c](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/mid/trig_table.c))**: Chứa toàn bộ logic toán học FOC, các bộ điều khiển vòng lặp (vị trí, tốc độ, dòng điện), bộ lập kế hoạch vị trí và máy trạng thái hiệu chuẩn. Hoàn toàn độc lập với thanh ghi phần cứng.
- **Tầng Phần Cứng (BSP - [bsp_pwm.c](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/bsp/bsp_pwm.c), [bsp_adc.c](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/bsp/bsp_adc.c), [bsp_mt6701.c](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/bsp/bsp_mt6701.c))**: Giao tiếp trực tiếp với Timer TIM1, ADC1 DMA, và cảm biến góc từ MT6701 qua SPI0.

```
       ┌────────────────────────────────────────────────────────┐
       │                 TIM1 CC4 Interrupt (20 kHz)            │
       └───────────────────────────┬────────────────────────────┘
                                   │
                                   ▼
 ┌─────────────────────────────────────────────────────────────────────────────┐
 │                         TIM1_CC_Handler (app_isr.c)                         │
 │  1. bsp_adc_start_conversion()                                              │
 │  2. raw_angle = bsp_mt6701_read_raw() (SPI0 14-bit)                         │
 │  3. motor_ctrl_fast_loop(raw_angle)                                         │
 │  4. bsp_pwm_clear_cc4_flag()                                                │
 └─────────────────────────────────┬───────────────────────────────────────────┘
                                   │
                                   ▼
 ┌─────────────────────────────────────────────────────────────────────────────┐
 │                        motor_ctrl_fast_loop()                               │
 │                                                                             │
 │  ┌───────────────────────────────────────────────────────────────────────┐  │
 │  │ 1. angle_speed():                                                     │  │
 │  │    - Chuyển raw 14-bit -> Vị trí 10-bit & Đa vòng (s_multiturn)       │  │
 │  │    - Tích lũy delta 10 chu kỳ -> Tính tốc độ thực tế (RPM)            │  │
 │  └──────────────────────────────────┬────────────────────────────────────┘  │
 │                                     ▼                                       │
 │  ┌───────────────────────────────────────────────────────────────────────┐  │
 │  │ 2. foc_step():                                                        │  │
 │  │    a) Tính góc điện theta_e = (raw - offset) * 10 * 3834 / 1000       │  │
 │  │    b) Tra bảng trig_sin, trig_cos (định dạng Q-10000)                 │  │
 │  │    c) Đọc dòng ADC pha U, V -> Khôi phục Iw = -(Iu + Iv)              │  │
 │  │    d) foc_clarke_park() -> id, iq                                     │  │
 │  │    e) Vòng điều khiển phân tầng:                                      │  │
 │  │       - Position Mode: pos_mode_plan() -> Vận tốc / Giữ vị trí (PD)   │  │
 │  │       - Speed Mode: PI Tốc độ -> iq_ref                               │  │
 │  │       - Current Loop: PI Id (-> 0) & PI Iq -> Vd, Vq                  │  │
 │  │    f) foc_inv_park_svpwm() -> Duty chu kỳ 3 pha (U, V, W)             │  │
 │  │    g) bsp_pwm_set_compare() -> Nạp thanh ghi CCR1, CCR2, CCR3 TIM1    │  │
 │  └───────────────────────────────────────────────────────────────────────┘  │
 └─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Chu Trình Thời Gian Thực & Định Thời Phần Cứng

### 2.1. Cấu hình Timer TIM1 và tần số PWM
- **Xung nhịp Timer**: $96\text{ MHz} / (\text{Prescaler } 1 + 1) = 48\text{ MHz}$.
- **Chế độ đếm**: Center-aligned mode 2 (`TIM_CounterMode_CenterAligned2`).
- **Chu kỳ đếm (`FOC_PWM_ARR`)**: $1199$ ($0x04AF$).
- **Tần số sóng mang PWM**:
  $$f_{PWM} = \frac{48\text{ MHz}}{2 \times 1200} = 20\text{ kHz} \quad (T = 50\ \mu\text{s})$$
- **Thời gian chết phần cứng (Dead-time)**: $0x2F$ ($47 \times \frac{1}{96\text{ MHz}} \approx 490\text{ ns}$).

### 2.2. Thời điểm ngắt và đồng bộ lấy mẫu ADC
- Kênh so sánh **CC4** được đặt tại giá trị:
  $$\text{CC4} = \text{FOC_PWM_ARR} - 0x46 = 1199 - 70 = 1129$$
- Khi counter đếm tiến đến $1129$ (ngay trước đỉnh chu kỳ PWM, thời điểm các khóa bán dẫn phía dưới Low-side đang dẫn ổn định), ngắt `TIM1_CC_Handler` được kích hoạt.
- Tại đầu ngắt, hàm `bsp_adc_start_conversion()` phát lệnh kích đổi ADC để đo dòng pha qua Shunt Low-side mà không bị ảnh hưởng bởi nhiễu chuyển mạch (switching transients).

---

## 3. Cảm Biến Góc & Tính Toán Tốc Độ (`angle_speed`)

### 3.1. Đọc và lọc góc rotor
Động cơ sử dụng cảm biến góc từ trường tuyệt đối **MT6701** giao tiếp qua **SPI0 Master 16-bit** với tốc độ cao:
- Dữ liệu thô thu được có độ phân giải 14-bit: $s\_raw\_angle \in [0, 16383]$.
- Vị trí cơ bản 10-bit cho bảng điều khiển (Control Table):
  $$\text{s\_pos10} = s\_raw\_angle \gg 4 \quad (0 \dots 1023)$$

### 3.2. Theo dõi vị trí đa vòng (Multi-turn Tracking)
Để phục vụ điều khiển vị trí vượt qua giới hạn 1 vòng ($360^\circ$), giải thuật kiểm tra bước nhảy qua điểm $0$:
- Phát hiện nhảy vòng 10-bit:
  - Nếu $\Delta pos > 512$: $f\_turns = f\_turns - 1$
  - Nếu $\Delta pos < -512$: $f\_turns = f\_turns + 1$
  - Vị trí đa vòng: $s\_multiturn = (f\_turns \ll 10) + \text{s\_pos10}$
- Tương tự với bước nhảy 14-bit để đếm số vòng cơ khí $s\_rev\_cnt$ (ngưỡng phát hiện wrap-around là $12000$).

### 3.3. Thuật toán đo tốc độ quay (RPM Calculation)
Tốc độ được tính thông qua việc tích lũy sai phân góc 14-bit sau mỗi $10$ chu kỳ ngắt FOC (mỗi $10 \times 50\ \mu\text{s} = 500\ \mu\text{s} = 0.5\text{ ms}$):
$$\text{Speed (RPM)} = f\_delta\_acc \times 7.324219226837158$$

**Chứng minh hằng số quy đổi:**
- Tần số FOC: $20.000\text{ Hz}$.
- Số lần lấy mẫu tích lũy: $10$ lần $\implies$ Tần số tính tốc độ: $2.000\text{ Hz}$.
- Encoder 14-bit: $16.384\text{ xung/vòng}$.
$$\text{Hệ số} = \frac{2000\text{ chu kỳ/s} \times 60\text{ s/phút}}{16384\text{ xung/vòng}} = \frac{120000}{16384} \approx 7.32421875\text{ RPM/count}$$
Độ phân giải đo tốc độ là **$7.32\text{ RPM}$**.

---

## 4. Mô Hình Toán Học FOC (Fixed-Point Q-Format)

Do vi điều khiển Cortex-M0 không có FPU phần cứng và không có bộ chia phần cứng, toàn bộ các phép tính lượng giác và biến đổi hệ tọa độ được chuẩn hóa sang dạng **số nguyên cố định (Fixed-point)** với hệ số tỉ lệ $10.000$.

### 4.1. Góc điện và bảng lượng giác (`trig_table`)
Góc điện $\theta_e$ được tính từ góc thô cơ khí và góc lệch hiệu chuẩn ($s\_angle\_off$):
$$x = (s\_raw\_angle - s\_angle\_off) \times 10 \pmod{16384}$$
$$\theta_e = \frac{x \times 3834}{1000} \pmod{62831}$$
Trong đó:
- $62831 \approx 2\pi \times 10000$ (tương ứng $360^\circ$ điện).
- Tỉ số $3834 / 1000$ quy đổi chính xác từ thang đo 14-bit ($16384$) sang thang đo $2\pi \times 10000$ theo số cặp cực của động cơ ($16384 \times 3.834 \approx 62816$).

Hàm `trig_sin` và `trig_cos` tra cứu bảng sin 128 phần tử với kỹ thuật chia tối ưu không dùng lệnh chia phần mềm:
```c
static inline uint32_t div123(uint32_t x) {
    return (x * 17051UL) >> 21;  /* Tương đương x / 123 với 0 <= x < 16384 */
}
```

### 4.2. Lấy mẫu dòng điện pha
Dòng điện pha được đo từ 2 bộ khuếch đại thuật toán OPA tích hợp bên trong MCU (OPA2 cho Pha U, OPA1 cho Pha V) với độ lợi $Gain = 20\text{x}$:
- Giá trị ADC 12-bit dịch phải 2 bit về 10-bit:
  $$i_u = (\text{ADC}[0] \gg 2) - s\_off\_u$$
  $$i_v = (\text{ADC}[1] \gg 2) - s\_off\_v$$
- Dòng điện pha W được khôi phục theo định luật Kirchhoff:
  $$i_w = -(i_u + i_v)$$

### 4.3. Biến đổi Clarke - Park kết hợp (`foc_clarke_park`)
Firmware gộp hai phép biến đổi Clarke ($3 \to 2$ pha tĩnh) và Park ($2$ pha tĩnh $\to$ hệ quy chiếu quay $d-q$) thành một khối tính toán trực tiếp để giảm thiểu phép toán trung gian:
- Với $c = \cos(\theta_e) \times 10000$ và $s = \sin(\theta_e) \times 10000$:
- Hằng số $11547 \approx 10000 \times \frac{2}{\sqrt{3}}$:
$$t_1 = \frac{c \cdot i_w}{10000}, \quad s_{iu} = s \cdot i_u, \quad s_{iv} = s \cdot i_v$$
$$x = t_1 + \frac{s_{iu} - s_{iv}}{11547}$$
$$t_3 = \frac{c \cdot i_v + c \cdot i_u}{20000}$$
$$i_d = \frac{2}{3} (x - t_3)$$
$$y = \frac{-s \cdot i_w}{10000} + \frac{c \cdot i_u - c \cdot i_v}{11547}$$
$$q_3 = \frac{s_{iv} + s_{iu}}{20000}$$
$$i_q = \frac{2}{3} (y + q_3)$$

Kết quả thu được là vector dòng điện $i_d$ (dòng từ hóa) và $i_q$ (dòng tạo mô-men).

---

## 5. Cấu Trúc Các Vòng Lặp Điều Khiển (Control Loops)

Firmware hỗ trợ 3 chế độ hoạt động chính được chọn qua thanh ghi `velocity_mode_control`:

```
 [Chế độ 2: Position Mode]
  Goal Position ──►[pos_mode_plan()] ──(xa mục tiêu: v = 1.5*err)──┐
                          │                                         │
                          ▼ (gần mục tiêu <= 3 counts)             │
                   [PD Position Hold]                               │
                   (Iq = err*2.5 - spd*0.05)                        ▼
                          │                              [Chế độ 1: Speed Mode]
                          │                               Target Speed ──► [PI Speed]
                          │                                                    │
                          ├───────────────── iq_ref ◄──────────────────────────┘
                          │
                          ▼
             [Vòng Dòng Điện - Current Loop]
              id_ref = 0, iq_ref
              ├──► [PI Id] ──► Vd ──┐
              └──► [PI Iq] ──► Vq ──┼──► [Inverse Park + SVPWM] ──► PWM CCR1..3
                                    │
 [Chế độ 0: Open Loop]             │
  Moving Speed ─────────► Vq ───────┘ (Vd = 0)
```

### 5.1. Chế độ 0: Vòng hở (Open Loop - `VEL_MODE_OPEN_LOOP`)
- Đặt trực tiếp điện áp $V_q = \text{s\_moving\_speed\_cmd}$, $V_d = 0$.
- Kẹp giá trị $V_q \in [-1199, 1199]$ (bằng `FOC_PWM_ARR`).

### 5.2. Chế độ 1: Điều khiển Vận tốc vòng kín (Closed-Loop Speed - `VEL_MODE_CLOSED_LOOP`)
Sử dụng bộ điều khiển PI vận tốc để tính dòng mục tiêu $i_{q\_ref}$:
$$e_v = \text{speed\_cmd} - \text{speed}$$
$$integ_v = \text{clamp}(integ_v + e_v, \pm 1000000)$$
$$i_{q\_ref} = \frac{integ_v}{3000} + \frac{e_v}{2}$$
$$i_{q\_ref} = \text{clamp}(i_{q\_ref}, \pm 300)$$
- Độ lợi: $K_{p\_spd} = 0.5$, $K_{i\_spd} = \frac{1}{3000}$.
- Giới hạn dòng điện $i_{q\_ref}$ bão hòa ở $\pm 300$.

### 5.3. Chế độ 2: Điều khiển Vị trí (Joint Mode - `VEL_MODE_POSITION`)
Bộ lập kế hoạch `pos_mode_plan()` điều phối chuyển động:
1. **Kiểm tra giới hạn cơ khí**: So khớp `goal_position` với `min_angle_limit` và `max_angle_limit`. Nếu vi phạm, kích hoạt cờ lỗi `ERR_FLAG_ANGLE`.
2. **Tối ưu hóa đường đi ngắn nhất**: Khi cấu hình đủ 1 vòng tròn ($0 \dots 1023$), tự động chọn hướng quay ngắn nhất ($|err| > 512 \implies err \pm 1024$).
3. **Cơ chế chuyển vùng (Hysteresis Deadband)**:
   - Khi $|err| \le 3$ counts: Kích hoạt chế độ giữ vị trí (`s_pos_hold = 1`).
   - Khi $|err| > 6$ counts: Thoát chế độ giữ vị trí (`s_pos_hold = 0`).
4. **Hành vi khi di chuyển xa mục tiêu**:
   - Sinh vận tốc tham chiếu theo hàm tỷ lệ: $v = 1.5 \times |err|$ RPM.
   - Kẹp vận tốc theo giới hạn `moving_speed` (mặc định 200 RPM, tối đa 1000 RPM, tối thiểu 8 RPM).
   - Đưa lệnh vận tốc này vào vòng kín điều khiển tốc độ (PI Speed).
5. **Hành vi khi giữ vị trí (`s_pos_hold = 1`)**:
   - Sử dụng bộ điều khiển PD trực tiếp tính $i_{q\_ref}$:
     $$i_{q\_ref} = (f\_pos\_target - s\_multiturn) \times 2.5 - \text{speed} \times 0.05$$
   - Giúp triệt tiêu sai số xác lập và chống rung lắc khi dừng tại vị trí mục tiêu.

### 5.4. Vòng điều khiển Dòng điện lõi (Current Loop)
Hoạt động ở tần số $20\text{ kHz}$ bên trong `foc_step()`:
- Đặt $i_{d\_ref} = 0$ (chiến lược $i_d = 0$ tối ưu mô-men cho động cơ BLDC nam châm mặt SPM).
- Sai số dòng điện:
  $$e_d = -i_d, \quad e_q = i_{q\_ref} - i_q$$
- Khâu tích phân (Anti-windup với ngưỡng $\pm 129999$):
  $$integ_d = \text{clamp}(integ_d + 30 \times e_d, \pm 129999)$$
  $$integ_q = \text{clamp}(integ_q + 30 \times e_q, \pm 129999)$$
- Điện áp ngõ ra $V_d, V_q$:
  $$V_d = \frac{401 \times e_d + integ_d}{100}$$
  $$V_q = \frac{401 \times e_q + integ_q}{100}$$
  - Tương đương $K_p = 4.01$, $K_i = 0.3$ trên mỗi chu kỳ $50\ \mu\text{s}$.
- Kẹp điện áp $V_d, V_q \in [-1199, 1199]$ (bằng `FOC_PWM_ARR`).

---

## 6. Biến Đổi Nghịch Park & Điều Chế Vector Không Gian (SVPWM)

Hàm `foc_inv_park_svpwm()` thực hiện biến đổi từ hệ tọa độ quay $(V_d, V_q)$ sang duty cycle 3 pha:

### 6.1. Biến đổi nghịch Park
$$\alpha = \frac{c \cdot V_d - s \cdot V_q}{10000}$$
$$b_{num} = s \cdot V_d + c \cdot V_q$$
$$b_{div} = \frac{b_{num}}{11547}, \quad a_{div} = \frac{\alpha}{2}$$
$$V_b = b_{div} - a_{div}$$
$$V_c = -b_{div} - a_{div}$$

### 6.2. Dịch trung điểm (Min-Max Mid-point / SVPWM Centering)
Kỹ thuật tiêm sóng hài thứ ba (Zero-sequence voltage injection) được thực hiện đơn giản bằng thuật toán tìm trung điểm cực trị:
$$V_{min} = \min(\alpha, V_b, V_c)$$
$$V_{max} = \max(\alpha, V_b, V_c)$$
$$V_{mid} = \frac{V_{min} + V_{max}}{2}$$

Điện áp điều chế 3 pha:
$$duty_U = \alpha - V_{mid} + \text{FOC_PWM_DUTY_MID} \quad (599)$$
$$duty_V = V_b - V_{mid} + \text{FOC_PWM_DUTY_MID}$$
$$duty_W = V_c - V_{mid} + \text{FOC_PWM_DUTY_MID}$$

Các giá trị được kẹp trong ngưỡng an toàn: $[\text{FOC_PWM_DUTY_MIN}, \text{FOC_PWM_ARR}] = [70, 1199]$.

### 6.3. Nạp thanh ghi phần cứng Timer TIM1
Do cấu hình mạch kích và cực tính phần cứng, giá trị so sánh của Timer được đảo ngược:
$$\text{TIM1->CCR1} = \text{FOC_PWM_ARR} - duty_U$$
$$\text{TIM1->CCR2} = \text{FOC_PWM_ARR} - duty_V$$
$$\text{TIM1->CCR3} = \text{FOC_PWM_ARR} - duty_W$$

---

## 7. Máy Trạng Thái Tự Động Hiệu Chuẩn (Calibration State Machine)

Để FOC hoạt động chính xác, hệ thống triển khai máy trạng thái hiệu chuẩn 3 giai đoạn tự động khi nhận lệnh `motor_ctrl_start_calibration()`:

```
  [Lệnh Calib] ──► CAL_STATE_ZERO_TOTAL (Bước 1: Offset Shunt tổng)
                          │  (PWM = 0, bỏ qua 3200 ticks, lấy trung bình 1024 mẫu)
                          ▼
                   CAL_STATE_ZERO_PHASE (Bước 2: Offset Shunt pha U, V)
                          │  (Duty nhỏ 0x64, bỏ qua 3200 ticks, lấy trung bình 1024 mẫu)
                          ▼
                   CAL_STATE_ALIGN      (Bước 3: Khóa góc rotor & tìm Offset)
                          │  (Vq = 0, Ramp Vd lên 400 trong 1.5s, đọc raw_angle -> angle_offset)
                          ▼
                   CAL_STATE_IDLE       (Hoàn tất, lưu Flash EEPROM, calib_done = 1)
```

| Trạng thái | Tên trạng thái | Điều kiện PWM | Hành động | Mục đích |
| :---: | :--- | :---: | :--- | :--- |
| **1** | `CAL_STATE_ZERO_TOTAL` | Tắt PWM ($0, 0, 0$) | Đợi $3200$ ticks ổn định; lấy trung bình $1024$ mẫu ADC Shunt tổng. | Xác định điểm 0 tĩnh của Shunt dòng tổng (`offset_total`). |
| **2** | `CAL_STATE_ZERO_PHASE` | Duty thấp ($0x64$) | Đợi $3200$ ticks ổn định; lấy trung bình $1024$ mẫu pha U và V. | Xác định điểm 0 tĩnh của Shunt pha U và V (`offset_u`, `offset_v`). |
| **3** | `CAL_STATE_ALIGN` | Cố định góc $\theta_e = 0$, $V_q = 0$ | Tăng dần điện áp $V_d$ từ $0$ lên $400$ ($+0.04$ mỗi tick) trong $30.000$ ticks ($1.5\text{ s}$). | Rotor tự động xoay và khóa chặt vào trục D. Giá trị $s\_raw\_angle$ lúc này được gán làm `angle_offset`. |
| **0** | `CAL_STATE_IDLE` | PWM bình thường | Bật `calib_done = 1`, đánh dấu `g_eeprom_dirty_flag = 1`. | Hệ thống sẵn sàng chạy FOC vòng kín; các offset được lưu vĩnh viễn vào Flash. |

---

## 8. Tóm Tắt Các Hằng Số & Tham Số Cấu Hình

| Hằng số | Giá trị | Ý nghĩa |
| :--- | :---: | :--- |
| `FOC_PWM_ARR` | `1199` (`0x04AF`) | Giá trị chu kỳ Timer TIM1 (48 MHz / 2400 = 20 kHz PWM). |
| `FOC_PWM_DEAD_TIME` | `0x2F` (47) | Thời gian chết phần cứng ~490 ns. |
| `FOC_PWM_ADC_TRIG` | `1129` (`ARR - 70`) | Điểm kích ngắt CC4 để lấy mẫu ADC trước đỉnh đếm. |
| `FOC_PWM_DUTY_MIN` | `70` (`0x46`) | Giới hạn duty cycle nhỏ nhất để bảo đảm bootstrap nạp tụ. |
| `FOC_PWM_DUTY_MID` | `599` (`0x257`) | Điểm dịch mức trung tâm SVPWM ($ARR / 2$). |
| `CURRENT_KI_GAIN` | `30` | Hệ số tích phân vòng dòng điện ($K_i = 0.3$). |
| `CURRENT_INTEG_LIMIT` | `129999` | Giới hạn chống bão hòa tích phân khâu dòng điện. |
| `VEL_KI_DIV` | `3000` | Hệ số chia tích phân vòng tốc độ. |
| `POS_HOLD_KP` | `2.5f` | Hệ số tỷ lệ giữ vị trí ($K_p$). |
| `POS_HOLD_KD` | `0.05f` | Hệ số vi phân / cản vận tốc giữ vị trí ($K_d$). |
| `IQ_LIMIT` | `300` | Giới hạn dòng mô-men cực đại cho phép. |

---

## 9. Tham Khảo Mã Nguồn Liên Quan

- Khởi tạo phần cứng PWM: [`bsp_pwm_init()`](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/bsp/bsp_pwm.c#L13-L54)
- Trình phục vụ ngắt 20 kHz: [`TIM1_CC_Handler()`](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/app/app_isr.c#L16-L20)
- Vòng lặp FOC nhanh: [`motor_ctrl_fast_loop()`](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/mid/motor_ctrl.c#L428-L432)
- Biến đổi Clarke - Park: [`foc_clarke_park()`](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/mid/foc_math.c#L16-L33)
- Biến đổi Nghịch Park & SVPWM: [`foc_inv_park_svpwm()`](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/mid/foc_math.c#L36-L54)
- Bảng Sin/Cos Fixed-Point: [`trig_sin()`](file:///d:/Lumi/LM_2026/ROBOT/.src/EC4012/LCM32F037_BLDC_Keil/Source/mid/trig_table.c#L37-L52)
