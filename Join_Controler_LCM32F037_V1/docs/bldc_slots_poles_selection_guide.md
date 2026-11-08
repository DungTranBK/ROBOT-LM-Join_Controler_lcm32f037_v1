# CẨM NANG THIẾT KẾ & LỰA CHỌN CẤU HÌNH ĐỘNG CƠ BLDC: SỐ CỰC (POLES) VÀ SỐ RÃNH (SLOTS)
## Hướng Dẫn Kỹ Thuật: Nguyên Lý Phối Hợp, Tác Động Cơ - Điện Và Tối Ưu Hóa Thuật Toán FOC

> **Định dạng tài liệu**: Soạn thảo hoàn toàn bằng văn bản thuần túy và bảng biểu trực quan, **không sử dụng mã LaTeX phức tạp** để tránh lỗi hiển thị trên các trình đọc Markdown và IDE.

---

## MỤC LỤC
1. [Khái Niệm Cốt Lõi: Slots, Poles Và Pole Pairs](#1-khái-niệm-cốt-lõi-slots-poles-và-pole-pairs)
2. [Tác Động Của Số Cặp Cực (P) Đến Hệ Thống](#2-tác-động-của-số-cặp-cực-p-đến-hệ-thống)
3. [Tác Động Của Số Rãnh Cuộn Dây Stator (Slots - S)](#3-tác-động-của-số-rãnh-cuộn-dây-stator-slots---s)
4. [Nguyên Tắc Phối Hợp Kinh Điển Giữa Slots Và Poles](#4-nguyên-tắc-phối-hợp-kinh-điển-giữa-slots-và-poles)
5. [Quy Trình 5 Bước Lựa Chọn Cấu Hình Cho Dự Án](#5-quy-trình-5-bước-lựa-chọn-cấu-hình-cho-dự-án)
6. [Bảng Tra Cứu Các Tổ Hợp Slots / Poles Công Nghiệp Phổ Biến](#6-bảng-tra-cứu-các-tổ-hợp-slots--poles-công-nghiệp-phổ-biến)
7. [Phân Tích Cấu Hình Động Cơ EC4012 (P = 10) Trong Dự Án](#7-phân-tích-cấu-hình-động-cơ-ec4012-p--10-trong-dự-án)

---

## 1. Khái Niệm Cốt Lõi: Slots, Poles Và Pole Pairs

Trong một động cơ không chổi than (BLDC) 3 pha:

```text
                  [STATOR CỐ ĐỊNH]             [ROTOR QUAY]
                  Số Rãnh Cuộn Dây             Số Cực Nam Châm
                   Slots (Ký hiệu: S)          Poles (Ký hiệu: 2P)
                         │                           │
                         │                     Số Cặp Cực Nam Châm
                         │                     Pole Pairs (Ký hiệu: P)
                         ▼                           ▼
                 Ví dụ: 12 Rãnh cuộn         Ví dụ: 14 Cực = 7 Cặp cực
                        (12S hoặc 12N)              (14P hay P = 7)
```

1. **Slots (S hoặc N - Rãnh Stator):**
   - Là các răng thép kỹ thuật điện trên Stator dùng để quấn các cuộn dây đồng.
   - Vì là động cơ 3 pha (U, V, W), **tổng số rãnh S luôn luôn phải chia hết cho 3** (mỗi pha chiếm `S / 3` rãnh).
   - Ví dụ: 3, 6, 9, 12, 18, 24, 36 rãnh...

2. **Poles (Ký hiệu 2P - Số cực nam châm Rotor):**
   - Là tổng số các thỏi nam châm vĩnh cửu (hoặc các vùng từ hóa Bắc/Nam) dán trên Rotor.
   - **Số cực luôn luôn là số chẵn** vì nam châm luôn đi theo từng cặp Bắc (N) và Nam (S).
   - Ví dụ: 2, 4, 6, 8, 10, 14, 20 cực...

3. **Pole Pairs (Ký hiệu P - Số CẶP cực):**
   - `P = Số cực / 2`
   - Ví dụ: Động cơ có 20 cực nam châm thì có `P = 10 cặp cực`. Đây chính là con số xuất hiện trực tiếp trong code thuật toán FOC.

### Hình ảnh trực quan so sánh 2 triết lý thiết kế động cơ:

![So sánh thiết kế động cơ BLDC ít cực tốc độ cao vs nhiều cực mô-men xoắn lớn](images/slots_poles_guide.jpg)

- **Bên Trái (A - High-Speed Low-Pole):** Ít rãnh Stator, ít cực nam châm Rotor (P nhỏ). Rãnh dây rộng chứa cuộn dây đồng cỡ lớn. Tối ưu cho **tốc độ quay siêu cao (hàng chục nghìn RPM)**, mô-men xoắn nhỏ.
- **Bên Phải (B - High-Torque Multi-Pole):** Thiết kế dạng đĩa dẹt (Pancake) như dòng **EC4012**. Rất nhiều răng Stator và nhiều cực nam châm (P lớn, ví dụ P = 10). Tối ưu cho **mật độ mô-men xoắn cực đại ở tốc độ thấp, kiểm soát vị trí chính xác trong Robot và khớp truyền động trực tiếp (Direct Drive)**.

---

## 2. Tác Động Của Số Cặp Cực (P) Đến Hệ Thống

Số cặp cực `P` là thông số quan trọng nhất quyết định "tính cách" của động cơ:

### 2.1. Đánh đổi giữa Lực kéo (Mô-men xoắn) và Tốc độ tối đa
Mô-men xoắn điện từ của động cơ tỷ lệ thuận với số cặp cực:
```text
Mô-men xoắn (Torque) = (3 / 2) * P * Từ_thông * Iq
```
- **Khi P lớn (Nhiều cặp cực, ví dụ P = 7, 10, 14):**
  - **Lực kéo cực kỳ khỏe ở tốc độ thấp.** Giống như xe gài số 1, động cơ có thể truyền động trực tiếp (Direct Drive) kéo bánh xe, khớp robot mà không cần hộp số giảm tốc.
  - **Độ êm ái vượt trội:** Mỗi bước chuyển giữa các cực từ rất nhỏ, triệt tiêu gần như hoàn toàn hiện tượng rung giật khi quay chậm.
  - **Nhược điểm:** Tốc độ tối đa thấp vì sức phản điện động (Back-EMF) tăng rất nhanh theo tốc độ, nhanh chóng triệt tiêu điện áp cấp từ nguồn pin.
- **Khi P nhỏ (Ít cặp cực, ví dụ P = 1, 2):**
  - **Tốc độ quay cực cao:** Có thể đạt từ 20.000 đến 100.000 vòng/phút.
  - **Nhược điểm:** Lực kéo yếu ở dải tốc độ thấp, dễ giật bước khi chạy chậm.

### 2.2. Quan hệ giữa Tần số điện và Tốc độ cơ
```text
Tần số dòng điện xoay chiều f (Hz) = (P * Tốc độ cơ RPM) / 60
```
- Động cơ quay được 1 vòng cơ học ngoài đời thì dòng điện 3 pha phải trải qua đúng `P` chu kỳ sóng sin.
- **Tác động lên Vi điều khiển (MCU):**
  - Giả sử động cơ quay ở tốc độ 3.000 RPM:
    - Nếu `P = 1`: Tần số điện chỉ là `(1 * 3000) / 60 = 50 Hz`. MCU điều khiển rất nhàn nhã.
    - Nếu `P = 10`: Tần số điện vọt lên `(10 * 3000) / 60 = 500 Hz`!
  - Để bộ điều khiển FOC bám kịp sóng sin 500 Hz với độ chính xác cao (ít nhất 20 đến 40 mẫu cho mỗi chu kỳ sóng sin), **MCU bắt buộc phải chạy ngắt FOC ở tần số rất cao (từ 10 kHz đến 20 kHz)**.

### 2.3. Yêu cầu độ phân giải của Cảm biến góc (Encoder)
```text
Góc cơ ngoài đời = Góc điện trong MCU / P
```
- Với `P = 10`: Nếu cảm biến góc đo sai lệch chỉ **1 độ cơ**, góc điện bên trong MCU sẽ bị lệch tới **10 độ điện**.
- Góc điện bị lệch 10° sẽ làm giảm lực kéo động cơ và khiến động cơ bị nóng do xuất hiện dòng `Id` vô công.
- Vì vậy, **động cơ có P càng lớn thì bắt buộc Encoder phải có độ phân giải càng cao** (12-bit đến 14-bit, tức 4.096 đến 16.384 xung/vòng).

---

## 3. Tác Động Của Số Rãnh Cuộn Dây Stator (Slots - S)

Số rãnh `S` trên Stator quyết định không gian chứa dây đồng và phân bố từ trường:

### 3.1. Kích thước rãnh và Nội trở cuộn dây (R)
- **Nhiều rãnh (S lớn, ví dụ 24, 36 rãnh):**
  - Mỗi rãnh sẽ có kích thước hẹp lại. Việc quấn dây bằng máy sẽ khó khăn hơn.
  - Tuy nhiên, từ trường sinh ra quanh chu vi vòng tròn sẽ cực kỳ đều đặn, bước sóng từ trường tiệm cận sóng sin chuẩn, giúp động cơ chạy cực êm và không có tiếng rít.
- **Ít rãnh (S nhỏ, ví dụ 3, 6, 9, 12 rãnh):**
  - Rãnh to và rộng, chứa được dây đồng đường kính lớn hoặc quấn được nhiều vòng dây $\implies$ dễ chế tạo, dòng chịu tải lớn, tản nhiệt tốt.

### 3.2. Tổn hao sắt (Iron Losses)
- Răng stator càng nhiều và tần số đổi chiều từ trường càng cao thì tổn hao do dòng điện xoáy (Foucault) và từ trễ (Hysteresis) trong lõi thép Stator càng lớn, làm Stator bị nóng lên khi quay ở tốc độ cao.

---

## 4. Nguyên Tắc Phối Hợp Kinh Điển Giữa Slots Và Poles

Không thể chọn bừa số rãnh `S` và số cực `P`. Chúng phải tuân theo các quy luật điện từ nghiêm ngặt:

### 4.1. Quy tắc cấm tuyệt đối: Không bao giờ chọn S = 2P
- Ví dụ: 12 rãnh và 12 cực nam châm (12S12P).
- **Lý do:** Khi `S = 2P`, tất cả các cực nam châm trên Rotor đều đồng thời đối diện trực diện với các răng của Stator. Lực hút từ tính tại tất cả các răng sẽ cộng hưởng lại cực mạnh, tạo ra hiện tượng **Khóa từ (Magnetic Locking)**.
- Động cơ sẽ bị kẹt cứng (Cogging Torque cực lớn), tay vặn rất nặng và vi điều khiển không thể nào khởi động động cơ êm ái được.

### 4.2. Hệ số quấn dây (Winding Factor - Kw)
- Trong thiết kế động cơ, tỷ số giữa `S` và `P` quyết định **Hệ số quấn dây `Kw`** (thường từ 0 đến 1).
- `Kw` thể hiện hiệu suất chuyển hóa dòng điện chạy qua cuộn dây thành từ trường hữu ích.
- **Tiêu chuẩn thiết kế:** Luôn chọn tổ hợp sao cho `Kw >= 0.86` (tốt nhất là từ `0.93` đến `0.96`).

### 4.3. Quy tắc chọn chênh lệch giữa Slots và Poles
Thông thường trong công nghiệp, người ta chọn số cực và số rãnh lệch nhau 2 hoặc 4 đơn vị:
```text
Poles = Slots ± 2   (Ví dụ: 12S10P, 12S14P)
Poles = Slots ± 4   (Ví dụ: 24S20P, 24S28P)
```
Tổ hợp này giúp:
- Triệt tiêu gần như hoàn toàn mô-men cản Cogging (động cơ quay trơn tru như có ổ bi).
- Đạt hệ số quấn dây `Kw` cao nhất (thường đạt 0.93 - 0.95).

---

## 5. Quy Trình 5 Bước Lựa Chọn Cấu Hình Cho Dự Án

Khi bắt đầu thiết kế hoặc chọn mua động cơ BLDC cho một hệ thống, hãy thực hiện theo 5 bước sau:

```text
 ┌─────────────────────────────────────────────────────────────┐
 │ Bước 1: Xác định yêu cầu Tải (Tốc độ RPM & Mô-men Nm)       │
 └──────────────────────────────┬──────────────────────────────┘
                                │
                                ▼
 ┌─────────────────────────────────────────────────────────────┐
 │ Bước 2: Chọn Số Cặp Cực P (Nhiều cặp cực cho lực kéo/robot, │
 │                           Ít cặp cực cho tốc độ cao/quạt)   │
 └──────────────────────────────┬──────────────────────────────┘
                                │
                                ▼
 ┌─────────────────────────────────────────────────────────────┐
 │ Bước 3: Chọn Số Rãnh Slots S (Chọn tổ hợp có Kw cao > 0.9)  │
 └──────────────────────────────┬──────────────────────────────┘
                                │
                                ▼
 ┌─────────────────────────────────────────────────────────────┐
 │ Bước 4: Kiểm tra giới hạn Tần số điện vs Tần số ngắt FOC    │
 │         (f_dien = P * RPM / 60 < F_FOC / 20)                │
 └──────────────────────────────┬──────────────────────────────┘
                                │
                                ▼
 ┌─────────────────────────────────────────────────────────────┐
 │ Bước 5: Chọn Cảm Biến Góc (Độ phân giải Encoder >= 12-bit)  │
 └─────────────────────────────────────────────────────────────┘
```

### Chi tiết từng bước:

#### Bước 1: Xác định yêu cầu tải
- Ứng dụng cần quay nhanh (hàng chục nghìn RPM, như quạt, drone, bơm)?
- Hay cần lực kéo khỏe ở tốc độ chậm (dưới 3.000 RPM, như khớp tay robot, bánh xe AGV, Gimbal)?

#### Bước 2: Quyết định số cặp cực P
- **Nếu cần Tốc độ cao:** Chọn `P = 1, 2, 3` (tương ứng 2, 4, 6 cực).
- **Nếu cần Lực kéo lớn, chạy êm, điều khiển vị trí:** Chọn `P = 7, 10, 11, 14` (tương ứng 14, 20, 22, 28 cực).

#### Bước 3: Chọn số rãnh S
- Dựa trên số cực đã chọn ở Bước 2, tra cứu bảng phối hợp kinh điển (Mục 6) để chọn số rãnh `S` sao cho hệ số `Kw` lớn nhất và mô-men Cogging nhỏ nhất.

#### Bước 4: Kiểm tra khả năng đáp ứng của Vi điều khiển (MCU)
- Tính tần số điện cực đại: `f_dien_max = (P * RPM_max) / 60`.
- **Quy tắc vàng của FOC:** Tần số ngắt FOC phải gấp **ít nhất 20 lần** tần số điện cực đại:
  ```text
  F_ngat_FOC >= 20 * f_dien_max
  ```
- *Ví dụ kiểm tra:* Động cơ `P = 10`, tốc độ mong muốn `3.000 RPM` $\implies f = (10 * 3000)/60 = 500\text{ Hz}$.
  $\implies F_{FOC} \ge 20 \times 500 = 10.000\text{ Hz} = 10\text{ kHz}$.
  Vi điều khiển LCM32F037 trong dự án chạy ngắt FOC ở **20 kHz**, hoàn toàn dư sức đáp ứng mượt mà!

#### Bước 5: Chọn cảm biến vị trí góc
- Động cơ có `P >= 7`: Tuyệt đối không dùng cảm biến Hall 6 bước nếu muốn chạy FOC mượt. Bắt buộc dùng cảm biến góc từ trường (Magnetic Encoder) giao tiếp SPI/SSI có độ phân giải từ 12-bit đến 14-bit (như MT6701, AS5048).

---

## 6. Bảng Tra Cứu Các Tổ Hợp Slots / Poles Công Nghiệp Phổ Biến

| Cấu Hình (Slots - Poles) | Số Cặp Cực (P) | Hệ Số Kw | Đặc Tính Nổi Bật | Ứng Dụng Điển Hình |
| :---: | :---: | :---: | :--- | :--- |
| **3S2P / 6S2P** | **P = 1** | 0.866 | Tốc độ siêu cao (50k - 100k RPM), mô-men nhỏ | Máy mài nha khoa, quạt phản lực mini |
| **9S6P** | **P = 3** | 0.866 | Tốc độ cao, lực kéo trung bình | Máy bay mô hình RC tốc độ cao |
| **12S10P** | **P = 5** | 0.933 | Mô-men rất tốt, Cogging cực thấp, chạy rất êm | Động cơ servo cỡ nhỏ, máy in 3D cao cấp |
| **12S14P** | **P = 7** | 0.933 | **Tổ hợp phổ biến nhất thế giới**: Tỷ số Lực/Trọng lượng tối ưu | Động cơ Drone / Flycam quadcopter |
| **18S20P** | **P = 10** | 0.945 | Mô-men lớn, dạng đĩa mỏng (Pancake), chạy siêu êm | Gimbal chống rung camera, robot công nghiệp |
| **24S20P / 24S22P** | **P = 10, 11** | 0.949 | Mô-men cực lớn, bước từ siêu mịn, truyền động trực tiếp | Khớp xoay Robot, xe lăn điện, bánh xe robot AGV |
| **36S42P** | **P = 21** | 0.953 | Siêu mô-men xoắn ở tốc độ thấp, đường kính lớn | Động cơ truyền động trực tiếp máy giặt, xe đạp điện |

---

## 7. Phân Tích Cấu Hình Động Cơ EC4012 (P = 10) Trong Dự Án

Động cơ **EC4012** được trang bị trên robot của bạn là dòng động cơ không chổi than dạng đĩa dẹt (Pancake Motor) với thông số kỹ thuật thực tế:

```text
       Đường kính ngoài: ~40 mm
       Chiều dày stator: ~12 mm
       Số cặp cực (P):   10 cặp cực (20 cực nam châm trên rotor)
       Cấu hình:         Thường là 18S20P hoặc 24S20P
```

### Tại sao nhà sản xuất lại chọn cấu hình P = 10 cho EC4012?
1. **Mục đích thiết kế làm khớp Robot (Robot Joint Actuator):**
   - Khớp robot đòi hỏi **lực giữ và lực kéo rất khỏe**, không cần quay quá nhanh (thường dưới 1.000 RPM) nhưng cần giữ vị trí cực kỳ vững và mượt mà.
   - Cấu hình `P = 10` đem lại mật độ mô-men xoắn (Torque Density) cực cao ngay trên kích thước nhỏ gọn 40mm.
2. **Loại bỏ hộp số cồng kềnh:**
   - Nhờ lực kéo mạnh của 10 cặp cực, động cơ có thể gắn trực tiếp vào cơ cấu truyền động khớp mà không cần hộp số bánh răng nhiều cấp, giảm độ rơ (backlash) và tiếng ồn.
3. **Sự ăn khớp hoàn hảo với firmware FOC trong dự án:**
   - Vi điều khiển LCM32F037 kích hoạt ngắt định thời tại **20 kHz** (`50 µs` một lần).
   - Ở tốc độ làm việc tối đa 1.000 RPM của khớp robot, tần số điện chỉ là:
     `f = (10 * 1000) / 60 = 166.7 Hz`.
   - Với ngắt 20 kHz, MCU lấy mẫu tới **120 lần trong một chu kỳ sóng điện** (`20.000 / 166.7 = 120`), tạo ra chất lượng sóng sin điều khiển cực kỳ hoàn hảo, chuyển động êm tuyệt đối và không phát ra tiếng rít khó chịu.

---
*Tài liệu được biên soạn phục vụ công tác thiết kế phần cứng, lựa chọn động cơ và tối ưu hóa hệ thống điều khiển FOC.*
