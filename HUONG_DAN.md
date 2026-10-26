# Hướng Dẫn Robot 2 Bánh Tự Cân Bằng

## Sơ Đồ Kết Nối

### MPU-6050 (GY-521) → Arduino Uno
| GY-521 | Arduino Uno |
|--------|-------------|
| VCC    | 5V (hoặc 3.3V) |
| GND    | GND |
| SDA    | A4  |
| SCL    | A5  |
| AD0    | GND (địa chỉ 0x68) |
| INT    | Không dùng |

### CNC Shield (gắn lên Arduino Uno)
CNC Shield cắm trực tiếp lên Arduino Uno.

### A4988 → CNC Shield
- Cắm A4988 vào slot **X** (motor trái) và **Y** (motor phải)
- Chú ý chiều cắm: đúng chiều theo ký hiệu trên board

### NEMA 17 → A4988
Mỗi motor NEMA 17 có 4 dây (2 cuộn):
| Cuộn | Dây | A4988 |
|------|-----|-------|
| A    | A+  | 1A    |
| A    | A-  | 1B    |
| B    | B+  | 2A    |
| B    | B-  | 2B    |

> Nếu motor quay ngược chiều mong muốn → đổi A+ với A- (hoặc B+ với B-)

### Nguồn Điện

Xem mục **Hướng Dẫn Nguồn Điện Chi Tiết** bên dưới.

---

## Cài Đặt Microstepping (Jumper trên CNC Shield)

Cắm jumper vào vị trí **MS1, MS2, MS3** bên dưới từng slot driver:

| Chế độ | MS1 | MS2 | MS3 | Ghi chú |
|--------|-----|-----|-----|---------|
| Full   | OFF | OFF | OFF | Mượt ít, nhanh |
| 1/2    | ON  | OFF | OFF | |
| 1/4    | OFF | ON  | OFF | |
| **1/8** | **ON** | **ON** | **OFF** | **Khuyến nghị** |
| 1/16   | ON  | ON  | ON  | Mượt nhất, chậm hơn |

> Trong code, `MICROSTEP = 8` phải khớp với jumper bạn cắm.

---

## Điều Chỉnh Dòng A4988

Xoay biến trở trên A4988 để đặt dòng phù hợp NEMA 17:
```
Vref = Imáx × 8 × Rs
```
Với NEMA 17 thông thường (1.5A): Vref ≈ 0.75V (chỉnh ~50% để an toàn khi test).

---

## Hướng Dẫn Tinh Chỉnh PID

Mở file `robot_balance.ino`, tìm phần:
```cpp
float Kp = 18.0;
float Ki = 0.8;
float Kd = 1.2;
float TARGET_ANGLE = 0.0;
```

### Quy trình chỉnh:
1. **Đặt Kp=10, Ki=0, Kd=0** → upload lên Arduino
2. **Mở Serial Monitor** (115200 baud) xem góc và PID output
3. **Tăng Kp** từ từ đến khi robot bắt đầu lắc nhẹ quanh điểm đứng
4. **Tăng Kd** để giảm lắc (thường = Kp × 0.05 ~ 0.1)
5. **Tăng Ki** nhỏ (0.1~1.0) nếu robot cứ trôi về một phía
6. **Chỉnh TARGET_ANGLE** (+/-) nếu robot nghiêng về phía trước hoặc sau khi đứng yên

### Dấu hiệu nhận biết:
| Hiện tượng | Nguyên nhân | Giải pháp |
|-----------|------------|-----------|
| Robot không phản ứng | Kp quá nhỏ | Tăng Kp |
| Robot lắc mạnh, té | Kp quá lớn | Giảm Kp |
| Robot rung/dao động | Kd quá nhỏ | Tăng Kd |
| Robot phản ứng chậm | Kd quá lớn | Giảm Kd |
| Robot trôi dần một hướng | Ki quá nhỏ | Tăng Ki |
| Robot lắc chậm, không ổn định | Ki quá lớn | Giảm Ki |

---

## Hướng Đặt MPU-6050

MPU-6050 phải đặt sao cho:
- **Trục Y** nằm dọc theo chiều cao robot (hướng lên/xuống)
- **Trục Z** nằm ngang (hướng trục bánh xe)

Nếu đặt khác hướng → sửa hàm `calcAccelAngle()` trong code:
```cpp
// Hiện tại dùng ay và az
float calcAccelAngle(int16_t ay, int16_t az) {
    return atan2f((float)ay, (float)az) * 57.2957795f;
}
// Nếu IMU xoay 90° → đổi thành ax, az
// Nếu IMU xoay khác → đổi phù hợp
```

---

## Thư Viện Cần Cài

Trong Arduino IDE → Tools → Manage Libraries:
- **Wire** (có sẵn, không cần cài)

---

## Upload Code

1. Mở `robot_balance.ino` bằng Arduino IDE
2. Chọn Board: **Arduino Uno**
3. Chọn đúng COM port
4. Click **Upload**
5. Mở **Serial Monitor** (115200 baud) để xem dữ liệu

---

---

## Hướng Dẫn Nguồn Điện Chi Tiết

### Kiến Trúc Nguồn

Robot dùng **1 nguồn 12V** cấp cho cả motor lẫn Arduino:

```
[Pin 12V]
   |
   +--[Công tắc]--[Cầu chì 5A]--+-- CNC Shield VIN (terminal block) → Motor
   |                              |
   |                             [Tụ 100µF ~ 470µF] (song song, lọc spike)
   |                              |
   +------------------------------+-- Arduino Jack DC (7–12V)
   |
  GND ──────────────────────────────── CNC Shield GND = Arduino GND (chung)
```

> **Quan trọng:** GND của nguồn motor và GND Arduino **phải nối chung**.  
> Nếu không, A4988 hoạt động sai và có thể hỏng chip.

---

### Lựa Chọn Pin / Nguồn

| Loại | Điện áp | Dung lượng | Ghi chú |
|------|---------|-----------|---------|
| **3S LiPo** ✓ tốt nhất | 11.1–12.6V | ≥ 2200mAh 25C | Nhẹ, xả nhanh, lý tưởng cho robot |
| 3x 18650 nối tiếp | 11.1–12.6V | ≥ 2500mAh/cell | Rẻ hơn LiPo, dễ mua |
| Adapter DC 12V | 12V cố định | — | Chỉ dùng khi test tại bàn, ≥ 5A |
| Ắc quy 12V lead-acid | 12V | 7–12Ah | Nặng nhưng rẻ, dùng được |

**Dòng tối thiểu: 5A** (tổng tải ~2.6A, peak ~4A khi khởi động motor)

---

### Tính Dòng Tiêu Thụ

| Linh kiện | Dòng điện |
|-----------|----------|
| 2x NEMA17 (tải trung bình) | ~2.5A |
| Arduino Uno | ~50mA |
| 2x A4988 (logic) | ~20mA |
| MPU-6050 | ~4mA |
| **Tổng** | **~2.6A** (peak ~4A) |

---

### Đấu Dây Nguồn Vào CNC Shield

**Terminal block trên CNC Shield** (góc board, 2 đầu vít):
- Đầu **`+`** (VIN): nối dây **đỏ** từ pin 12V (qua công tắc và cầu chì)
- Đầu **`–`** (GND): nối dây **đen** từ pin 12V

**Arduino:**
- Nối cùng nguồn 12V vào **Jack DC** của Arduino (trung tâm `+`, vỏ `–`)
- Hoặc dùng USB để cấp nguồn Arduino riêng (nhưng vẫn phải nối chung GND)

---

### Tụ Lọc Nguồn (Bắt Buộc)

Gắn **tụ điện 100µF ~ 470µF, 16V trở lên** song song với terminal block VIN của CNC Shield:
- Cực `+` tụ → VIN (dây đỏ)
- Cực `–` tụ → GND (dây đen)

> Không có tụ → khi motor đảo chiều đột ngột, voltage spike có thể **hỏng A4988 vĩnh viễn**.

---

### Thứ Tự Đấu Dây An Toàn

1. **Đấu GND trước**: pin `–` → CNC Shield GND → Arduino GND (chung một điểm)
2. **Gắn tụ lọc** song song với terminal block VIN/GND
3. **Đấu +12V** vào CNC Shield terminal block VIN
4. **Đấu +12V** (hoặc USB) vào Arduino DC Jack
5. **Kiểm tra lại toàn bộ** trước khi bật công tắc
6. ⚠️ **KHÔNG bật nguồn 12V khi đang cắm USB** để upload code — ngắt USB trước

---

## Lưu Ý An Toàn

- Giữ robot đứng yên trong **2 giây đầu** khi cấp nguồn (đang calibrate gyro)
- Nếu robot ngã quá 45° → motor tự động tắt an toàn
- Nguồn motor nên dùng **12V / ≥2A** cho 2 motor NEMA 17
- Không để robot chạy trên bề mặt trơn trượt khi mới test
