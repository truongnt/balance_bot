/*
 * ============================================================
 *   ROBOT 2 BANH TU CAN BANG - v2 (B-ROBOT EVO2 architecture)
 * ============================================================
 *  Thay doi so voi v1:
 *  1. TIMER1 ISR phat xung step (chinh xac, khong giat boi Serial.print)
 *  2. CASCADED PID: vong ngoai van toc -> dieu chinh target_angle
 *     -> khong can chinh TARGET_ANGLE thu cong, robot khong bi troi
 *  3. ACCELERATION RAMPING: gioi han dSpeed/dt -> motor khong mat buoc,
 *     giam dong peak -> mat me hon
 *  4. CALIBRATION TACH LAM HAI (2026-10-25):
 *     - gyroBias: do MOI LAN BOOT, robot chi can dung YEN (tu the nao cung duoc).
 *       Bias nay troi theo nhiet (do duoc +2.1 roi -3.08 do/s trong cung mot buoi).
 *     - upright_offset (UPOFF): goc lap board MPU tren khung = HANG SO CO KHI.
 *       Do MOT LAN bang 'calup' (cam robot thang dung) roi 'save' vao EEPROM.
 *       Truoc day gop chung nen moi lan boot phai cam thang dung, va chi can cam
 *       hoi nghieng la lech setpoint: do duoc 2.32 / 4.51 / 5.47 do giua cac lan.
 *
 *  Phan cung (giong v1):
 *    Arduino Uno + CNC Shield + 2x A4988 + 2x NEMA17 + MPU-6050
 *    Motor trai  (X): STEP=D2, DIR=D5
 *    Motor phai  (Y): STEP=D3, DIR=D6
 *    Enable       : D8 (LOW = bat)
 *    MPU SDA/SCL  : A4 / A5
 *    Microstep 1/8: jumper M0+M1 ON, M2 OFF
 *
 *  BO THAM SO MAC DINH = bo DA DO CHUNG MINH 2026-10-22:
 *    144 giay can bang lien tuc, angle sigma 0.48 do, 96% thoi gian trong +-1 do
 *    ARCH=1 KP=45 KD=2.5 VDIR=1 KPV=0.008 KIV=0.001 ACC=350 SPD=3000
 *    KPPOS=0.015 (vong vi tri, do 2026-10-25: sigma 0.272 do, troi 0.064 cm/s)
 *  Hai loi goc da sua:
 *    - arch 0 ra lenh TOC DO -> banh lan deu khong sinh luc -> khong bao gio can bang duoc
 *    - vel_dir=-1 la PHAN HOI DUONG, tu khuech dai troi
 *
 *  Quy trinh tinh chinh (CHO arch 1 - quy trinh cu cho arch 0 da bo):
 *    1. Chinh tu TRONG ra NGOAI. Vong ngoai phai CHAM HON vong no boc.
 *    2. kp: diem hoa la 9.7 (vua du thang trong luc). 45 = ~4.6 lan.
 *    3. kd: tang cho het dao dong sau khi bi day.
 *    4. kpv: GIU THAP. 0.03 lam xau gap 4, 0.06 lam do. Chinh kiv thay vi kpv.
 *    5. kppos: 0.015. 0.05 sinh chu trinh gioi han 0.33Hz.
 *    6. Dung configs/balance_good.txt, dung chep tay tung so.
 * ============================================================
 */

#include <Wire.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Khai bao truoc: Arduino tu sinh prototype o dau file, neu khong co
// dong nay thi prototype cua getParamVal/setParamVal se khong biet kieu.
struct ParamDef;
struct VarDef;

// ============================================================
// PIN MAPPING (CNC Shield) - dung direct port de toc do
// ============================================================
//  D2 = PORTD bit 2 (STEP_L)
//  D3 = PORTD bit 3 (STEP_R)
//  D5 = PORTD bit 5 (DIR_L)
//  D6 = PORTD bit 6 (DIR_R)
//  D8 = PORTB bit 0 (ENABLE)
#define STEP_L_HIGH()  (PORTD |=  (1 << 2))
#define STEP_L_LOW()   (PORTD &= ~(1 << 2))
#define STEP_R_HIGH()  (PORTD |=  (1 << 3))
#define STEP_R_LOW()   (PORTD &= ~(1 << 3))
#define DIR_L_HIGH()   (PORTD |=  (1 << 5))
#define DIR_L_LOW()    (PORTD &= ~(1 << 5))
#define DIR_R_HIGH()   (PORTD |=  (1 << 6))
#define DIR_R_LOW()    (PORTD &= ~(1 << 6))
#define ENABLE_PIN     8

// MPU-6050
#define MPU_ADDR       0x68

// ============================================================
// THAM SO DIEU KHIEN
// ============================================================

// --- PID vong trong (Angle, ~100Hz) ---
// Gia tri arch 1, do 2026-10-22. KHONG dung chung voi arch 0 (von la 80/12).
float Kp_angle = 45.0;   // arch 1: ~4.6x trong luc (diem hoa la 9.7)
float Ki_angle = 0.0;
float Kd_angle = 2.5;    // arch 1: zeta ~0.7

// --- PID vong ngoai (Velocity, ~10Hz) ---
float Kp_vel  = 0.008;
float Ki_vel  = 0.001;
int8_t vel_dir = 1;             // tune LIVE qua lenh "vdir -1" / "vdir 1"
float max_target_offset = 3.0f;  // tune LIVE qua lenh "mto 3.0"

// Bias cho CG offset - tune LIVE qua lenh "bias +0.5", luu EEPROM bang "save"
float target_bias = 0.0f;

// Velocity filter — tune LIVE qua lenh "vfa 0.6"
float vel_filter_alpha = 0.7f;

// --- Stepper / motor --- tune LIVE qua lenh "maxspd / maxacc"
#define MICROSTEP          8
int16_t max_steps_per_sec = 3000;
int16_t max_accel          = 350;   // B-ROBOT EVO2: MAX_ACCEL 14 x25 (1/8 step) = 350/tick
#define MIN_STEP_RATE      8     // period toi da = ZERO_SPEED -> 250000/32000 = 7.8
#define ZERO_SPEED         32000

// Chuoi phien ban: DOI MOI LAN SUA FIRMWARE.
// Hoi bang lenh "VER" qua BT/USB, va in luon trong banner khoi dong.
// Co cai nay thi khong bao gio phai suy ra "chip dang chay ban nao" tu hanh vi
// nua — 26/10 da suy sai hai lan, trong khi lenh nap van bao thanh cong.
#define FW_VERSION "2026-10-27a-drive"

// --- An toan --- tune LIVE qua "fall 45 / rec 10 / recg 100"
float fall_angle    = 45.0f;
float recover_angle = 10.0f;
float recover_gyro  = 100.0f;

// --- Kien truc output --- doi LIVE qua lenh "arch 0" / "arch 1"
//  0 = PID -> toc do TRUC TIEP  (ban hien tai)
//  1 = PID -> GIA TOC, tich phan thanh toc do (kieu B-ROBOT EVO2: control_output += ...)
//  LUU Y: hai che do can BO GAIN HOAN TOAN KHAC NHAU.
//    arch 0 -> kp ~80, kd ~12   (nhung arch 0 KHONG BAO GIO can bang duoc:
//                                lenh toc do -> banh lan deu -> khong sinh luc)
//    arch 1 -> kp 45,  kd 2.5   (DO THUC TE, khong phai uoc tu B-ROBOT.
//                                Chep ti le Kd/Kp cua B-ROBOT ra kd 180 -> hong.)
uint8_t integrate_out  = 1;
float   control_output = 0;      // state cua che do tich phan

// --- MPU DLPF (reg 0x1A) --- tune LIVE qua "dlpf 5"
//  0=260Hz 1=184 2=94 3=44 4=21 5=10 6=5 Hz. Loc manh hon = tre nhieu hon.
uint8_t dlpf_cfg = 0x05;

// --- Vong dieu khien ---
#define LOOP_PERIOD_US     10000UL
#define VEL_LOOP_DIVIDER   10

// --- Complementary filter --- tune LIVE qua lenh "alpha / alfa"
float cf_alpha      = 0.97f;   // Tin gyro bao nhieu (0.95-0.98)
float accel_lpf     = 0.85f;   // Loc noise accel tu motor (0.7-0.9): 0.85 giam filter-drift

// --- Soft resync CF vs accel --- "resync 3" / "resynck 0.04"
float resync_thresh = 3.0f;    // Lech bao nhieu do thi keo CF ve phia accel
float resync_gain   = 0.04f;   // Keo manh bao nhieu moi vong

// --- Anti-windup vong velocity --- "awin 5"
float awin_angle    = 5.0f;    // Chi tich luy Ki_vel khi |angle| nho hon nguong nay

// --- VONG VI TRI --- "kppos 0.05" (0 = TAT)
//  Vong velocity chi triet tieu VAN TOC, khong dua robot VE CHO CU.
//  Vong nay dung stepsL/stepsR tich luy lam phan hoi vi tri, sinh ra
//  mot vel_setpoint keo robot ve pos_target. Dang bi lai thi tu tat.
long  pos_target  = 0;      // moc vi tri muon giu (step)
float Kp_pos      = 0.015f; // DO 2026-10-25: angle sigma 0.272, 99% trong +-1 do,
                            // troi 0.064 cm/s (khong co vong vi tri: 1.04 cm/s).
                            // 0 = tat. 0.05 sinh chu trinh gioi han 0.33Hz, sigma 1.86.
float vel_cmd_max = 40.0f;  // tran cho vel_setpoint (step/100ms)
// DO 25/10/2026: de 200 (=2000 step/s) thi vong vi tri luc lech 1m doi 872 step/s
// va duoc cho het -> speed leo toi tran 3000 trong 2.5s roi robot nga. 40 = 400
// step/s ~ 7 cm/s, nam trong gia toc that cua motor.
float drive_vel_gain = 20.0f; // steps/100ms tren moi don vi bt_drive
// DRIVE 1.0 -> 20 steps/100ms = 200 step/s = 3.5 cm/s; DRVMAX 1.5 -> 5.3 cm/s.

// --- Lai tu xa --- tune LIVE: "drvstep / drvmax / trnstep / trnmax"
float   bt_drive_step = 0.4f;   // moi lan F/B thay doi target bao nhieu do
float   bt_drive_max  = 1.5f;
int16_t bt_turn_step  = 100;    // moi lan L/R thay doi turn bao nhieu step/s
int16_t bt_turn_max   = 300;
// --- Ho tro app dieu khien tren dien thoai ---
// Xem ghi chu dai trong handleBTCommand() ve cach phan biet ky tu don voi tu.
uint8_t bt_idle_ms    = 60;     // im qua bao lau thi chot lenh (ms)
float   app_speed     = 1.0f;   // thanh truot 0-9 cua app -> he so 0.1..1.0

// --- Soft-start sau recovery --- "soft 300" (300 x 10ms = 3 giay)
int16_t soft_start_loops = 300;

// ============================================================
// BIEN TOAN CUC
// ============================================================
uint8_t deferred_print = 0;   // 0=idle, >0 = dang in (1 dong moi tick)
uint8_t deferred_end   = 0;   // chi so dong cuoi cung can in

float angle           = 0;     // Goc nghieng filtered (do)
float gyroBias        = 0;     // Offset gyro (raw)
// Goc doc duoc tu accel KHI ROBOT THANG DUNG THAT = goc lap dat cua board MPU
// tren khung. Day la HANG SO CO KHI, khong doi giua cac lan bat may, nen phai
// do MOT LAN roi luu EEPROM — khong do lai moi lan boot.
// Do lai bang lenh 'calup' (phai cam robot thang dung).
// 2.0f PHAI khop voi cot default cua UPOFF trong bang PARAMS: mac dinh nam o hai
// cho. 'reset' dung cot trong bang, con khi EEPROM trong thi firmware giu nguyen
// gia tri khoi tao nay. Lech nhau thi hai duong cho ra hai cau hinh khac nhau —
// da dinh dung 26/10: nap xong UPOFF ra 0 thay vi 2.0. Chay check_defaults.py.
float upright_offset  = 2.0f;

// Inner PID state
float pidI_angle      = 0;

// Outer PID state
float target_angle    = 0;     // Setpoint dong, do velocity loop dieu chinh
float pidI_vel        = 0;
long  last_steps_avg  = 0;

// Speed ramping
int16_t current_speed = 0;     // Toc do hien tai sau ramping (chung cho 2 motor)

// === Bien trung gian cua loop() - de toan cuc de BANG LOG truy cap duoc ===
float gyro_rate    = 0;   // do/s, da tru bias
float accel_angle  = 0;   // goc tu accel, sau LPF
float vel_filtered = 0;   // van toc (step/100ms) sau LPF
float vel_err      = 0;   // sai so van toc dua vao vong ngoai
float pid_out      = 0;   // ket qua PID vong trong (truoc khi vao arch 0/1)
float dt           = 0;   // chu ky thuc te cua vong (giay)
long  loop_ms      = 0;   // millis() tai moi tick - moc thoi gian THAT cua robot

// === BLUETOOTH REMOTE CONTROL ===
// Nhan lenh tu HC-05 qua Serial (D0/D1)
float bt_drive    = 0;         // -1.5 .. +1.5 do, them vao target_angle (tien/lui)
int16_t bt_turn   = 0;         // -300 .. +300 step/s, motor R - motor L

// Trang thai
bool fallen           = false;

// Soft-start sau recovery: ramp Kp 50%->100% trong soft_start_loops vong
uint16_t soft_start_cnt = 0;

// === Volatile state truy cap trong ISR ===
volatile uint16_t periodL = ZERO_SPEED;   // ticks giua 2 step (ticks=0.5us @ Timer1 prescaler 8)
volatile uint16_t periodR = ZERO_SPEED;
volatile int8_t   dirSignL = 0;            // -1, 0, +1 (cho buoc dem)
volatile int8_t   dirSignR = 0;
volatile long     stepsL = 0;              // Buoc da chay (signed, cho tinh van toc)
volatile long     stepsR = 0;

// Timing
unsigned long last_loop_us = 0;

// ============================================================
// TIMER1 ISR - Phat xung step (precise)
// ============================================================
//  Timer1: 16-bit, prescaler 8 -> 2 MHz tick (0.5us/tick)
//  Dung 2 channel doc lap: OCR1A cho motor L, OCR1B cho motor R
//  Khi compare match: phat 1 xung step (HIGH ngan roi LOW)
//                     va lich next interrupt = OCR + period

ISR(TIMER1_COMPA_vect) {
  if (periodL >= ZERO_SPEED) { OCR1A = TCNT1 + ZERO_SPEED; return; }  // dung: hen lai xa
  uint16_t next = OCR1A + periodL;
  if ((int16_t)(next - TCNT1) < 4) next = TCNT1 + 4;  // slip guard (period < 32000 nen cast an toan)
  OCR1A = next;
  STEP_L_HIGH();
  stepsL += dirSignL;   // volatile long += volatile int8_t: ~25 cycles = 1.5us > A4988 min
  STEP_L_LOW();
}

ISR(TIMER1_COMPB_vect) {
  if (periodR >= ZERO_SPEED) { OCR1B = TCNT1 + ZERO_SPEED; return; }  // dung: hen lai xa
  uint16_t next = OCR1B + periodR;
  if ((int16_t)(next - TCNT1) < 4) next = TCNT1 + 4;  // slip guard (period < 32000 nen cast an toan)
  OCR1B = next;
  STEP_R_HIGH();
  stepsR += dirSignR;
  STEP_R_LOW();
}

void initTimer1() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler 64 -> 250kHz (4us/tick)
  // Prescaler 64 thay vi 8: chu ky toi da van la ZERO_SPEED=32000 tick,
  // nhung moi tick gio la 4us nen toc do CHAM NHAT lenh duoc tut tu
  // 62 step/s xuong 7.8 step/s. Do phan giai o toc do cao van du:
  // 3000 step/s -> period 83 tick (sai so 1.2%).
  OCR1A   = TCNT1 + ZERO_SPEED;
  OCR1B   = TCNT1 + ZERO_SPEED;
  TIFR1   = (1 << OCF1A) | (1 << OCF1B);  // Clear flag
  TIMSK1  = (1 << OCIE1A) | (1 << OCIE1B);
  interrupts();
}

// ============================================================
// SET MOTOR SPEED (voi acceleration ramping + turn support)
// ============================================================
//  base_speed: toc do trung binh, -MAX..+MAX step/s
//  turn_offset: chenh lech R - L. DO THUC TE 2026-10-25:
//    bt_turn DUONG -> banh phai tien, banh trai lui -> xoay THEO chieu kim
//    dong ho (sang PHAI). Lenh 'R' cong bt_turn nen 'R' = re phai, dung nhan.
//    Toc do xoay do duoc: turn=120 -> ~22 do/giay, van giu can bang.
//  Ramping ap dung tren base_speed
void setMotorSpeed(int16_t target_speed) {
  // Ramping tren base_speed
  if (target_speed > current_speed + max_accel)
    current_speed += max_accel;
  else if (target_speed < current_speed - max_accel)
    current_speed -= max_accel;
  else
    current_speed = target_speed;

  // Ap dung turn offset
  int16_t left_speed  = current_speed - bt_turn;
  int16_t right_speed = current_speed + bt_turn;
  setSingleMotor(true,  left_speed);   // motor trai
  setSingleMotor(false, right_speed);  // motor phai
}

// Helper: set 1 motor (left=true cho L, false cho R)
void setSingleMotor(bool isLeft, int16_t speed) {
  int8_t sign;
  uint16_t absspeed;
  if (speed >= 0) { sign = +1; absspeed = speed; }
  else            { sign = -1; absspeed = -speed; }

  // DIR pin (motor R dao chieu vi gan doi xung)
  if (isLeft) {
    if (sign > 0) DIR_L_HIGH(); else DIR_L_LOW();
  } else {
    if (sign > 0) DIR_R_LOW();  else DIR_R_HIGH();
  }

  // Period
  uint16_t newPeriod;
  if (absspeed < MIN_STEP_RATE) {
    newPeriod = ZERO_SPEED;
  } else {
    uint32_t p = 250000UL / absspeed;   // 250kHz sau khi doi prescaler
    if (p >= ZERO_SPEED) newPeriod = ZERO_SPEED;
    else                 newPeriod = (uint16_t)p;
  }

  noInterrupts();
  if (isLeft) {
    bool was_stopped = (periodL >= ZERO_SPEED);
    periodL  = newPeriod;
    dirSignL = (newPeriod >= ZERO_SPEED) ? 0 : sign;
    if (was_stopped && newPeriod < ZERO_SPEED) {
      OCR1A = TCNT1 + 100;
    }
  } else {
    bool was_stopped = (periodR >= ZERO_SPEED);
    periodR  = newPeriod;
    dirSignR = (newPeriod >= ZERO_SPEED) ? 0 : sign;
    if (was_stopped && newPeriod < ZERO_SPEED) {
      OCR1B = TCNT1 + 100 + (newPeriod >> 1);
    }
  }
  interrupts();
}

void stopMotors() {
  noInterrupts();
  periodL = ZERO_SPEED;
  periodR = ZERO_SPEED;
  dirSignL = 0;
  dirSignR = 0;
  interrupts();
  current_speed = 0;
}

void enableMotors()  { digitalWrite(ENABLE_PIN, LOW);  }
void disableMotors() { digitalWrite(ENABLE_PIN, HIGH); }

// ============================================================
// MPU-6050
// ============================================================
void writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}

void readMPU(int16_t *ax, int16_t *ay, int16_t *az,
             int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  *ax = (Wire.read() << 8) | Wire.read();
  *ay = (Wire.read() << 8) | Wire.read();
  *az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();              // bo qua nhiet do
  *gx = (Wire.read() << 8) | Wire.read();
  *gy = (Wire.read() << 8) | Wire.read();
  *gz = (Wire.read() << 8) | Wire.read();
}

// Go ket bus I2C. Khi nap code / reset giua luc MPU dang truyen, MPU khong
// reset theo nen no tiep tuc ghi SDA xuong thap -> Wire.begin() cho mai mai.
// Danh toi da 9 xung SCL de MPU ket thuc byte dang do va nha SDA ra.
void i2cRecover() {
  pinMode(A4, INPUT_PULLUP);        // SDA
  pinMode(A5, INPUT_PULLUP);        // SCL
  delayMicroseconds(10);
  for (uint8_t i = 0; i < 9 && digitalRead(A4) == LOW; i++) {
    pinMode(A5, OUTPUT); digitalWrite(A5, LOW);   // keo SCL thap
    delayMicroseconds(5);
    pinMode(A5, INPUT_PULLUP);                    // nha cho pull-up keo len
    delayMicroseconds(5);
  }
  // Phat dieu kien STOP: SDA len cao trong khi SCL dang cao
  pinMode(A4, OUTPUT); digitalWrite(A4, LOW);
  delayMicroseconds(5);
  pinMode(A4, INPUT_PULLUP);
  delayMicroseconds(10);
}

void initMPU() {
  i2cRecover();
  Wire.begin();
  Wire.setWireTimeout(25000, true);  // 25ms roi bo cuoc, khong treo vinh vien
  Wire.setClock(400000);
  writeMPU(0x6B, 0x00);   // Wake
  writeMPU(0x1B, 0x00);   // Gyro ±250°/s -> 131 LSB/(°/s)
  writeMPU(0x1C, 0x00);   // Accel ±2g
  writeMPU(0x1A, dlpf_cfg);   // DLPF, mac dinh 10Hz - doi LIVE qua "dlpf n"
                          // Co the giam them 0x05 (10Hz) neu van rung
  delay(100);
}

// Goc tu accelerometer (giu nguyen cong thuc v1, them upright_offset)
float calcAccelAngleRaw(int16_t ay, int16_t az) {
  return atan2f((float)-ay, (float)az) * 57.2957795f;
}

// Calibrate: lay 1000 mau khi robot dung yen
//  - gyroBias: trung binh gx
//  - upright_offset: trung binh accelAngle (== goc bias khi cam dung thang)
// Lay trung binh N mau. Tra ve gyro raw trung binh, dong thoi ghi goc accel
// trung binh vao *outAng. Robot phai DUNG YEN trong suot qua trinh.
static float sampleStill(int N, float *outAng) {
  long  sumGx  = 0;
  float sumAng = 0;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU(&ax, &ay, &az, &gx, &gy, &gz);
    sumGx  += gx;
    sumAng += calcAccelAngleRaw(ay, az);
    delay(2);
  }
  if (outAng) *outAng = sumAng / N;
  return (float)sumGx / N;
}

// --- Chi do GYRO BIAS. Robot o tu the NAO CUNG DUOC, mien la dung yen. ---
// Bias gyro troi theo nhiet do (do thuc te: +2.1 roi -3.08 do/s trong mot buoi),
// nen phai do lai moi lan boot va bat ky khi nao robot troi mot huong.
void calibrateGyro() {
  Serial.println(F("Calib gyro: giu robot IM 4 giay (tu the nao cung duoc)..."));
  delay(2500);
  gyroBias = sampleStill(1000, NULL);
  Serial.print(F("  gyroBias = ")); Serial.println(gyroBias, 2);
}

// --- Chi do UPRIGHT_OFFSET. BAT BUOC cam robot THANG DUNG THAT. ---
// Day la hang so co khi (goc lap board MPU). Do xong nho 'save' de ghi EEPROM,
// tu do ve sau khong phai do lai nua.
void calibrateUpright() {
  Serial.println(F("Calib upright: cam robot THANG DUNG va im 4 giay..."));
  delay(2500);
  float ang;
  sampleStill(1000, &ang);
  upright_offset = ang;
  Serial.print(F("  upright_offset = ")); Serial.println(upright_offset, 2);
  Serial.println(F("  -> nghieng robot xuong roi 'save' de ghi EEPROM"));
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== ROBOT BALANCE v3 + BT ==="));

  // Load params tu EEPROM (neu co), neu khong dung gia tri mac dinh
  if (loadParams()) {
    Serial.println(F("Params loaded from EEPROM:"));
    printParams();
  } else {
    Serial.println(F("Using default params (no EEPROM data)"));
  }

  // Pin modes
  pinMode(2, OUTPUT); pinMode(3, OUTPUT);
  pinMode(5, OUTPUT); pinMode(6, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  disableMotors();

  // MPU
  initMPU();
  Serial.println(F("MPU OK"));

  // Chi do gyro bias. upright_offset da duoc loadParams() lay tu EEPROM —
  // KHONG do lai, vi do lai doi hoi cam robot thang dung moi lan boot va chi
  // can cam hoi nghieng la lech setpoint (do duoc: 2.32 / 4.51 / 5.47 do giua
  // cac lan boot). Chua co EEPROM thi upright_offset = 0 va robot se bao.
  calibrateGyro();
  if (upright_offset == 0.0f) {
    Serial.println(F("[!] UPOFF=0 chua hieu chinh. Cam robot thang dung roi"));
    Serial.println(F("[!] go 'calup', sau do nghieng xuong va 'save'."));
  }

  // Doc goc khoi tao
  int16_t ax, ay, az, gx, gy, gz;
  readMPU(&ax, &ay, &az, &gx, &gy, &gz);
  angle = calcAccelAngleRaw(ay, az) - upright_offset;
  Serial.print(F("Goc ban dau: ")); Serial.println(angle, 2);
  // BIAS_HINT da BO. Ly do: angle o day = 1 mau don TRU trung binh 1000 mau
  // vua lay xong ngay truoc do -> ky vong toan hoc = 0. No do NHIEU CAM BIEN,
  // khong do CG offset. Gui no thanh BIAS chi lam robot nghieng lech ngau nhien
  // +-1 do moi lan boot. Dung lenh "bias <val>" thu cong neu can bu CG.

  // Khoi tao Timer1 ISR
  initTimer1();

  // Chi bat motor neu robot DANG DUNG. Bat nguon luc robot dang nam thi vao
  // thang trang thai FALLEN, motor khong duoc cap dong -> khong phi dien.
  // Chi dung duoc tu khi UPOFF la hang so tuyet doi (xem muc 4 dau file);
  // truoc day 'angle' chi tuong doi voi tu the luc calibrate nen khong tin duoc.
  if (fabsf(angle) > fall_angle) {
    fallen = true;
    disableMotors();
    Serial.print(F("Robot dang NAM (")); Serial.print(angle, 1);
    Serial.println(F(" do) - motor tat, dung len de chay"));
  } else {
    enableMotors();
  }
  delay(100);

  Serial.println(F("Bat dau!"));
  emitCols();          // bao cho PC biet thu tu cot telemetry

  last_loop_us = micros();
}

// ============================================================
// BLUETOOTH COMMAND PARSER
// ============================================================
// Lenh 1 ky tu (gui qua app Serial Bluetooth Terminal):
//   F = forward (+target_angle)    B = backward (-target_angle)
//   L = turn left  (R faster)      R = turn right (L faster)
//   S = stop tien/lui/quay         P = print PID gains
// ============================================================
// EEPROM LAYOUT (luu params qua power cycle)
// ============================================================

// ============================================================
// BANG BIEN LOG (TELEMETRY TABLE)
// ============================================================
//  THEM 1 BIEN LOG DUOC = THEM 1 DONG vao VARS[] ben duoi.
//  Chon field nao duoc gui: log_mask (1 bit / dong, theo thu tu bang).
//    "log"          -> liet ke, [x] = dang bat
//    "log GYRO"     -> bat/tat field do
//    "logall" / "lognone"
//    "logmask 127"  -> dat thang bitmask (nam trong PARAMS, luu EEPROM)
//    "logdiv 2"     -> chia tan so: 1=100Hz, 2=50Hz, 4=25Hz
//  Moi lan doi lua chon, firmware in lai dong "#COLS ..." de PC biet cot.
//
//  BANG THONG: 115200 baud = ~11.5 KB/s. Moi field float ~7 byte.
//  7 field @100Hz = 4.3 KB/s (an toan). Tren 10 field thi dat logdiv 2.

#define VT_F32  0
#define VT_I16  1
#define VT_I32  2      // long thuong
#define VT_I32V 3      // volatile long, phai doc atomic

struct VarDef {
  char     name[8];
  void    *ptr;
  uint8_t  type;
  uint8_t  dec;        // so chu so thap phan khi in (chi cho VT_F32)
};

const VarDef VARS[] PROGMEM = {
//  ten        bien              kieu      dec
  { "ms",      &loop_ms,         VT_I32,   0 },  // moc thoi gian cua ROBOT
  { "angle",   &angle,           VT_F32,   2 },
  { "accel",   &accel_angle,     VT_F32,   2 },
  { "target",  &target_angle,    VT_F32,   2 },
  { "gyro",    &gyro_rate,       VT_F32,   1 },
  { "speed",   &current_speed,   VT_I16,   0 },
  { "stepsL",  (void*)&stepsL,   VT_I32V,  0 },
  { "stepsR",  (void*)&stepsR,   VT_I32V,  0 },
  // --- mac dinh TAT, bat khi can soi ---
  { "pidout",  &pid_out,         VT_F32,   0 },  // PID vong trong truoc arch
  { "ctrlout", &control_output,  VT_F32,   0 },  // sau arch (arch 1 moi khac pidout)
  { "ivel",    &pidI_vel,        VT_F32,   1 },  // tich phan vong velocity
  { "iang",    &pidI_angle,      VT_F32,   2 },  // tich phan vong angle
  { "velf",    &vel_filtered,    VT_F32,   1 },  // van toc sau loc
  { "velerr",  &vel_err,         VT_F32,   1 },
  { "drive",   &bt_drive,        VT_F32,   2 },
  { "turn",    &bt_turn,         VT_I16,   0 },
  { "dt",      &dt,              VT_F32,   4 },  // soi jitter vong lap
};
#define NUM_VARS  ((uint8_t)(sizeof(VARS) / sizeof(VARS[0])))

uint32_t log_mask = 0x00FF;   // 8 field dau: ms + 7 field kinh dien
                              // PHAI 32-bit: bang da co 17 dong, uint16 tran o dong 17
uint8_t  log_div  = 1;        // 1 = moi tick (100Hz)
uint8_t  log_cnt  = 0;

int8_t findVar(const char *name) {
  VarDef vd;
  for (uint8_t i = 0; i < NUM_VARS; i++) {
    memcpy_P(&vd, &VARS[i], sizeof(vd));
    if (strcasecmp(name, vd.name) == 0) return (int8_t)i;
  }
  return -1;
}

// Bao cho PC biet thu tu cot hien tai. In khi khoi dong va moi lan doi log_mask.
void emitCols() {
  VarDef vd;
  Serial.print(F("#COLS"));
  for (uint8_t i = 0; i < NUM_VARS; i++) {
    if (!(log_mask & ((uint32_t)1 << i))) continue;
    memcpy_P(&vd, &VARS[i], sizeof(vd));
    Serial.print(' ');
    Serial.print(vd.name);
  }
  Serial.println();
}

void emitTelemetry() {
  VarDef vd;
  bool first = true;
  for (uint8_t i = 0; i < NUM_VARS; i++) {
    if (!(log_mask & ((uint32_t)1 << i))) continue;
    memcpy_P(&vd, &VARS[i], sizeof(vd));
    if (!first) Serial.print('\t');
    first = false;
    switch (vd.type) {
      case VT_F32: Serial.print(*(float *)vd.ptr, vd.dec); break;
      case VT_I16: Serial.print(*(int16_t *)vd.ptr);       break;
      case VT_I32: Serial.print(*(long *)vd.ptr);          break;
      default: {
        noInterrupts();
        long v = *(volatile long *)vd.ptr;   // stepsL/stepsR: doc atomic
        interrupts();
        Serial.print(v);
        break;
      }
    }
  }
  Serial.println();
}

// ============================================================
// BANG THAM SO (PARAM TABLE)
// ============================================================
//  THEM 1 THAM SO TUNE DUOC = THEM 1 DONG vao PARAMS[] ben duoi.
//  Tu dong co ngay: lenh BT, luu/doc EEPROM, reset ve default, in khi go "p".
//  Phia Python KHONG phai sua gi - no gui thang "<TEN> <gia tri>".
//
//  Go "<TEN>" khong kem gia tri -> doc lai gia tri hien tai.

#define PT_F32  0
#define PT_I16  1
#define PT_I8   2
#define PT_U8   3
#define PT_U16  4
#define PT_U32  5

struct ParamDef {
  char     name[9];
  void    *ptr;
  uint8_t  type;
  float    lo, hi, def;
};

const ParamDef PARAMS[] PROGMEM = {
//  ten        bien                kieu      min       max      default
  { "KP",      &Kp_angle,          PT_F32,    0.0f,   500.0f,    45.0f  },
  { "KI",      &Ki_angle,          PT_F32,    0.0f,    50.0f,     0.0f  },
  { "KD",      &Kd_angle,          PT_F32,    0.0f,   500.0f,     2.5f  },
  { "KPV",     &Kp_vel,            PT_F32,    0.0f,     1.0f,     0.008f},
  { "KIV",     &Ki_vel,            PT_F32,    0.0f,     1.0f,     0.001f},
  { "VDIR",    &vel_dir,           PT_I8,    -1.0f,     1.0f,     1.0f  },
  { "MTO",     &max_target_offset, PT_F32,    0.5f,     8.0f,     3.0f  },
  { "BIAS",    &target_bias,       PT_F32,  -10.0f,    10.0f,     0.0f  },
  { "VFA",     &vel_filter_alpha,  PT_F32,    0.0f,     0.95f,    0.7f  },
  { "ALPHA",   &cf_alpha,          PT_F32,    0.80f,    0.999f,   0.97f },
  { "ALPF",    &accel_lpf,         PT_F32,    0.0f,     0.95f,    0.85f },
  { "DLPF",    &dlpf_cfg,          PT_U8,     0.0f,     6.0f,     5.0f  },
  { "SPD",     &max_steps_per_sec, PT_I16,  500.0f, 15000.0f,  3000.0f  },
  { "ACC",     &max_accel,         PT_I16,   50.0f,  3000.0f,   350.0f  },
  { "ARCH",    &integrate_out,     PT_U8,     0.0f,     1.0f,     1.0f  },
  { "FALL",    &fall_angle,        PT_F32,   15.0f,    80.0f,    45.0f  },
  { "REC",     &recover_angle,     PT_F32,    0.5f,    45.0f,    10.0f  },
  { "RECG",    &recover_gyro,      PT_F32,    5.0f,   500.0f,   100.0f  },
  { "RESYNC",  &resync_thresh,     PT_F32,    0.5f,    30.0f,     3.0f  },
  { "RESYNCK", &resync_gain,       PT_F32,    0.0f,     0.5f,     0.04f },
  { "AWIN",    &awin_angle,        PT_F32,    1.0f,    30.0f,     5.0f  },
  { "SOFT",    &soft_start_loops,  PT_I16,    0.0f,  2000.0f,   300.0f  },
  // UPOFF 2.0: DO 25/10/2026 — robot dang can bang on dinh, 'target' dung im o
  // +1.89..+2.10 do qua 3 lan do. Do la goc phai nghieng so voi cai accel coi la
  // "thang" thi moi giu duoc vi tri. De 0 thi moi lan boot vong toc do phai tich
  // phan hang chuc giay moi bo toi do -> dung cai "troi mot phia" luc moi dung.
  { "UPOFF",   &upright_offset,    PT_F32,  -90.0f,    90.0f,     2.0f  },
  { "KPPOS",   &Kp_pos,            PT_F32,    0.0f,     1.0f,     0.015f},
  { "VMAX",    &vel_cmd_max,       PT_F32,   10.0f,  2000.0f,    40.0f  },
  // Khoang cho phep AM: neu 'DRIVE 1' hoa ra la LUI thi gui 'DRVVEL -20' la doi
  // chieu, khong phai nap lai code. Chieu nay chua do duoc truoc khi nap vi no
  // phu thuoc cach gan motor, va dau target->buoc da do duoc la NGUOC nhau.
  { "DRVVEL",  &drive_vel_gain,    PT_F32, -200.0f,   200.0f,    20.0f  },
  // BTIDLE chinh song duoc vi 60ms chi la uoc tinh: app gui cham hon, hoac goi BT
  // bi cat vun, thi nang len; muon joystick nhay hon thi ha xuong.
  { "BTIDLE",  &bt_idle_ms,        PT_U8,    10.0f,   250.0f,    60.0f  },
  { "DRVSTEP", &bt_drive_step,     PT_F32,    0.05f,    3.0f,     0.4f  },
  { "DRVMAX",  &bt_drive_max,      PT_F32,    0.1f,     8.0f,     1.5f  },
  { "TRNSTEP", &bt_turn_step,      PT_I16,   10.0f,  1000.0f,   100.0f  },
  { "TRNMAX",  &bt_turn_max,       PT_I16,   10.0f,  2000.0f,   300.0f  },
  { "LOGMASK", &log_mask,          PT_U32,    0.0f, 16777215.0f, 255.0f },
  { "LOGDIV",  &log_div,           PT_U8,     1.0f,    20.0f,     1.0f  },
};
#define NUM_PARAMS  ((uint8_t)(sizeof(PARAMS) / sizeof(PARAMS[0])))

int8_t findParam(const char *name) {
  ParamDef pd;
  for (uint8_t i = 0; i < NUM_PARAMS; i++) {
    memcpy_P(&pd, &PARAMS[i], sizeof(pd));
    if (strcmp(name, pd.name) == 0) return (int8_t)i;
  }
  return -1;
}

float getParamVal(const ParamDef *pd) {
  switch (pd->type) {
    case PT_F32: return *(float *)pd->ptr;
    case PT_I16: return (float)(*(int16_t *)pd->ptr);
    case PT_I8:  return (float)(*(int8_t *)pd->ptr);
    case PT_U16: return (float)(*(uint16_t *)pd->ptr);
    case PT_U32: return (float)(*(uint32_t *)pd->ptr);
    default:     return (float)(*(uint8_t *)pd->ptr);
  }
}

void setParamVal(const ParamDef *pd, float v) {
  v = constrain(v, pd->lo, pd->hi);
  switch (pd->type) {
    case PT_F32: *(float *)pd->ptr   = v;                       break;
    case PT_I16: *(int16_t *)pd->ptr = (int16_t)lroundf(v);     break;
    case PT_I8:  *(int8_t *)pd->ptr  = (int8_t)lroundf(v);      break;
    case PT_U16: *(uint16_t *)pd->ptr = (uint16_t)lroundf(v);   break;
    case PT_U32: *(uint32_t *)pd->ptr = (uint32_t)lroundf(v);   break;
    default:     *(uint8_t *)pd->ptr = (uint8_t)lroundf(v);     break;
  }
}

// Tac dung phu bat buoc khi mot so tham so thay doi
void onParamSet(const char *name) {
  if      (strcmp(name, "DLPF") == 0) writeMPU(0x1A, dlpf_cfg);
  else if (strcmp(name, "LOGMASK") == 0) emitCols();
  else if (strcmp(name, "ARCH") == 0) { control_output = 0; pidI_angle = 0; pidI_vel = 0; }
  // VDIR dao DAU cua vel_err nen tich phan cu thanh sai dau -> bat buoc xoa.
  // BIAS thi khong: no chi doi setpoint, xoa them trim chi gay giat (xem lenh S).
  else if (strcmp(name, "VDIR") == 0) pidI_vel = 0;
}

// ============================================================
// EEPROM: [magic u16][float x NUM_PARAMS], theo dung thu tu bang
// ============================================================
//  MAGIC nhung NUM_PARAMS -> them/bot tham so tu dong vo hieu du lieu cu.
//  LUU Y: doi THU TU cac dong ma giu nguyen so luong thi KHONG tu phat hien
//  duoc -> sau khi doi thu tu, nho "reset" roi "save" lai.
#define EEPROM_MAGIC   ((uint16_t)(0xAB00 | NUM_PARAMS))
#define EEPROM_ADDR    0

void saveParams() {
  uint16_t magic = EEPROM_MAGIC;
  EEPROM.put(EEPROM_ADDR, magic);
  int addr = EEPROM_ADDR + sizeof(magic);
  ParamDef pd;
  for (uint8_t i = 0; i < NUM_PARAMS; i++) {
    memcpy_P(&pd, &PARAMS[i], sizeof(pd));
    float v = getParamVal(&pd);
    EEPROM.put(addr, v);
    addr += sizeof(float);
  }
  Serial.println(F("> SAVED to EEPROM"));
}

bool loadParams() {
  uint16_t magic;
  EEPROM.get(EEPROM_ADDR, magic);
  if (magic != EEPROM_MAGIC) return false;
  int addr = EEPROM_ADDR + sizeof(magic);
  ParamDef pd;
  for (uint8_t i = 0; i < NUM_PARAMS; i++) {
    memcpy_P(&pd, &PARAMS[i], sizeof(pd));
    float v;
    EEPROM.get(addr, v);
    setParamVal(&pd, v);
    addr += sizeof(float);
  }
  return true;
}

void resetParams() {
  ParamDef pd;
  for (uint8_t i = 0; i < NUM_PARAMS; i++) {
    memcpy_P(&pd, &PARAMS[i], sizeof(pd));
    setParamVal(&pd, pd.def);
  }
  pidI_vel = 0; pidI_angle = 0; control_output = 0;
}

void printParams() {                       // dai 1..NUM_PARAMS
  deferred_print = 1;
  deferred_end   = NUM_PARAMS;
}

void printVars() {                         // dai NUM_PARAMS+1..+NUM_VARS
  deferred_print = NUM_PARAMS + 1;
  deferred_end   = NUM_PARAMS + NUM_VARS;
}

// In 1 tham so moi TICK. Go het bang mat NUM_PARAMS tick (~220ms).
void doPrintStep() {
  uint8_t i = deferred_print - 1;
  if (i < NUM_PARAMS) {                    // --- mot dong tham so ---
    ParamDef pd;
    memcpy_P(&pd, &PARAMS[i], sizeof(pd));
    Serial.print(pd.name);
    Serial.print('=');
    Serial.print(getParamVal(&pd), 4);
    Serial.print(F(" ["));
    Serial.print(pd.lo, 2);
    Serial.print(F(".."));
    Serial.print(pd.hi, 2);
    Serial.println(']');
  } else {                                 // --- mot dong bien log ---
    uint8_t v = i - NUM_PARAMS;
    if (v < NUM_VARS) {
      VarDef vd;
      memcpy_P(&vd, &VARS[v], sizeof(vd));
      Serial.print((log_mask & ((uint32_t)1 << v)) ? F("[x] ") : F("[ ] "));
      Serial.println(vd.name);
    }
  }
  deferred_print++;
  if (deferred_print > deferred_end) deferred_print = 0;
}

// ============================================================
// BLUETOOTH COMMAND PARSER (line-buffered)
// ============================================================
// Protocol: moi lenh la 1 dong ket thuc bang '\n'
// Dieu khien: F B L R S (1 ky tu, khong can newline)
// Tham so:    KP 80.5 | KD 12 | KI 0 | KPV 0.008 | KIV 0.001
//             BIAS 0.5 | BIASADJ +0.3
//             SAVE | LOAD | RESET | GET

// Cong them vao lenh lai roi chan theo DRVMAX/TRNMAX da nhan app_speed.
// Tach ham rieng vi 8 nhanh lenh (F B L R G I H J) dung chung logic nay.
// app_speed chan BIEN DO, KHONG ghi de DRVMAX/TRNMAX — hai cai do la tham so
// luu EEPROM, lenh lai khong duoc phep sua.
static void driveAdd(float d) {
  float lim = bt_drive_max * app_speed;
  bt_drive = constrain(bt_drive + d, -lim, lim);
  Serial.print(F("> drive=")); Serial.println(bt_drive, 2);
}

static void turnAdd(int16_t d) {
  float lim = (float)bt_turn_max * app_speed;
  bt_turn = (int16_t)constrain((float)bt_turn + (float)d, -lim, lim);
  Serial.print(F("> turn=")); Serial.println(bt_turn);
}

void parseCommand(char* buf) {
  // Chuyen sang uppercase
  for (char* p = buf; *p; p++) *p = toupper(*p);

  // Tach keyword va value
  char* sp = strchr(buf, ' ');
  float val = 0;
  bool has_val = false;
  if (sp) { *sp = 0; val = atof(sp + 1); has_val = true; }

  // ---- 1. Lenh HANH DONG (khong phai tham so) ----
  // KHONG xoa pidI_vel o day: no la trim giu thang ma vong toc do da tich luy.
  // Do 25/10/2026: xoa no lam target tut tu 0.68 xuong 0.40, nghia la lenh "tien"
  // lai lam robot nga ve sau 0.28 do.
  if      (strcmp(buf, "F") == 0) { driveAdd(+bt_drive_step); }
  else if (strcmp(buf, "B") == 0) { driveAdd(-bt_drive_step); }
  else if (strcmp(buf, "L") == 0) { turnAdd(-bt_turn_step); }
  else if (strcmp(buf, "R") == 0) { turnAdd(+bt_turn_step); }
  // Nut cheo cua app: G=truoc-trai I=truoc-phai H=sau-trai J=sau-phai.
  // 'G' mot minh KHAC 'GET' vi strcmp so sanh ca chuoi, nen khong va cham.
  else if (strcmp(buf, "G") == 0) { driveAdd(+bt_drive_step); turnAdd(-bt_turn_step); }
  else if (strcmp(buf, "I") == 0) { driveAdd(+bt_drive_step); turnAdd(+bt_turn_step); }
  else if (strcmp(buf, "H") == 0) { driveAdd(-bt_drive_step); turnAdd(-bt_turn_step); }
  else if (strcmp(buf, "J") == 0) { driveAdd(-bt_drive_step); turnAdd(+bt_turn_step); }
  // Thanh truot toc do cua app: mot chu so '0'..'9'
  else if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == 0) {
    app_speed = 0.1f + 0.1f * (float)(buf[0] - '0');   // '0'->0.1 ... '9'->1.0
    driveAdd(0.0f); turnAdd(0);                        // ap tran moi ngay lap tuc
    Serial.print(F("> app_speed=")); Serial.println(app_speed, 2);
  }
  // KHONG xoa pidI_vel: no la trim giu thang ma vong toc do da tich luy duoc.
  // DO 26/10/2026: bam nut dung luc dang can bang -> target tut ngay mot luong
  // Ki_vel*pidI_vel (ivel hay o +-500, tuc 0.5 do) -> robot troi roi nga, va
  // phai hang chuc giay moi bo lai duoc vi Ki_vel=0.001 chay o 10Hz.
  else if (strcmp(buf, "S") == 0) { bt_drive = 0; bt_turn = 0;
    noInterrupts(); pos_target = (stepsL + stepsR) / 2; interrupts();  // giu cho dang dung
    Serial.println(F("> STOP")); }
  else if (strcmp(buf, "VER") == 0) {
    Serial.print(F("> VER=")); Serial.println(F(FW_VERSION));
  }
  else if (strcmp(buf, "P") == 0 || strcmp(buf, "GET") == 0) { printParams(); }
  // BIASADJ la lenh TUONG DOI nen khong nam trong bang
  else if (strcmp(buf, "BIASADJ") == 0) {
    target_bias += val;   // khong xoa pidI_vel — xem ghi chu o lenh S
    Serial.print(F("> BIAS=")); Serial.println(target_bias, 4);
  }
  else if (strcmp(buf, "SAVE") == 0) {
    if (fallen) saveParams();
    else Serial.println(F("> SAVE: nghieng robot xuong truoc (EEPROM block 150ms)"));
  }
  else if (strcmp(buf, "LOAD") == 0) {
    if (loadParams()) { writeMPU(0x1A, dlpf_cfg); Serial.println(F("> LOADED")); printParams(); }
    else Serial.println(F("> No saved data"));
  }
  else if (strcmp(buf, "RESET") == 0) {
    resetParams(); writeMPU(0x1A, dlpf_cfg);
    Serial.println(F("> RESET to defaults"));
  }
  // Dat THANG gia tri lai, thay vi nhan nut tung nac F/B/L/R
  else if (strcmp(buf, "DRIVE") == 0) {
    bt_drive = constrain(val, -bt_drive_max, bt_drive_max);
    Serial.print(F("> drive=")); Serial.println(bt_drive, 2);
  }
  else if (strcmp(buf, "TURN") == 0) {
    bt_turn = (int16_t)constrain(val, -bt_turn_max, bt_turn_max);
    Serial.print(F("> turn=")); Serial.println(bt_turn);
  }
  else if (strcmp(buf, "LOG") == 0) {
    if (sp) {                              // "LOG GYRO" -> bat/tat field do
      int8_t vi = findVar(sp + 1);
      if (vi < 0) { Serial.print(F("? var ")); Serial.println(sp + 1); }
      else { log_mask ^= ((uint32_t)1 << vi); emitCols(); }
    } else {
      printVars();                         // "LOG" -> liet ke
    }
  }
  else if (strcmp(buf, "LOGALL") == 0)  { log_mask = (uint32_t)((1UL << NUM_VARS) - 1); emitCols(); }
  else if (strcmp(buf, "LOGNONE") == 0) { log_mask = 0; emitCols(); }
  else if (strcmp(buf, "COLS") == 0)    { emitCols(); }
  // CAL = chi gyro bias. Robot NAM cung chay duoc, chi can dung yen.
  else if (strcmp(buf, "CAL") == 0) {
    if (fallen) { calibrateGyro(); Serial.println(F("> gyro recalibrated")); }
    else Serial.println(F("> CAL: nghieng robot xuong truoc (block 4s)"));
  }
  // CALUP = do lai goc lap MPU. PHAI cam robot THANG DUNG THAT.
  // Do xong nho 'save' de ghi EEPROM, tu do khong phai do lai nua.
  else if (strcmp(buf, "CALUP") == 0) {
    if (fallen) { calibrateUpright(); angle = 0; }
    else Serial.println(F("> CALUP: nghieng robot xuong truoc (block 4s)"));
  }
  // ---- 2. Con lai: tra BANG THAM SO ----
  else {
    int8_t idx = findParam(buf);
    if (idx < 0) { Serial.print(F("? ")); Serial.println(buf); return; }
    ParamDef pd;
    memcpy_P(&pd, &PARAMS[idx], sizeof(pd));
    if (has_val) {                 // co gia tri -> dat
      setParamVal(&pd, val);
      onParamSet(pd.name);
    }                              // khong co -> chi doc lai
    Serial.print(F("> "));
    Serial.print(pd.name);
    Serial.print('=');
    Serial.println(getParamVal(&pd), 4);
  }
}

// Ky tu ma app dieu khien gui don le: 4 huong, 4 huong cheo, stop, va thanh truot.
static bool isJoyChar(char c) {
  return strchr("FBLRSGIHJ", c) != NULL || (c >= '0' && c <= '9');
}

void handleBTCommand() {
  static char buf[32];
  static uint8_t pos = 0;

  static uint32_t last_rx = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    last_rx = millis();

    // Tat ca lenh deu buffer den newline (tranh 'B' cua BIAS bi parse la backward)
    if (c == '\n' || c == '\r') {
      if (pos > 0) {
        buf[pos] = 0;
        parseCommand(buf);
        pos = 0;
      }
    } else if (pos == 1 && c == buf[0] && isJoyChar(c)) {
      // App giu nut thi gui lap lien tuc, co the nhanh hon bt_idle_ms, luc do hai
      // ky tu dinh vao nhau thanh "FF" — khong phai lenh nao ca. Ky tu lai LAP LAI
      // chinh no thi chac chan la hai lan bam, khong co lenh nao bat dau bang "FF"
      // hay "55". Chot cai truoc roi bat dau cai moi.
      // Chi xet khi GIONG HET nhau: "BIAS" tu PC la 'B' roi 'I', hai ky tu lai
      // KHAC nhau, nen khong bi cat nham thanh backward + "IAS".
      buf[1] = 0;
      parseCommand(buf);
      buf[0] = c;
      pos = 1;
    } else if (pos < 31) {
      buf[pos++] = c;
    } else {
      pos = 0;  // overflow, reset
    }
  }

  // --- Chot lenh khi duong truyen im (cho app dien thoai) ---
  // App kieu Arduino BT Control gui 'F' tran trui, khong newline, nen neu chi cho
  // '\n' thi lenh khong bao gio chay. Khong the phan biet bang NOI DUNG: 'B' vua la
  // backward vua la chu dau cua BIAS, 'S' vua la stop vua la chu dau cua SAVE —
  // chinh va cham nay la ly do kieu parse 1 ky tu da bi bo truoc day.
  // Phan biet bang THOI GIAN:
  //   PC gui tron ca dong trong mot sendall -> o 115200 baud 10 ky tu het <1ms,
  //   nen khoang trong GIUA cac ky tu cua mot tu gan nhu khong ton tai.
  //   App gui mot ky tu roi im hang tram ms.
  // Qua bt_idle_ms khong co ky tu moi -> coi nhu het lenh.
  // Xau nhat (goi BT bi cat dung giua tu) chi la mot dong '? XY', vo hai.
  if (pos > 0 && (uint32_t)(millis() - last_rx) > (uint32_t)bt_idle_ms) {
    buf[pos] = 0;
    parseCommand(buf);
    pos = 0;
  }
}

// ============================================================
// LOOP - 100Hz fixed rate
// ============================================================
uint8_t  vel_counter  = 0;

void loop() {
  // Xu ly lenh BT moi vong (khong block)
  handleBTCommand();

  // === Cho den next loop tick (100Hz) ===
  unsigned long now = micros();
  unsigned long elapsed = now - last_loop_us;
  if (elapsed < LOOP_PERIOD_US) return;
  dt = elapsed / 1000000.0f;
  last_loop_us = now;
  loop_ms = (long)millis();        // de PC co moc thoi gian that, khong bi BT lam nhoe

  // In params 1 dong moi TICK (100Hz) - PHAI nam DUOI cong 100Hz.
  // Truoc day no nam TREN cong, ma loop() quay hang nghin lan/10ms
  // -> ca 6 dong xa ra trong vai us -> day TX buffer 64B -> block ~15ms.
  if (deferred_print > 0) doPrintStep();

  // === 1. Doc MPU + Complementary filter ===
  int16_t ax, ay, az, gx, gy, gz;
  readMPU(&ax, &ay, &az, &gx, &gy, &gz);
  gyro_rate = -((float)gx - gyroBias) / 131.0f;
  float accel_raw     = calcAccelAngleRaw(ay, az) - upright_offset;
  // Software low-pass tren accel de bot motor vibration noise
  accel_angle = accel_lpf * accel_angle + (1.0f - accel_lpf) * accel_raw;
  angle = cf_alpha * (angle + gyro_rate * dt) + (1.0f - cf_alpha) * accel_angle;

  // Soft resync: khi CF va accel lech >3° thi keo nhe CF ve phia accel
  // Ngan gyro drift tich luy lau dai ma khong lam nhay cam voi motor noise
  float angle_drift = accel_angle - angle;
  if (fabsf(angle_drift) > resync_thresh) {
    angle += resync_gain * angle_drift;
  }

  // === 2. An toan ===
  // Check CA filtered angle VA raw accel angle (de tranh filter drift bypass safety)
  // Neu accel raw bao nghieng > FALL thi 100% robot dang nga, du filter chua catch up
  bool filter_fall = (fabsf(angle) > fall_angle);
  bool accel_fall  = (fabsf(accel_angle) > fall_angle);
  if (filter_fall || accel_fall) {
    if (!fallen) {
      fallen = true;
      stopMotors();
      disableMotors();
      pidI_angle  = 0;
      pidI_vel    = 0;
      target_angle = 0;
      control_output = 0;
      // Neu accel phat hien -> filter da drift, force re-sync angle = accel
      if (accel_fall && !filter_fall) {
        Serial.println(F("[!] FALLEN (filter drift! resync angle)"));
        angle = accel_angle;  // Reset filter ve true angle
      } else {
        Serial.println(F("[!] FALLEN"));
      }
    }
    // Van phat telemetry khi NAM. Truoc day nhanh nay return truoc doan in,
    // nen PC khong phan biet duoc "robot dang nam" voi "mat ket noi" — da hai
    // lan ket luan nham trong buoi 25/10.
    if (++log_cnt >= log_div) { log_cnt = 0; emitTelemetry(); }
    return;
  }
  if (fallen) {
    if (fabsf(angle) < recover_angle && fabsf(gyro_rate) < recover_gyro) {
      fallen = false;
      noInterrupts(); stepsL = 0; stepsR = 0; interrupts();
      last_steps_avg = 0;
      pos_target     = 0;    // moc vi tri moi = cho vua dung day
      soft_start_cnt = (uint16_t)soft_start_loops;  // ramp Kp tu 50% de tranh overshoot post-recovery
      enableMotors();
      Serial.println(F("[+] RECOVERED"));
    } else {
      if (++log_cnt >= log_div) { log_cnt = 0; emitTelemetry(); }
      return;
    }
  }

  // === 3. VONG NGOAI: Velocity PID -> dieu chinh target_angle (10Hz) ===
  if (++vel_counter >= VEL_LOOP_DIVIDER) {
    vel_counter = 0;

    // Doc step counters atomic
    noInterrupts();
    long stepsAvg = (stepsL + stepsR) / 2;
    interrupts();

    // Van toc = delta steps trong 100ms (don vi: steps/100ms)
    long velocity = stepsAvg - last_steps_avg;
    last_steps_avg = stepsAvg;

    // Low-pass filter velocity de bot transient spike
    vel_filtered = vel_filter_alpha * vel_filtered
                 + (1.0f - vel_filter_alpha) * (float)velocity;

    // Sai so van toc: muon = 0
    // --- Vong VI TRI: sinh vel_setpoint keo robot ve pos_target ---
    // Dang co lenh lai -> keo moc theo robot, khong chong lai nguoi lai.
    if (fabsf(bt_drive) > 0.01f || bt_turn != 0) pos_target = stepsAvg;
    float vel_setpoint = 0.0f;
    if (fabsf(bt_drive) > 0.01f) {
      // bt_drive la lenh TOC DO BANH (steps/100ms), KHONG phai lenh goc nghieng.
      // DO 25/10/2026: cong thang bt_drive vao target_angle thi vo dung. Trong tam
      // chi cao 59mm so voi mat dat, nen nghieng 1 do chi sinh ~10 mN.m quanh diem
      // tiep xuc — nho hon momen detent cua NEMA17 (10-16 mN.m) nen banh khong lan.
      // Do duoc: DRIVE 1.0 -> di 0.10cm trong 3s; phai toi DRIVE 2.5 moi nhuc nhich.
      // Dat thang vao vel_setpoint thi vong toc do tu nghieng du bao nhieu cung duoc.
      vel_setpoint = bt_drive * drive_vel_gain;
    } else if (Kp_pos > 0.0001f) {
      float pos_err = (float)(stepsAvg - pos_target);
      vel_setpoint  = -Kp_pos * pos_err;
    }
    // Chan o CA HAI nhanh. VMAX la tran toc do ma robot thuc su tang toc toi duoc
    // ma khong nga; khong co ly do gi de lenh lai duoc phep vuot qua no.
    vel_setpoint = constrain(vel_setpoint, -vel_cmd_max, vel_cmd_max);
    vel_err = (vel_setpoint - vel_filtered) * vel_dir;
    // Anti-windup: khong tich luy khi dang XOAY. Do 25/10/2026: xoay 2 giay lam
    // pidI_vel cuon toi -492/500, va no con nguyen do sau khi xoay xong roi day
    // target_angle lech -> robot trot 8cm. Con bt_drive thi VAN phai tich luy,
    // vi gio chinh tich phan nay la thu sinh ra goc nghieng de dat toc do yeu cau.
    if (fabsf(angle) < awin_angle && bt_turn == 0) {
      pidI_vel += vel_err;
      pidI_vel  = constrain(pidI_vel, -500.0f, 500.0f);
    }

    target_angle = target_bias + Kp_vel * vel_err + Ki_vel * pidI_vel;
    target_angle = constrain(target_angle, -max_target_offset - 2.0f, max_target_offset + 2.0f);
  }

  // === 4. VONG TRONG: Angle PID (100Hz) ===
  float angle_err = target_angle - angle;
  pidI_angle += angle_err * dt;
  pidI_angle  = constrain(pidI_angle, -200.0f, 200.0f);

  // D-term: dung gyro_rate truc tiep (it nhieu hon sai phan goc)
  // d(error)/dt = -d(angle)/dt = -gyro_rate
  float d_term = -gyro_rate;

  // Soft-start: ramp Kp tu 50%->100% sau recovery, tranh overshoot khi moi phuc hoi
  float kp_eff = Kp_angle;
  if (soft_start_cnt > 0) {
    float t = 1.0f - (float)soft_start_cnt / (float)soft_start_loops;
    kp_eff = Kp_angle * (0.5f + 0.5f * t);
    soft_start_cnt--;
  }

  pid_out = kp_eff * angle_err
                + Ki_angle * pidI_angle
                + Kd_angle * d_term;

  // === 5. Output -> motor (voi acceleration ramping) ===
  if (integrate_out) {
    // arch 1 (B-ROBOT EVO2): pid_out la lenh GIA TOC -> tich phan ra toc do.
    // Con lac nguoc phan ung voi GIA TOC banh, nen day moi la dang chuan.
    control_output += pid_out;
  } else {
    // arch 0: pid_out CHINH LA toc do (ban goc cua du an nay)
    control_output = pid_out;
  }
  control_output = constrain(control_output,
                             -(float)max_steps_per_sec,
                             +(float)max_steps_per_sec);

  // Lenh lai cong THANG vao toc do banh, khong di qua target_angle.
  // DO 26/10/2026: giu nghieng on dinh 3.16 do -> robot di 0.15cm trong 4 giay,
  // tuc dung yen. Vong trong bam target rat chinh xac nen o trang thai xac lap
  // no khong ra lenh buoc nao; luc do banh dung yen NHUNG MOTOR DANG CO DIEN va
  // giu vi tri voi momen 0.18 N.m, trong khi momen lat o 3.16 do chi 32 mN.m.
  // Phai toi ~18 do trong luc moi thang duoc, tuc robot nga truoc khi lan.
  // => Nghieng khong bao gio lai duoc robot nay. bt_turn chay tot suot tu dau
  //    chinh vi no cong thang vao lenh motor; lai phai di cung duong do.
  // DRVVEL giu nguyen y nghia: DRIVE 1.0 = 20 steps/100ms = 200 step/s.
  float drive_sp = bt_drive * drive_vel_gain * 10.0f;    // steps/100ms -> step/s
  setMotorSpeed((int16_t)constrain(control_output + drive_sp,
                                   -(float)max_steps_per_sec,
                                   +(float)max_steps_per_sec));

  // === 6. Telemetry (bang VARS + log_mask, xem BANG BIEN LOG) ===
  if (++log_cnt >= log_div) {
    log_cnt = 0;
    emitTelemetry();
  }

}
