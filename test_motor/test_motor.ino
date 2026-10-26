/*
 * TEST MOTOR — Kiểm tra chiều quay 2 motor NEMA17
 * ================================================
 * Upload file này trước khi chạy code cân bằng
 * để xác nhận motor trái/phải quay đúng chiều.
 *
 * Quan sát:
 *   - "TIEN": cả 2 bánh phải quay về phía TRƯỚC robot
 *   - "LUI" : cả 2 bánh phải quay về phía SAU  robot
 *
 * Nếu 1 bánh quay ngược → đổi 2 dây cuộn A (hoặc cuộn B)
 * của motor đó trên A4988 (đổi 1A↔1B hoặc 2A↔2B)
 *
 * Jumper microstepping phải khớp với #define MICROSTEP bên dưới
 */

// ── Chân (giống file chính) ──────────────────────
#define STEP_L      2
#define STEP_R      3
#define DIR_L       5
#define DIR_R       6
#define ENABLE_PIN  8

// ── Cấu hình ────────────────────────────────────
#define MICROSTEP   8       // Khớp với jumper MS1/MS2 trên CNC Shield
#define STEP_SPEED  800     // Steps/giây (~vừa mắt để quan sát)

// ── Biến stepper ────────────────────────────────
unsigned long lastStepL = 0;
unsigned long lastStepR = 0;
bool stepStateL = false;
bool stepStateR = false;
long stepInterval = 0;

// ────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(STEP_L,    OUTPUT);
  pinMode(STEP_R,    OUTPUT);
  pinMode(DIR_L,     OUTPUT);
  pinMode(DIR_R,     OUTPUT);
  pinMode(ENABLE_PIN,OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);   // Bật driver

  stepInterval = 1000000L / STEP_SPEED;

  Serial.println(F("=== TEST MOTOR ==="));
  Serial.println(F("Quan sat 2 banh xe:"));
  Serial.println(F("  TIEN -> 2 banh quay cung chieu ve phia truoc"));
  Serial.println(F("  LUI  -> 2 banh quay cung chieu ve phia sau"));
  Serial.println(F("Neu 1 banh nguoc -> doi day cuon A tren A4988 do"));
}

// ────────────────────────────────────────────────
void loop() {

  // ── 1. Chạy TIẾN 3 giây ──────────────────────
  Serial.println(F("\n>> TIEN (3 giay)..."));
  setDir(true);
  runFor(3000);

  // ── 2. Dừng 1 giây ───────────────────────────
  Serial.println(F(">> DUNG"));
  stopMotors();
  delay(1000);

  // ── 3. Chạy LÙI 3 giây ───────────────────────
  Serial.println(F(">> LUI (3 giay)..."));
  setDir(false);
  runFor(3000);

  // ── 4. Dừng 1 giây ───────────────────────────
  Serial.println(F(">> DUNG"));
  stopMotors();
  delay(1000);

  // ── 5. Test từng motor riêng ──────────────────
  Serial.println(F("\n>> CHI MOTOR TRAI (2 giay)..."));
  setDir(true);
  runLeftOnly(2000);
  stopMotors();
  delay(800);

  Serial.println(F(">> CHI MOTOR PHAI (2 giay)..."));
  setDir(true);
  runRightOnly(2000);
  stopMotors();
  delay(1500);
}

// ────────────────────────────────────────────────
// Đặt chiều: forward=true → tiến
void setDir(bool forward) {
  // Motor trái
  digitalWrite(DIR_L, forward ? HIGH : LOW);
  // Motor phải: gắn đối xứng nên đảo chiều
  digitalWrite(DIR_R, forward ? LOW  : HIGH);
}

// Chạy cả 2 motor trong ms milliseconds
void runFor(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    pulse();
  }
}

// Chạy chỉ motor trái
void runLeftOnly(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    unsigned long now = micros();
    if (now - lastStepL >= (unsigned long)stepInterval) {
      stepStateL = !stepStateL;
      digitalWrite(STEP_L, stepStateL ? HIGH : LOW);
      lastStepL = now;
    }
  }
}

// Chạy chỉ motor phải
void runRightOnly(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    unsigned long now = micros();
    if (now - lastStepR >= (unsigned long)stepInterval) {
      stepStateR = !stepStateR;
      digitalWrite(STEP_R, stepStateR ? HIGH : LOW);
      lastStepR = now;
    }
  }
}

// Tạo xung cho cả 2 motor
void pulse() {
  unsigned long now = micros();
  if (now - lastStepL >= (unsigned long)stepInterval) {
    stepStateL = !stepStateL;
    digitalWrite(STEP_L, stepStateL ? HIGH : LOW);
    lastStepL = now;
  }
  if (now - lastStepR >= (unsigned long)stepInterval) {
    stepStateR = !stepStateR;
    digitalWrite(STEP_R, stepStateR ? HIGH : LOW);
    lastStepR = now;
  }
}

void stopMotors() {
  stepInterval = 1000000L / STEP_SPEED; // reset lại
}
