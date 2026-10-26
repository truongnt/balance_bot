// Upload sketch nay, sau do:
//   1. Giu KEY tren HC-05
//   2. Power cycle robot (ca Arduino + HC-05)
//   3. Arduino tu dong gui AT command -> HC-05 doi baud
//   4. Den LED nhay 3 lan = xong
//   5. Upload lai robot_balance firmware
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(38400);
  delay(1500);  // cho HC-05 khoi dong xong AT mode

  Serial.print(F("AT\r\n"));         delay(300);
  Serial.print(F("AT+UART=115200,0,0\r\n")); delay(500);

  // Nhay den 3 lan = da gui xong
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(200);
    digitalWrite(LED_BUILTIN, LOW);  delay(200);
  }
  // Nhay lien tuc cho biet dang cho
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH); delay(800);
    digitalWrite(LED_BUILTIN, LOW);  delay(800);
  }
}
void loop() {}
