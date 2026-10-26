// Passthrough: relay USB <-> HC-05 (cung Hardware Serial D0/D1)
// Dung de gui AT commands den HC-05 qua COM port
void setup() { Serial.begin(38400); }
void loop()  { while (Serial.available()) Serial.write(Serial.read()); }
