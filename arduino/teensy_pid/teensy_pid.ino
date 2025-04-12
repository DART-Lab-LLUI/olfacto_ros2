void setup() {
  Serial.begin(500000);
  analogReadResolution(12);
}

void loop() {
  static uint32_t lastMicros = 0;
  const uint32_t interval = 100;  // 1000 µs = 1 ms = 1000 Hz

  if (micros() - lastMicros >= interval) {
    lastMicros += interval;
    int val = analogRead(A0);
    Serial.write((uint8_t *)&val, 2);
    //Serial.println(val);
  }
}
