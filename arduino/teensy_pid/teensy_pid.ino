const uint32_t interval = 100;  // 100 µs = 0.1 ms = 10000 Hz
const int samplesPerBatch = 20; // amount of samples to average 20 -> 500Hz

int sum = 0;
int samples = 0;
uint32_t lastMicros = 0;

void setup() {
  Serial.begin(500000);
  analogReadResolution(12);
}

void loop() {
  if (micros() - lastMicros >= interval) {
    lastMicros += interval;
    int val = analogRead(A0);

    sum += val;
    samples += 1;

    if (samples >= samplesPerBatch) {
      uint32_t timestamp = micros();
      int avg = sum / samplesPerBatch;

      Serial.print(timestamp);
      Serial.print(",");
      Serial.println(avg);

      sum = 0;
      samples = 0;
    }
  }
}

