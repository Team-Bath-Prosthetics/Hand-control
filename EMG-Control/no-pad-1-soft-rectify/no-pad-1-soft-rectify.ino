const int SENSOR_PIN = A0;

float baseline = 2048;   // tracks the slow-moving resting level
float envelope = 0;      // the smoothed "how hard is it flexing" value
int threshold = 300;     // placeholder — re-tune after watching the plotter

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  int raw = analogRead(SENSOR_PIN);

  baseline += (raw - baseline) * 0.001;      // slowly follow DC drift
  float rectified = abs(raw - baseline);     // flip the negative swings positive
  envelope += (rectified - envelope) * 0.05; // smooth it into a rising/falling level

  Serial.print(envelope);
  Serial.print(",");
  Serial.println(threshold);

  digitalWrite(LED_BUILTIN, envelope > threshold ? HIGH : LOW);
  delayMicroseconds(500);  // sample much faster than before — needed to catch the wiggle
}