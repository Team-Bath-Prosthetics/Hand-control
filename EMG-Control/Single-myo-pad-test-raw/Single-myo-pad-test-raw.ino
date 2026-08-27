const int SENSOR_PIN = A0;
int threshold = 4000;  // will need re-tuning, see note below

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  int reading = analogRead(SENSOR_PIN);

  Serial.print(reading);
  Serial.print(",");
  Serial.println(threshold);

  digitalWrite(LED_BUILTIN, reading > threshold ? HIGH : LOW);
  delay(20);
}