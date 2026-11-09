#include <Arduino.h>

const int SENSOR_PIN = A0;

//float envelope = 0;
int threshold = 200;  // placeholder — retune once you see real numbers

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT); 
}

void loop() {
  int reading = analogRead(SENSOR_PIN);

// envelope value was not found 
//  Serial.print(envelope);
  Serial.print(",");
  Serial.println(reading);

  digitalWrite(LED_BUILTIN, reading > threshold ? HIGH : LOW);
  delay(5);
}
