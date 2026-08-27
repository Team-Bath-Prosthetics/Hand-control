// Bidirectional N20 motor test - L298N driver
const int IN1 = 5;
const int IN2 = 6;
const int ENA = 9;

void driveMotor(int speed) { // speed: -255 to 255
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW); // coast; HIGH,HIGH = brake
  }
  analogWrite(ENA, abs(speed));
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
}

void loop() {
  Serial.println("Forward ramp");
  for (int s = 0; s <= 255; s += 5) { driveMotor(s); delay(30); }
  delay(500);

  Serial.println("Stop");
  driveMotor(0);
  delay(500);

  Serial.println("Reverse ramp");
  for (int s = 0; s >= -255; s -= 5) { driveMotor(s); delay(30); }
  delay(500);

  Serial.println("Stop");
  driveMotor(0);
  delay(1000);
}