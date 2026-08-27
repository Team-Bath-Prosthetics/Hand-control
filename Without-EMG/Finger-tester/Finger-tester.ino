// N20 motor - simple open/close test
const int IN1 = 5;
const int IN2 = 6;
const int ENA = 9;

int testSpeed = 700; // 0-255

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  Serial.println("Ready. 'o' = open, 'c' = close, 'x' = stop");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'o') {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, testSpeed);
      Serial.println("Opening");
    }
    else if (cmd == 'c') {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, testSpeed);
      Serial.println("Closing");
    }
    else if (cmd == 'x') {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      Serial.println("Stopped");
    }
  }
}