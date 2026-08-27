struct Finger {
  int in1, in2, ena;
};

Finger fingers[4] = {
  {2, 3, 10},   // Finger 1
  {4, 5, 11},   // Finger 2
  {6, 7, 12},   // Finger 3
  {8, 9, 13}    // Finger 4
};

int testSpeed = 150;

void openFinger(int i) {
  digitalWrite(fingers[i].in1, HIGH);
  digitalWrite(fingers[i].in2, LOW);
  analogWrite(fingers[i].ena, testSpeed);
  Serial.print("Finger "); Serial.print(i + 1); Serial.println(" opening");
}

void closeFinger(int i) {
  digitalWrite(fingers[i].in1, LOW);
  digitalWrite(fingers[i].in2, HIGH);
  analogWrite(fingers[i].ena, testSpeed);
  Serial.print("Finger "); Serial.print(i + 1); Serial.println(" closing");
}

void stopFinger(int i) {
  digitalWrite(fingers[i].in1, LOW);
  digitalWrite(fingers[i].in2, LOW);
  analogWrite(fingers[i].ena, 0);
}

void stopAll() {
  for (int i = 0; i < 4; i++) stopFinger(i);
  Serial.println("All fingers stopped");
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(fingers[i].in1, OUTPUT);
    pinMode(fingers[i].in2, OUTPUT);
    pinMode(fingers[i].ena, OUTPUT);
    stopFinger(i);
  }
  Serial.println("Ready. Keys 1-8: odd=open, even=close. '0' = stop all.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1': openFinger(0); break;
      case '2': closeFinger(0); break;
      case '3': openFinger(1); break;
      case '4': closeFinger(1); break;
      case '5': openFinger(2); break;
      case '6': closeFinger(2); break;
      case '7': openFinger(3); break;
      case '8': closeFinger(3); break;
      case '0': stopAll(); break;
    }


  }
}



const int SENSOR_PIN = A0;

// float envelope = 0;
int threshold = 200;  // placeholder — retune once you see real numbers

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT); /
}

void loop() {
    int finger_signal = analog.read(SENSOR_PIN);

    if (finger_signal>threshold){
      openFinger(0)
    } 
}