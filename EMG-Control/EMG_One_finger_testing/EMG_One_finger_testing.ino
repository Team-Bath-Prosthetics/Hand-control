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
const int SENSOR_PIN = A0;
int threshold_high = 220;  // placeholder — retune once you see real numbers
int threshold_low = 140;

unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 50;

bool fingersActive = false;   // true = EMG has triggered and not yet released
bool emgCloseMode = true;     // true = threshold crossing closes; false = threshold crossing opens

void openFinger(int i) {
  digitalWrite(fingers[i].in1, HIGH);
  digitalWrite(fingers[i].in2, LOW);
  analogWrite(fingers[i].ena, testSpeed);
}

void closeFinger(int i) {
  digitalWrite(fingers[i].in1, LOW);
  digitalWrite(fingers[i].in2, HIGH);
  analogWrite(fingers[i].ena, testSpeed);
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

void closeAll() {
  for (int i = 0; i < 4; i++) closeFinger(i);
  Serial.println("All fingers closing");
}

void openAll() {
  for (int i = 0; i < 4; i++) openFinger(i);
  Serial.println("All fingers opening");
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(fingers[i].in1, OUTPUT);
    pinMode(fingers[i].in2, OUTPUT);
    pinMode(fingers[i].ena, OUTPUT);
    stopFinger(i);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Ready. Keys 1-8: odd=open, even=close. '0'=stop all. 'A'=open all. 's'=toggle EMG mode.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1': openFinger(0); fingersActive = false; break;
      case '2': closeFinger(0); fingersActive = false; break;
      case '3': openFinger(1); break;
      case '4': closeFinger(1); break;
      case '5': openFinger(2); break;
      case '6': closeFinger(2); break;
      case '7': openFinger(3); break;
      case '8': closeFinger(3); break;
      case '0':
        stopAll();
        fingersActive = false;
        break;
      case 'A':
        openAll();
        fingersActive = false;
        break;
      case 's':
        emgCloseMode = !emgCloseMode;
        Serial.print("EMG mode: ");
        Serial.println(emgCloseMode ? "close on trigger" : "open on trigger");
        break;
    }
  }

  int finger_signal = analogRead(SENSOR_PIN);



  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    Serial.println(finger_signal);
  }

  if (!fingersActive) {
    // idle: watch for the ON threshold
    if (finger_signal > threshold_high) {
      if (emgCloseMode) closeFinger(0); else openFinger(0);
      fingersActive = true;
    }
  } else {
    // driving: watch for the OFF threshold
    if (finger_signal < threshold_low) {
      stopAll();
      fingersActive = false;
    }
  }
}