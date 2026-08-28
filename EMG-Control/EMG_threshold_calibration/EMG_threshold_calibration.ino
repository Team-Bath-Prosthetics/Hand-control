/*
  EMG_threshold_calibration.ino
  ------------------------------------------------------------------
  Standalone calibration routine for the MyoWare 2.0 envelope output.
  No motor driving - thresholds only. Variable names match the main
  finger-control sketch so this drops straight in later.

  Board:  Arduino Nano ESP32
  Sensor: MyoWare 2.0 ENV output -> A0 (GPIO1, ADC1)

  Serial Monitor MUST be set to "New Line" (or "Both NL & CR").
  Commands are read as whole lines, not single characters.

  Commands:
    calibrate  - run the two-stage calibration
    t          - print current thresholds
    m          - mute / unmute the live value stream
*/

// ---------------------------------------------------------------- config

const int SENSOR_PIN = A0;

// Live thresholds. Defaults are the hand-tuned values that worked in testing.
int threshold_low  = 140;
int threshold_high = 220;

// Calibration parameters
const unsigned long CAL_CAPTURE_MS = 5000;   // length of each capture window
const unsigned long CAL_SAMPLE_MS  = 2;      // ~500 Hz during capture
const float LOW_FRACTION   = 0.10f;
const float HIGH_FRACTION  = 0.20f;
const int   MIN_VALID_SPAN = 100;            // max must exceed rest by this

// Set true to reference thresholds to the span above rest instead of to
// raw max. See the note at computeThresholds().
const bool USE_REST_REFERENCED = false;

// Live stream throttle
const unsigned long PRINT_INTERVAL_MS = 50;
unsigned long lastPrintMs = 0;
bool streaming = true;

// Calibration results, kept for inspection
int emg_rest_avg = 0;
int emg_max_avg  = 0;
int emg_max_peak = 0;

// ------------------------------------------------------------ serial i/o

String serialBuffer = "";

// Non-blocking. Returns true once a complete line has arrived, trimmed
// and lower-cased into 'out'.
bool readLine(String &out) {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        out = serialBuffer;
        out.trim();
        out.toLowerCase();
        serialBuffer = "";
        return true;
      }
      // bare terminator, ignore
    } else {
      serialBuffer += c;
    }
  }
  return false;
}

// Blocking wrapper, used only inside calibration.
String waitForLine() {
  String line;
  while (!readLine(line)) {
    delay(1);
  }
  return line;
}

// --------------------------------------------------------------- capture

struct Capture {
  int  mean;
  int  peak;
  int  floorVal;
  long samples;
};

Capture captureWindow(unsigned long durationMs) {
  unsigned long start = millis();
  long sum = 0;
  long n   = 0;
  int  peak = 0;
  int  floorVal = 32767;
  int  lastSecond = -1;

  while (millis() - start < durationMs) {
    int v = analogRead(SENSOR_PIN);
    sum += v;
    n++;
    if (v > peak)     peak = v;
    if (v < floorVal) floorVal = v;

    // countdown so the user knows it is still running
    int secondsElapsed = (millis() - start) / 1000;
    if (secondsElapsed != lastSecond) {
      lastSecond = secondsElapsed;
      Serial.print(durationMs / 1000 - secondsElapsed);
      Serial.print("... ");
    }
    delay(CAL_SAMPLE_MS);
  }
  Serial.println();

  Capture c;
  c.mean     = (n > 0) ? (int)(sum / n) : 0;
  c.peak     = peak;
  c.floorVal = floorVal;
  c.samples  = n;
  return c;
}

void reportCapture(const char *label, Capture c) {
  Serial.print(label);
  Serial.print(" -> mean ");  Serial.print(c.mean);
  Serial.print(" | min ");    Serial.print(c.floorVal);
  Serial.print(" | peak ");   Serial.print(c.peak);
  Serial.print(" | n=");      Serial.println(c.samples);
}

// ----------------------------------------------------------- calibration

/*
  Two ways to turn the captures into thresholds.

  Fraction of max (as specified):
      low  = LOW_FRACTION  * max
      high = HIGH_FRACTION * max
  This assumes the ADC reads ~0 with the muscle slack. The MyoWare
  envelope does not - it idles around 90 counts - so on a mean max of
  870 this gives low = 87, which sits BELOW rest. The state machine
  would then latch on and never release.

  Rest-referenced:
      low  = rest + LOW_FRACTION  * (max - rest)
      high = rest + HIGH_FRACTION * (max - rest)
  Same numbers give 168 / 246, which is close to the 140 / 220 found by
  hand. Flip USE_REST_REFERENCED to compare them on your own arm.
*/
void computeThresholds(int rest, int maxVal) {
  if (USE_REST_REFERENCED) {
    int span = maxVal - rest;
    threshold_low  = rest + (int)(LOW_FRACTION  * span);
    threshold_high = rest + (int)(HIGH_FRACTION * span);
  } else {
    threshold_low  = (int)(LOW_FRACTION  * maxVal);
    threshold_high = (int)(HIGH_FRACTION * maxVal);
  }
}

void printThresholds() {
  Serial.print("thresholds: low = ");
  Serial.print(threshold_low);
  Serial.print(" | high = ");
  Serial.println(threshold_high);
}

void runCalibration() {
  int savedLow  = threshold_low;
  int savedHigh = threshold_high;

  Serial.println();
  Serial.println("=========== CALIBRATION ===========");
  Serial.println("Type x at any prompt to abort.");
  Serial.println();

  // ---- Stage 1: rest ----
  Serial.println("Stage 1 of 2: RESTING activity.");
  Serial.println("Relax the muscle completely and keep the arm still.");
  Serial.println("Type r + Enter to start a 5 second capture.");

  String cmd = waitForLine();
  if (cmd == "x") { Serial.println("Aborted. Thresholds unchanged."); return; }
  if (cmd != "r") { Serial.println("Unexpected input. Aborted."); return; }

  Serial.print("Measuring rest, hold still: ");
  Capture rest = captureWindow(CAL_CAPTURE_MS);
  reportCapture("REST", rest);
  Serial.println();

  // ---- Stage 2: max ----
  Serial.println("Stage 2 of 2: MAXIMUM tension.");
  Serial.println("Contract as hard as you comfortably can and HOLD it.");
  Serial.println("Type r + Enter to start a 5 second capture.");

  cmd = waitForLine();
  if (cmd == "x") { Serial.println("Aborted. Thresholds unchanged."); return; }
  if (cmd != "r") { Serial.println("Unexpected input. Aborted."); return; }

  Serial.print("Measuring max, hold the contraction: ");
  Capture maxc = captureWindow(CAL_CAPTURE_MS);
  reportCapture("MAX ", maxc);
  Serial.println();

  // ---- Validate ----
  int span = maxc.mean - rest.mean;
  if (span < MIN_VALID_SPAN) {
    Serial.println("!! CALIBRATION FAILED !!");
    Serial.print("Max is only ");
    Serial.print(span);
    Serial.println(" counts above rest - not distinguishable.");
    Serial.println("Check electrode contact, skin prep and cable seating.");
    threshold_low  = savedLow;
    threshold_high = savedHigh;
    printThresholds();
    return;
  }

  // ---- Apply ----
  emg_rest_avg = rest.mean;
  emg_max_avg  = maxc.mean;
  emg_max_peak = maxc.peak;
  computeThresholds(emg_rest_avg, emg_max_avg);

  Serial.println("--- READING COMPLETE ---");
  Serial.print("rest avg = ");   Serial.print(emg_rest_avg);
  Serial.print(" | max avg = "); Serial.print(emg_max_avg);
  Serial.print(" | max peak = ");Serial.println(emg_max_peak);
  printThresholds();

  // ---- Sanity warnings ----
  if (threshold_low <= emg_rest_avg) {
    Serial.println();
    Serial.println("!! WARNING: low threshold is at or below resting level.");
    Serial.println("   The hand will latch on and never release.");
    Serial.println("   Set USE_REST_REFERENCED to true and re-run.");
  }
  if (threshold_high <= threshold_low) {
    Serial.println("!! WARNING: no hysteresis gap - high <= low.");
  }
  Serial.println("===================================");
  Serial.println();
}

// ------------------------------------------------------------- framework

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(200);

  Serial.println();
  Serial.println("EMG threshold calibration");
  Serial.println("Commands: calibrate | t (thresholds) | m (mute stream)");
  printThresholds();
}

void loop() {
  String cmd;
  if (readLine(cmd)) {
    if (cmd == "calibrate") {
      runCalibration();
    } else if (cmd == "t") {
      printThresholds();
    } else if (cmd == "m") {
      streaming = !streaming;
      Serial.println(streaming ? "stream on" : "stream muted");
    } else {
      Serial.print("unknown command: ");
      Serial.println(cmd);
    }
  }

  int finger_signal = analogRead(SENSOR_PIN);

  unsigned long now = millis();
  if (streaming && (now - lastPrintMs >= PRINT_INTERVAL_MS)) {
    lastPrintMs = now;
    Serial.println(finger_signal);
  }
}
