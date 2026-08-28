/*
  EMG_hand_control.ino
  ------------------------------------------------------------------
  Tendon-driven hand, 4 fingers on L298N drivers, controlled from a
  MyoWare 2.0 envelope output with hysteresis and runtime calibration.

  Board:  Arduino Nano ESP32
  Sensor: MyoWare 2.0 ENV output -> A0 (GPIO1, ADC1)
  Motors: 2x L298N, IN1/IN2/ENA per finger

  Serial Monitor MUST be set to "New Line" (or "Both NL & CR").
  All commands are read as whole lines.

  Commands:
    1 3 5 7    open finger 1..4
    2 4 6 8    close finger 1..4
    0          stop all
    a          open all
    s          toggle EMG direction (close-on-trigger / open-on-trigger)
    t          print current thresholds
    m          mute / unmute the live value stream
    calibrate  run the two-stage threshold calibration
*/

// ================================================================ config

const int SENSOR_PIN = A0;
int testSpeed = 150;

// Live thresholds. Defaults are the hand-tuned values that worked in
// testing; calibrate overwrites them.
int threshold_low = 140;
int threshold_high = 220;

// Calibration parameters
const unsigned long CAL_CAPTURE_MS = 5000; // length of each capture window
const unsigned long CAL_SAMPLE_MS = 2;     // ~500 Hz during capture
const float LOW_FRACTION = 0.05f;
const float HIGH_FRACTION = 0.10f;
const int MIN_VALID_SPAN = 100; // max must exceed rest by this

// Live stream throttle
const unsigned long PRINT_INTERVAL_MS = 50;
unsigned long lastPrintMs = 0;
bool streaming = false;

// EMG state machine
bool fingersActive = false; // true = EMG has triggered and not yet released
bool emgCloseMode = true;   // true = trigger closes; false = trigger opens

// Calibration results, kept for inspection
int emg_rest_avg = 0;
int emg_max_avg = 0;
int emg_max_peak = 0;

// ---- types must be declared before ANY function definition, because the
// ---- Arduino preprocessor injects auto-generated prototypes above the
// ---- first function in the file.

struct Finger
{
  int in1, in2, ena;
};

struct Capture
{
  int mean;
  int peak;
  int floorVal;
  long samples;
};

Finger fingers[4] = {
    {2, 3, 10}, // Finger 1
    {4, 5, 11}, // Finger 2
    {6, 7, 12}, // Finger 3
    {8, 9, 13}  // Finger 4
};

// ======================================================= motor primitives

void openFinger(int i)
{
  digitalWrite(fingers[i].in1, HIGH);
  digitalWrite(fingers[i].in2, LOW);
  analogWrite(fingers[i].ena, testSpeed);
}

void closeFinger(int i)
{
  digitalWrite(fingers[i].in1, LOW);
  digitalWrite(fingers[i].in2, HIGH);
  analogWrite(fingers[i].ena, testSpeed);
}

void stopFinger(int i)
{
  digitalWrite(fingers[i].in1, LOW);
  digitalWrite(fingers[i].in2, LOW);
  analogWrite(fingers[i].ena, 0);
}

void stopAll()
{
  for (int i = 0; i < 4; i++)
    stopFinger(i);
  Serial.println("All fingers stopped");
}

void closeAll()
{
  for (int i = 0; i < 4; i++)
    closeFinger(i);
  Serial.println("All fingers closing");
}

void openAll()
{
  for (int i = 0; i < 4; i++)
    openFinger(i);
  Serial.println("All fingers opening");
}

// ============================================================= serial i/o

String serialBuffer = "";

// Non-blocking. Returns true once a complete line has arrived, trimmed
// and lower-cased into 'out'.
bool readLine(String &out)
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (serialBuffer.length() > 0)
      {
        out = serialBuffer;
        out.trim();
        out.toLowerCase();
        serialBuffer = "";
        return true;
      }
      // bare terminator, ignore
    }
    else
    {
      serialBuffer += c;
    }
  }
  return false;
}

// Blocking wrapper, used only inside calibration.
String waitForLine()
{
  String line;
  while (!readLine(line))
  {
    delay(1);
  }
  return line;
}

// =============================================================== capture

Capture captureWindow(unsigned long durationMs)
{
  unsigned long start = millis();
  long sum = 0;
  long n = 0;
  int peak = 0;
  int floorVal = 32767;
  int lastSecond = -1;

  while (millis() - start < durationMs)
  {
    int v = analogRead(SENSOR_PIN);
    sum += v;
    n++;
    if (v > peak)
      peak = v;
    if (v < floorVal)
      floorVal = v;

    int secondsElapsed = (millis() - start) / 1000;
    if (secondsElapsed != lastSecond)
    {
      lastSecond = secondsElapsed;
      Serial.print(durationMs / 1000 - secondsElapsed);
      Serial.print("... ");
    }
    delay(CAL_SAMPLE_MS);
  }
  Serial.println();

  Capture c;
  c.mean = (n > 0) ? (int)(sum / n) : 0;
  c.peak = peak;
  c.floorVal = floorVal;
  c.samples = n;
  return c;
}

void reportCapture(const char *label, Capture c)
{
  Serial.print(label);
  Serial.print(" -> mean ");
  Serial.print(c.mean);
  Serial.print(" | min ");
  Serial.print(c.floorVal);
  Serial.print(" | peak ");
  Serial.print(c.peak);
  Serial.print(" | n=");
  Serial.println(c.samples);
}

// =========================================================== calibration

/*
  Thresholds are referenced to the span above rest:
      low  = rest + LOW_FRACTION  * (max - rest)
      high = rest + HIGH_FRACTION * (max - rest)
  The MyoWare envelope idles around 90 counts with the muscle slack, so
  that offset is baseline rather than signal and has to be subtracted
  before any fraction is applied.
*/
void computeThresholds(int rest, int maxVal)
{
  int span = maxVal - rest;
  threshold_low = rest + (int)(LOW_FRACTION * span);
  threshold_high = rest + (int)(HIGH_FRACTION * span);
}

void printThresholds()
{
  Serial.print("thresholds: low = ");
  Serial.print(threshold_low);
  Serial.print(" | high = ");
  Serial.println(threshold_high);
}

void runCalibration()
{
  int savedLow = threshold_low;
  int savedHigh = threshold_high;

  // ---- SAFETY INTERLOCK ----
  // Calibration blocks for 10+ seconds. Anything still driving would run
  // unattended for that whole time, so kill the motors and clear the EMG
  // latch before we go anywhere near a blocking capture.
  stopAll();
  fingersActive = false;

  Serial.println();
  Serial.println("=========== CALIBRATION ===========");
  Serial.println("Motors stopped. Type x at any prompt to abort.");
  Serial.println();

  // ---- Stage 1: rest ----
  Serial.println("Stage 1 of 2: RESTING activity.");
  Serial.println("Relax the muscle completely and keep the arm still.");
  Serial.println("Type r + Enter to start a 5 second capture.");

  String cmd = waitForLine();
  if (cmd == "x")
  {
    Serial.println("Aborted. Thresholds unchanged.");
    return;
  }
  if (cmd != "r")
  {
    Serial.println("Unexpected input. Aborted.");
    return;
  }

  Serial.print("Measuring rest, hold still: ");
  Capture rest = captureWindow(CAL_CAPTURE_MS);
  reportCapture("REST", rest);
  Serial.println();

  // ---- Stage 2: max ----
  Serial.println("Stage 2 of 2: MAXIMUM tension.");
  Serial.println("Contract as hard as you comfortably can and HOLD it.");
  Serial.println("Type r + Enter to start a 5 second capture.");

  cmd = waitForLine();
  if (cmd == "x")
  {
    Serial.println("Aborted. Thresholds unchanged.");
    return;
  }
  if (cmd != "r")
  {
    Serial.println("Unexpected input. Aborted.");
    return;
  }

  Serial.print("Measuring max, hold the contraction: ");
  Capture maxc = captureWindow(CAL_CAPTURE_MS);
  reportCapture("MAX ", maxc);
  Serial.println();

  // ---- Validate ----
  int span = maxc.mean - rest.mean;
  if (span < MIN_VALID_SPAN)
  {
    Serial.println("!! CALIBRATION FAILED !!");
    Serial.print("Max is only ");
    Serial.print(span);
    Serial.println(" counts above rest - not distinguishable.");
    Serial.println("Check electrode contact, skin prep and cable seating.");
    threshold_low = savedLow;
    threshold_high = savedHigh;
    printThresholds();
    return;
  }

  // ---- Apply ----
  emg_rest_avg = rest.mean;
  emg_max_avg = maxc.mean;
  emg_max_peak = maxc.peak;
  computeThresholds(emg_rest_avg, emg_max_avg);

  Serial.println("--- READING COMPLETE ---");
  Serial.print("rest avg = ");
  Serial.print(emg_rest_avg);
  Serial.print(" | max avg = ");
  Serial.print(emg_max_avg);
  Serial.print(" | max peak = ");
  Serial.println(emg_max_peak);
  printThresholds();

  // ---- Sanity warnings ----
  if (threshold_low <= emg_rest_avg)
  {
    Serial.println();
    Serial.println("!! WARNING: low threshold is at or below resting level.");
    Serial.println("   The hand will latch on and never release.");
  }
  if (threshold_high <= threshold_low)
  {
    Serial.println("!! WARNING: no hysteresis gap - high <= low.");
  }
  Serial.println("===================================");
  Serial.println();
  delay(2000);
}

// ========================================================= command parser

void handleCommand(const String &cmd)
{
  // Manual finger commands clear the EMG latch so the two control paths
  // cannot end up disagreeing about what the hand is doing.
  if (cmd == "1")
  {
    openFinger(0);
    fingersActive = false;
  }
  else if (cmd == "2")
  {
    closeFinger(0);
    fingersActive = false;
  }
  else if (cmd == "3")
  {
    openFinger(1);
    fingersActive = false;
  }
  else if (cmd == "4")
  {
    closeFinger(1);
    fingersActive = false;
  }
  else if (cmd == "5")
  {
    openFinger(2);
    fingersActive = false;
  }
  else if (cmd == "6")
  {
    closeFinger(2);
    fingersActive = false;
  }
  else if (cmd == "7")
  {
    openFinger(3);
    fingersActive = false;
  }
  else if (cmd == "8")
  {
    closeFinger(3);
    fingersActive = false;
  }
  else if (cmd == "0")
  {
    stopAll();
    fingersActive = false;
  }
  else if (cmd == "a")
  {
    openAll();
    fingersActive = false;
  }
  else if (cmd == "s")
  {
    emgCloseMode = !emgCloseMode;
    Serial.print("EMG mode: ");
    Serial.println(emgCloseMode ? "close on trigger" : "open on trigger");
  }
  else if (cmd == "t")
  {
    printThresholds();
  }
  else if (cmd == "m")
  {
    streaming = !streaming;
    Serial.println(streaming ? "stream on" : "stream muted");
  }
  else if (cmd == "calibrate")
  {
    runCalibration();
  }
  else
  {
    Serial.print("unknown command: ");
    Serial.println(cmd);
  }
}

// =============================================================== framework

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ;
  delay(200);

  for (int i = 0; i < 4; i++)
  {
    pinMode(fingers[i].in1, OUTPUT);
    pinMode(fingers[i].in2, OUTPUT);
    pinMode(fingers[i].ena, OUTPUT);
    stopFinger(i);
  }

  Serial.println();
  Serial.println("EMG hand control ready.");
  Serial.println("1-8 fingers | 0 stop | a open all | s mode");
  Serial.println("t thresholds | m stream | calibrate");
  printThresholds();
}

void loop()
{
  String cmd;
  if (readLine(cmd))
  {
    handleCommand(cmd);
  }

  int finger_signal = analogRead(SENSOR_PIN);

  unsigned long now = millis();
  if (streaming && (now - lastPrintMs >= PRINT_INTERVAL_MS))
  {
    lastPrintMs = now;
    Serial.println(finger_signal);
  }

  // ---- EMG state machine with hysteresis ----
  // Which threshold matters depends on which state we are in.
  if (!fingersActive)
  {
    if (finger_signal > threshold_high)
    {
      if (emgCloseMode)
        closeAll();
      else
        openAll();
      fingersActive = true;
    }
  }
  else
  {
    if (finger_signal < threshold_low)
    {
      stopAll();
      fingersActive = false;
    }
  }
}
