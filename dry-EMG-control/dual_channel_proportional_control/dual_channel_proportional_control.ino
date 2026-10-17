/*
  dual_channel_proportional_control.ino
  RMS ENVELOPE VERSION, TWO CHANNELS (A0 = closing site, A1 = opening site)
  ------------------------------------------------------------------
  Tendon-driven hand, 4 fingers on L298N drivers, controlled from two
  DFRobot Gravity Analog EMG sensors (OYMotion SEN0240, metal dry
  electrodes) with a software envelope per channel, hysteresis, runtime
  calibration and proportional speed control.

  Board:   Arduino Nano ESP32 (ESP32-S3, 3.3 V logic, 12-bit ADC)
  Sensors: channel 0: SEN0240 -> A0 (GPIO1, ADC1) - closing muscles
           channel 1: SEN0240 -> A1 (GPIO2, ADC1) - opening muscles,
                      other side of the forearm
           Both powered from the Nano's 3V3 rail, grounds returned to
           the Nano - NOT via the L298N / motor ground path.
  Motors:  2x L298N, IN1/IN2/ENA per finger

  WHY THIS SKETCH LOOKS DIFFERENT FROM THE MYOWARE VERSION
  The MyoWare ENV pin carried an envelope made in hardware, so
  analogRead() WAS muscle activity. The SEN0240 outputs raw amplified
  sEMG (gain 1000) centred on ~1.5 V. Its mean is the same relaxed or
  contracted - contraction only widens the swing around the centre.
  So the envelope is now built in firmware, once per channel:

      raw -> DC removal -> preFilter (slot) -> x^2 -> moving mean -> sqrt -> env

  (no separate rectify step: squaring already makes everything positive)

  and env replaces the old raw pin everywhere: state machine, PWM map,
  calibration and stream. The filters assume Ts = 1 ms, so sampling
  runs on a fixed 1 kHz micros() schedule and nothing in the control
  path is allowed to block.

  TWO-CHANNEL CONTROL
  Each channel has its own copy of the chain, its own thresholds and
  its own PWM map. Channel 0 closes, channel 1 opens ('s' swaps them).
  Only one channel drives at a time. If both are over their trigger
  thresholds together (co-contraction), the hand stops and EMG is
  locked out for CONFLICT_LOCKOUT_MS. The lockout is timed, NOT a
  delay(): sampling keeps running so the envelopes stay current and
  serial commands still work. After the timeout EMG stays locked out
  until BOTH channels have relaxed below their release thresholds, so
  a contraction still being held cannot fire the instant it ends.

  SAFETY GATING
  EMG control is DISARMED at power-up and stays disarmed until a
  successful 'calibrate' in this session: no threshold numbers survive
  the sensor change. Manual finger commands work while disarmed.

  Serial Monitor MUST be set to "New Line" (or "Both NL & CR").
  All commands are read as whole lines.

  Commands:
    1 3 5 7    open finger 1..4 at testSpeed
    2 4 6 8    close finger 1..4 at testSpeed
    0          stop all
    a          open all at testSpeed
    s          swap roles (A0 closes / A1 opens <-> A0 opens / A1 closes)
    t          print status, thresholds and PWM map endpoints
    m          mute / unmute the live stream (centred0, env0, centred1, env1, pwm)
    calibrate  run the four-stage threshold calibration (A0 then A1)

  TODO (out of scope, do not implement yet):
    - No stall detection at full closure. If the user keeps contracting
      after the fingers bottom out, the L298Ns sit at stall current.
*/

#include <math.h>

// ================================================================ config

const int NUM_CH = 2;
int testSpeed = 150;              // fixed speed used by manual commands

// ---- proportional drive ----
// Measured: below ~100 the motor will not reliably start from rest
// against rubber band tension. 220 is the tested upper limit.
// NOTE: both were measured at the core's default analogWrite PWM
// frequency. If the PWM frequency is ever changed (e.g. to move motor
// noise out of the EMG band) these must be re-measured.
const int PWM_MIN = 100;
const int PWM_MAX = 220;

// ---- sampling ----
// Every filter coefficient below is derived from TS_S, so this rate
// must be real, not nominal.
const unsigned long SAMPLE_US = 1000;              // 1 kHz
const float TS_S = SAMPLE_US * 1.0e-6f;

// If the loop ever falls this far behind (a long blocking print, say),
// we resync instead of catching up. Catching up would fire a burst of
// back-to-back ADC reads that all happen "now" - the filters would
// treat 20+ ms of time as having passed in a few hundred us. Skipping
// loses those samples but keeps the time base honest from then on.
// Every resync is counted; check 'overruns' in 't' after testing.
const unsigned long MAX_LAG_US = 20000;            // 20 samples

// ---- DC removal ----
// Slow EMA tracks the sensor's ~1.5 V reference in ADC counts. It is a
// first-order high-pass at fc = 1/(2*pi*tau) = 0.16 Hz: far below the
// EMG band, so it removes offset without touching the signal. The
// centre is measured, never hardcoded - the ESP32 ADC read ~1800 on
// our board versus a theoretical 1861.
const float DC_TAU_S = 1.0f;
const int   SEED_SAMPLES = 200;       // offset seeded with their mean
const unsigned long SETTLE_MS = 1000; // EMG stays disabled this long after seeding

// ---- envelope (moving-window RMS) ----
// Same window on both channels. RMS_WINDOW_MS is the tuning knob:
// shorter = snappier trigger/release but more ripple (chatter near the
// thresholds and jittery motor speed), longer = smoother but laggier.
// For a step in contraction the RMS goes 10-90% in ~0.8 * window
// (~120 ms at 150 ms). Unlike the EMA there is no tail: a contraction
// is completely forgotten one window after it ends.
const unsigned long RMS_WINDOW_MS = 150;
const int RMS_WINDOW = RMS_WINDOW_MS * 1000UL / SAMPLE_US;   // 150 samples
// Scales the envelope into a convenient range. Leave at 1.0 until you
// have measured rest/max, then tune if the numbers are awkward.
const float ENV_GAIN  = 1.0f;

// Alpha from physical constants: alpha = 1 - exp(-Ts / tau).
// (Dynamic initialisation of globals is fine in C++; TS_S is defined above.)
const float ALPHA_DC  = 1.0f - expf(-TS_S / DC_TAU_S);

// ---- decimation ----
const unsigned long SPEED_DECIM = 5;    // motor speed update at 200 Hz
const unsigned long PRINT_DECIM = 10;   // stream at 100 Hz

// ---- calibration settings, one set per channel ----
// Same values on both channels for now, but kept separate so each site
// can be tuned on its own: the two muscle groups will not give the same
// amplitude or the same rest ripple.
//
// MIN_VALID_SPAN: max envelope must exceed rest envelope by this much.
// 35 was measured on the EMA envelope and is only a starting guess for
// RMS: RE-MEASURE (log a few rest/max captures per channel) and set it
// to a fraction of a typical measured span.

// Channel 0 - A0, closing site
const float LOW_FRACTION_0   = 0.3f;
const float HIGH_FRACTION_0  = 0.5f;
const float MIN_VALID_SPAN_0 = 35.0f;

// Channel 1 - A1, opening site
const float LOW_FRACTION_1   = 0.3f;
const float HIGH_FRACTION_1  = 0.5f;
const float MIN_VALID_SPAN_1 = 35.0f;

const unsigned long CAL_CAPTURE_MS = 5000;

// Raw ADC limits for the clipping check. The SEN0240 swings 0 - 3.0 V
// and the ESP32-S3 ADC is nonlinear near both rails, so treat anything
// outside this window as clipped or near-clipped.
const int CLIP_LOW  = 100;
const int CLIP_HIGH = 3700;

// ---- co-contraction lockout ----
const unsigned long CONFLICT_LOCKOUT_MS = 2000;

// Live stream
bool streaming = false;

// EMG state machine
bool fingersActive = false;   // true = EMG has triggered and not yet released
int  activeCh      = 0;       // which channel is driving (valid while fingersActive)
bool emgCloseMode  = true;    // true = A0 closes / A1 opens; false = swapped
bool emgArmed      = false;   // set only by a successful calibrate this session
bool emgEnabled = true;                    // 'e' toggles; failsafe clears it
unsigned long activeSinceMs = 0;           // when the current EMG drive started
const unsigned long MAX_ACTIVE_MS = 2200;  // failsafe: max continuous drive time
int  currentPwm    = 0;       // last duty written to the enable pins

bool conflictLockout = false;              // co-contraction lockout in progress
unsigned long lockoutStartMs = 0;

// ---- shared signal chain state ----
bool  offsetSeeded = false;
int   seedCount    = 0;
unsigned long seedDoneMs = 0;

unsigned long nextSampleUs = 0;
unsigned long sampleCount  = 0;
unsigned long overrunCount = 0;

// ---- types must be declared before ANY function definition, because the
// ---- Arduino preprocessor injects auto-generated prototypes above the
// ---- first function in the file.

struct Finger {
  int in1, in2, ena;
};

// Everything one EMG channel owns: its pin, its calibration settings,
// thresholds and results, and its signal chain state. All float in the
// filters: no int truncation.
//
// RMS window: ring buffer of squared samples plus a running sum, so each
// sample costs one add, one subtract and one sqrt whatever the window.
// Squares are rounded to whole counts^2 and summed as integers so the
// running sum can never drift (a float sum accumulates rounding error
// from the add/subtract pairs). Even a 4096-count swing gives
// 150 * 4096^2 ~ 2.5e9, inside uint32_t.
struct EmgChannel {
  int         pin;
  const char *name;
  // calibration settings
  float lowFraction;
  float highFraction;
  float minValidSpan;
  // thresholds + calibration results (envelope units)
  float thrLow;
  float thrHigh;
  float restAvg;
  float maxAvg;
  float maxPeak;
  // signal chain state
  float    dcOffset;
  float    centred;           // last centred (and pre-filtered) sample
  float    env;               // the envelope
  int      lastRaw;           // last raw ADC read, for clipping checks
  long     seedSum;
  uint32_t rmsBuf[RMS_WINDOW];
  uint32_t rmsSum;
  int      rmsIdx;
};

// Stats of one channel's envelope over a capture window, plus the raw
// ADC range seen, which is the only way to spot clipping (the envelope
// hides it).
struct Capture {
  float mean;
  float peak;
  float floorVal;
  int   rawMin;
  int   rawMax;
  long  samples;
};

// Members not listed start at zero (thresholds 0 = placeholders only;
// EMG control is disarmed until calibrate fills them).
EmgChannel ch[NUM_CH] = {
  { A0, "A0", LOW_FRACTION_0, HIGH_FRACTION_0, MIN_VALID_SPAN_0 },
  { A1, "A1", LOW_FRACTION_1, HIGH_FRACTION_1, MIN_VALID_SPAN_1 }
};

Finger fingers[4] = {
  {2, 3, 10},   // Finger 1
  {4, 5, 11},   // Finger 2
  {6, 7, 12},   // Finger 3
  {8, 9, 13}    // Finger 4
};

// ======================================================= motor primitives

void openFinger(int i, int speed) {
  digitalWrite(fingers[i].in1, HIGH);
  digitalWrite(fingers[i].in2, LOW);
  analogWrite(fingers[i].ena, speed);
}

void closeFinger(int i, int speed) {
  digitalWrite(fingers[i].in1, LOW);
  digitalWrite(fingers[i].in2, HIGH);
  analogWrite(fingers[i].ena, speed);
}

void stopFinger(int i) {
  digitalWrite(fingers[i].in1, LOW);
  digitalWrite(fingers[i].in2, LOW);
  analogWrite(fingers[i].ena, 0);
}

void stopAll() {
  for (int i = 0; i < 4; i++) stopFinger(i);
  currentPwm = 0;
  Serial.println("All fingers stopped");
}

void closeAll(int speed) {
  for (int i = 0; i < 4; i++) closeFinger(i, speed);
  currentPwm = speed;
  Serial.print("All fingers closing @ ");
  Serial.println(speed);
}

void openAll(int speed) {
  for (int i = 0; i < 4; i++) openFinger(i, speed);
  currentPwm = speed;
  Serial.print("All fingers opening @ ");
  Serial.println(speed);
}

// Speed-only update. Direction pins are already latched, so during a
// grasp we touch nothing but the enable pins. No printing - this runs
// at the decimated control rate and would flood the monitor.
void applySpeedAll(int speed) {
  for (int i = 0; i < 4; i++) analogWrite(fingers[i].ena, speed);
  currentPwm = speed;
}

// ==================================================== channel roles

// Does this channel close the hand (true) or open it (false)?
bool channelCloses(int i) {
  return (i == 0) == emgCloseMode;
}

const char *roleOf(int i) {
  return channelCloses(i) ? "close" : "open";
}

// Start an EMG-driven movement for channel i.
void startDrive(int i, int pwm) {
  if (channelCloses(i)) closeFinger(0, pwm); else openFinger(0, pwm);
}

// ==================================================== proportional map

/*
  Maps a channel's EMG envelope onto motor duty, using that channel's
  own calibration.

  Input range runs from the channel's release threshold (not rest) up to
  its max average. Starting at the release threshold means speed decays
  smoothly to PWM_MIN as you relax rather than falling off a cliff at
  the moment of release. Topping out at the mean of a sustained max
  contraction rather than its peak means full speed is reachable
  without straining.
*/
int emgToPwm(const EmgChannel &c) {
  float lo = c.thrLow;
  float hi = c.maxAvg;
  float signal = c.env;

  // Degenerate calibration - fall back to the known-good fixed speed
  // rather than something arbitrary at either extreme.
  if (hi <= lo) return testSpeed;

  if (signal <= lo) return PWM_MIN;
  if (signal >= hi) return PWM_MAX;

  float frac = (signal - lo) / (hi - lo);
  return PWM_MIN + (int)(frac * (PWM_MAX - PWM_MIN));
}

// ========================================================== signal chain

/*
  Pre-filter slot. Pass-through for now, on purpose: the SEN0240 does
  its own analog filtering and we only add digital filtering if the
  validation tests show we need it.
    - Rest level rises next to a mains charger  -> 50 Hz notch here.
    - Envelope spikes on arm/cable movement     -> ~20 Hz high-pass here.
  TWO CHANNELS: this function is shared, which is only safe while it is
  stateless. Any real filter here keeps state (past samples), so it
  needs one copy of that state PER CHANNEL - move it into EmgChannel.
  That also rules out the OYMotion EMGFilters library as-is: it has a
  known multi-instance bug, and two channels need two instances.
*/
float preFilter(float x) {
  return x;
}

// Conditioning chain for one channel, on a sample already read.
void processChannel(EmgChannel &c, int raw) {
  // ---- 1. DC removal ----
  // (float resolution note: at ~1800 counts the float step is ~1e-4, so
  //  the tracker's residual error is ~0.1 count. Irrelevant here.)
  c.dcOffset += ((float)raw - c.dcOffset) * ALPHA_DC;
  float centred = (float)raw - c.dcOffset;

  // ---- 2. optional HP / notch ----
  centred = preFilter(centred);
  c.centred = centred;

  // ---- 3. square + 4. moving mean + 5. sqrt (+ output scaling) ----
  // Window fills during SETTLE_MS (1000 ms > 150 ms), so the envelope is
  // valid before signalReady() lets anything use it.
  uint32_t sq = (uint32_t)(centred * centred + 0.5f);
  c.rmsSum -= c.rmsBuf[c.rmsIdx];
  c.rmsBuf[c.rmsIdx] = sq;
  c.rmsSum += sq;
  if (++c.rmsIdx >= RMS_WINDOW) c.rmsIdx = 0;

  c.env = sqrtf((float)c.rmsSum / RMS_WINDOW) * ENV_GAIN;
}

/*
  One sample of the whole acquisition + conditioning chain for both
  channels, on a fixed 1 kHz schedule. Returns true if a sample was
  processed this call, false if it was not yet time.

  Called from loop(), the calibration capture and the calibration
  prompt wait, so the filters never go stale - which is why there is
  no longer any "reseed after calibrate" step.

  It processes at most one sample per call. The callers spin fast, so
  small lateness is caught up over the next few calls and the long-run
  rate stays exactly 1 kHz. The two ADC reads are back to back, a few
  tens of us apart - negligible at 1 kHz.
*/
bool sampleTick() {
  unsigned long now = micros();

  // Signed difference: correct across the micros() wrap (~71 min).
  if ((long)(now - nextSampleUs) < 0) return false;

  if ((unsigned long)(now - nextSampleUs) > MAX_LAG_US) {
    nextSampleUs = now + SAMPLE_US;   // resync, see MAX_LAG_US
    overrunCount++;
  } else {
    nextSampleUs += SAMPLE_US;        // from the intended time, never from now
  }

  int raw[NUM_CH];
  for (int i = 0; i < NUM_CH; i++) {
    raw[i] = analogRead(ch[i].pin);
    ch[i].lastRaw = raw[i];
  }
  sampleCount++;

  // ---- startup: seed each offset with a real mean, never 0 ----
  // Starting the tracker at 0 would make centred = ~1800 for the first
  // few seconds, i.e. a "maximum contraction" at power-up.
  if (!offsetSeeded) {
    for (int i = 0; i < NUM_CH; i++) ch[i].seedSum += raw[i];
    seedCount++;
    if (seedCount >= SEED_SAMPLES) {
      for (int i = 0; i < NUM_CH; i++) {
        ch[i].dcOffset = (float)ch[i].seedSum / (float)seedCount;
        ch[i].env      = 0.0f;
      }
      offsetSeeded = true;
      seedDoneMs   = millis();
    }
    return true;
  }

  for (int i = 0; i < NUM_CH; i++) processChannel(ch[i], raw[i]);

  return true;
}

// Offsets seeded AND given SETTLE_MS for the envelopes to settle.
bool signalReady() {
  return offsetSeeded && (millis() - seedDoneMs >= SETTLE_MS);
}

// ============================================================= serial i/o

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

// Blocking wrapper, used only inside calibration. Keeps the signal chain
// running while the user reads the prompt.
String waitForLine() {
  String line;
  while (!readLine(line)) {
    sampleTick();
  }
  return line;
}

// =============================================================== capture

// Statistics of BOTH channels' envelopes (not raw ADC) over a window,
// sampled at the real 1 kHz rate. Raw min/max is tracked alongside for
// the clip check. Capturing both at once is what lets calibration spot
// crosstalk: e.g. how far A1 rises while you close.
void captureWindow(unsigned long durationMs, Capture out[NUM_CH]) {
  unsigned long start = millis();
  double sum[NUM_CH];
  float  peak[NUM_CH], floorVal[NUM_CH];
  int    rawMin[NUM_CH], rawMax[NUM_CH];
  for (int i = 0; i < NUM_CH; i++) {
    sum[i] = 0.0; peak[i] = -1.0e9f; floorVal[i] = 1.0e9f;
    rawMin[i] = 4095; rawMax[i] = 0;
  }
  long n = 0;
  int  lastSecond = -1;

  while (millis() - start < durationMs) {
    if (!sampleTick()) continue;

    for (int i = 0; i < NUM_CH; i++) {
      float v = ch[i].env;
      sum[i] += v;
      if (v > peak[i])              peak[i] = v;
      if (v < floorVal[i])          floorVal[i] = v;
      if (ch[i].lastRaw < rawMin[i]) rawMin[i] = ch[i].lastRaw;
      if (ch[i].lastRaw > rawMax[i]) rawMax[i] = ch[i].lastRaw;
    }
    n++;

    int secondsElapsed = (millis() - start) / 1000;
    if (secondsElapsed != lastSecond) {
      lastSecond = secondsElapsed;
      Serial.print(durationMs / 1000 - secondsElapsed);
      Serial.print("... ");
    }
  }
  Serial.println();

  for (int i = 0; i < NUM_CH; i++) {
    out[i].mean     = (n > 0) ? (float)(sum[i] / n) : 0.0f;
    out[i].peak     = (n > 0) ? peak[i] : 0.0f;
    out[i].floorVal = (n > 0) ? floorVal[i] : 0.0f;
    out[i].rawMin   = rawMin[i];
    out[i].rawMax   = rawMax[i];
    out[i].samples  = n;
  }
}

void reportCapture(const char *label, const char *chName, Capture c) {
  Serial.print(label);
  Serial.print(" ");              Serial.print(chName);
  Serial.print(" -> env mean ");  Serial.print(c.mean, 1);
  Serial.print(" | min ");        Serial.print(c.floorVal, 1);
  Serial.print(" | peak ");       Serial.print(c.peak, 1);
  Serial.print(" | raw ");        Serial.print(c.rawMin);
  Serial.print("..");             Serial.print(c.rawMax);
  Serial.print(" | n=");          Serial.println(c.samples);
}

// =========================================================== calibration

/*
  Thresholds are referenced to the span above rest, per channel:
      low  = rest + lowFraction  * (max - rest)
      high = rest + highFraction * (max - rest)
  The rest envelope is not zero - it is the rectified noise floor of the
  sensor + ADC + skin interface - so it is baseline, not signal, and has
  to be subtracted before any fraction is applied.
*/
void computeThresholds(EmgChannel &c, float rest, float maxVal) {
  float span = maxVal - rest;
  c.thrLow  = rest + c.lowFraction  * span;
  c.thrHigh = rest + c.highFraction * span;
}

void printThresholds() {
  Serial.print("EMG control: ");
  if (!signalReady())       Serial.println("NOT READY (signal chain settling)");
  else if (!emgArmed)       Serial.println("DISARMED - run calibrate");
  else if (conflictLockout) Serial.println("ARMED, co-contraction LOCKOUT active");
  else                      Serial.println("ARMED");

  Serial.print("overruns = ");      Serial.println(overrunCount);

  for (int i = 0; i < NUM_CH; i++) {
    const EmgChannel &c = ch[i];
    Serial.print("--- ");  Serial.print(c.name);
    Serial.print(" (");    Serial.print(roleOf(i));   Serial.println(") ---");
    Serial.print("dc offset = ");     Serial.print(c.dcOffset, 1);
    Serial.print(" | env = ");        Serial.println(c.env, 1);
    Serial.print("thresholds: low = ");
    Serial.print(c.thrLow, 1);
    Serial.print(" | high = ");
    Serial.println(c.thrHigh, 1);
    Serial.print("pwm map: ");
    Serial.print(c.thrLow, 1);  Serial.print(" -> "); Serial.print(PWM_MIN);
    Serial.print("  ...  ");
    Serial.print(c.maxAvg, 1);  Serial.print(" -> "); Serial.println(PWM_MAX);
    Serial.print("env units per pwm step: ");
    Serial.println((c.maxAvg - c.thrLow) / (float)(PWM_MAX - PWM_MIN), 3);
  }
}

// One prompt + capture stage. Returns false if the user aborted.
bool runStage(const char *title, const char *instruction, const char *label,
              Capture out[NUM_CH]) {
  Serial.println(title);
  Serial.println(instruction);
  Serial.println("Type r + Enter to start a 5 second capture.");

  String cmd = waitForLine();
  if (cmd == "x") { Serial.println("Aborted. Thresholds unchanged."); return false; }
  if (cmd != "r") { Serial.println("Unexpected input. Aborted. Thresholds unchanged."); return false; }

  Serial.print("Capturing, hold it: ");
  captureWindow(CAL_CAPTURE_MS, out);
  for (int i = 0; i < NUM_CH; i++) reportCapture(label, ch[i].name, out[i]);
  Serial.println();
  return true;
}

// Is the span between rest and max big enough on channel i?
bool spanValid(int i, const Capture &rest, const Capture &maxc) {
  float span = maxc.mean - rest.mean;
  if (span >= ch[i].minValidSpan) return true;

  Serial.print("!! CALIBRATION FAILED on ");
  Serial.print(ch[i].name);
  Serial.println(" !!");
  Serial.print("Max is only ");
  Serial.print(span, 1);
  Serial.println(" env units above rest - not distinguishable.");
  Serial.println("Check electrode contact, skin prep and cable seating.");
  Serial.println("(MIN_VALID_SPAN is still a guess for this sensor - see its comment.)");
  Serial.println("Thresholds unchanged.");
  // emgArmed is left as it was: a previous good calibration stays live.
  return false;
}

void warnChannel(int i, const Capture &rest, const Capture &maxc) {
  const EmgChannel &c = ch[i];
  if (c.thrLow <= c.restAvg) {
    Serial.print("!! "); Serial.print(c.name);
    Serial.println(": low threshold is at or below resting level.");
    Serial.println("   The hand will latch on and never release.");
  }
  if (c.thrHigh <= c.thrLow) {
    Serial.print("!! "); Serial.print(c.name);
    Serial.println(": no hysteresis gap - high <= low.");
  }
  if (c.maxAvg <= c.thrHigh) {
    Serial.print("!! "); Serial.print(c.name);
    Serial.println(": pwm map tops out below the trigger point.");
    Serial.println("   Every movement on this channel will run at PWM_MAX.");
  }
  // Software envelope ripple at rest is much larger than the MyoWare's
  // hardware envelope, so check the rest PEAK, not just the mean.
  if (rest.peak >= c.thrHigh) {
    Serial.print("!! "); Serial.print(c.name);
    Serial.println(": rest envelope peaks reach its high threshold.");
    Serial.println("   Expect false triggers at rest. Raise this channel's");
    Serial.println("   HIGH_FRACTION or RMS_WINDOW_MS, or improve electrode contact.");
  }
  if (maxc.rawMin < CLIP_LOW || maxc.rawMax > CLIP_HIGH) {
    Serial.print("!! "); Serial.print(c.name);
    Serial.println(": raw signal clipped during its MAX capture.");
    Serial.print("   raw range ");
    Serial.print(maxc.rawMin); Serial.print(".."); Serial.println(maxc.rawMax);
    Serial.println("   Recalibrate with a slightly less intense contraction.");
  }
}

void runCalibration() {
  // ---- SAFETY INTERLOCK ----
  // Calibration blocks for 20+ seconds. Anything still driving would run
  // unattended for that whole time, so kill the motors and clear the EMG
  // latch before we go anywhere near a blocking capture.
  stopAll();
  fingersActive = false;

  if (!signalReady()) {
    Serial.println("Signal chain still settling - try again in a second.");
    return;
  }

  Serial.println();
  Serial.println("=========== CALIBRATION ===========");
  Serial.println("Motors stopped. Type x at any prompt to abort.");
  Serial.println();

  Capture rest0[NUM_CH], max0[NUM_CH], rest1[NUM_CH], max1[NUM_CH];

  // ---- A0: closing site ----
  if (!runStage("Stage 1 of 4: RESTING activity (A0, closing site).",
                "Relax the hand completely and keep the arm still.",
                "REST ", rest0)) return;

  if (!runStage("Stage 2 of 4: MAXIMUM CLOSING tension (A0).",
                "Close the hand as hard as you comfortably can and HOLD it.",
                "CLOSE", max0)) return;

  // Fail early: no point making the user do A1 if A0 is unusable.
  if (!spanValid(0, rest0[0], max0[0])) { printThresholds(); return; }

  // ---- A1: opening site ----
  if (!runStage("Stage 3 of 4: RESTING activity again (A1, opening site).",
                "Relax the hand completely again and keep the arm still.",
                "REST ", rest1)) return;

  if (!runStage("Stage 4 of 4: MAXIMUM OPENING tension (A1).",
                "Open the hand as hard as you comfortably can and HOLD it.",
                "OPEN ", max1)) return;

  if (!spanValid(1, rest1[1], max1[1])) { printThresholds(); return; }

  // ---- Apply (both channels, only once both are valid) ----
  ch[0].restAvg = rest0[0].mean;
  ch[0].maxAvg  = max0[0].mean;
  ch[0].maxPeak = max0[0].peak;
  computeThresholds(ch[0], ch[0].restAvg, ch[0].maxAvg);

  ch[1].restAvg = rest1[1].mean;
  ch[1].maxAvg  = max1[1].mean;
  ch[1].maxPeak = max1[1].peak;
  computeThresholds(ch[1], ch[1].restAvg, ch[1].maxAvg);

  emgArmed = true;
  conflictLockout = false;

  Serial.println("--- READING COMPLETE ---");
  for (int i = 0; i < NUM_CH; i++) {
    Serial.print(ch[i].name);
    Serial.print(": rest avg = ");  Serial.print(ch[i].restAvg, 1);
    Serial.print(" | max avg = ");  Serial.print(ch[i].maxAvg, 1);
    Serial.print(" | max peak = "); Serial.println(ch[i].maxPeak, 1);
  }
  printThresholds();

  // ---- Sanity warnings ----
  Serial.println();
  warnChannel(0, rest0[0], max0[0]);
  warnChannel(1, rest1[1], max1[1]);

  // ---- Crosstalk / co-contraction ----
  // If one movement drives the OTHER channel over its trigger, that
  // movement will hit the co-contraction lockout instead of working.
  if (max0[1].peak >= ch[1].thrHigh) {
    Serial.println("!! CROSSTALK: A1 reached its trigger while you CLOSED.");
    Serial.println("   Hard grasps will hit the co-contraction lockout.");
    Serial.println("   Move the A1 electrodes, or raise HIGH_FRACTION_1.");
  }
  if (max1[0].peak >= ch[0].thrHigh) {
    Serial.println("!! CROSSTALK: A0 reached its trigger while you OPENED.");
    Serial.println("   Hard opens will hit the co-contraction lockout.");
    Serial.println("   Move the A0 electrodes, or raise HIGH_FRACTION_0.");
  }

  Serial.println("EMG control ARMED.");
  Serial.println("===================================");
  Serial.println();
}

// ================================================== co-contraction lockout

void startConflictLockout() {
  stopAll();
  fingersActive   = false;
  conflictLockout = true;
  lockoutStartMs  = millis();
  Serial.print("!! CO-CONTRACTION: both channels triggered. EMG locked out for ");
  Serial.print(CONFLICT_LOCKOUT_MS);
  Serial.println(" ms, then until both muscles relax.");
}

// ========================================================= command parser

void handleCommand(const String &cmd) {
  // Manual finger commands clear the EMG latch so the two control paths
  // cannot end up disagreeing about what the hand is doing.
  if      (cmd == "1") { openFinger(0,  testSpeed); fingersActive = false; }
  else if (cmd == "2") { closeFinger(0, testSpeed); fingersActive = false; }
  else if (cmd == "3") { openFinger(1,  testSpeed); fingersActive = false; }
  else if (cmd == "4") { closeFinger(1, testSpeed); fingersActive = false; }
  else if (cmd == "5") { openFinger(2,  testSpeed); fingersActive = false; }
  else if (cmd == "6") { closeFinger(2, testSpeed); fingersActive = false; }
  else if (cmd == "7") { openFinger(3,  testSpeed); fingersActive = false; }
  else if (cmd == "8") { closeFinger(3, testSpeed); fingersActive = false; }
  else if (cmd == "0") { stopAll();                 fingersActive = false; }
  else if (cmd == "a") { openAll(testSpeed);        fingersActive = false; }
  else if (cmd == "c") { closeAll(testSpeed);       fingersActive = false; }
  else if (cmd == "s") {
    emgCloseMode = !emgCloseMode;
    Serial.print("EMG roles: A0 ");
    Serial.print(roleOf(0));
    Serial.print("s / A1 ");
    Serial.print(roleOf(1));
    Serial.println("s");
  }
  else if (cmd == "t") { printThresholds(); }
  else if (cmd == "m") {
    streaming = !streaming;
    Serial.println(streaming ? "stream on: centred0,env0,centred1,env1,pwm" : "stream muted");
  }
  else if (cmd == "calibrate") { runCalibration(); }
  else if (cmd == "e") {
    emgEnabled = !emgEnabled;
    if (!emgEnabled) {
      stopAll();
      fingersActive = false;
    }
    Serial.println(emgEnabled ? "EMG control ON" : "EMG control OFF");
  }
  else {
    Serial.print("unknown command: ");
    Serial.println(cmd);
  }
}

// =============================================================== framework

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(200);   // setup only; nothing in the control path may delay()

  analogReadResolution(12);

  for (int i = 0; i < 4; i++) {
    pinMode(fingers[i].in1, OUTPUT);
    pinMode(fingers[i].in2, OUTPUT);
    pinMode(fingers[i].ena, OUTPUT);
    stopFinger(i);
  }

  // Start the sample clock last, so setup time is not counted as lag.
  // Offset seeding then happens inside sampleTick() over the first
  // SEED_SAMPLES samples (200 ms) without blocking.
  nextSampleUs = micros();

  Serial.println();
  Serial.println("EMG hand control ready (2x SEN0240, RMS envelope, A0 close / A1 open).");
  Serial.println("1-8 fingers | 0 stop | a open all | s swap roles");
  Serial.println("t status | m stream | calibrate");
  Serial.println("EMG control is DISARMED until you run calibrate.");
}

void loop() {
  String cmd;
  if (readLine(cmd)) {
    handleCommand(cmd);
  }

  // Everything below runs once per 1 kHz sample and only then.
  if (!sampleTick()) return;
  if (!signalReady()) return;   // startup gate: seeded + settled

  // ---- two-channel EMG state machine with hysteresis ----
  // Trigger on a channel's high threshold, release on its low one.
  // Only one channel drives at a time; both over their trigger at once
  // is co-contraction -> stop + lockout. Disarmed = no EMG motion at all.
  if (emgArmed && emgEnabled) {
    if (conflictLockout) {
      // Stays locked until the time is up AND both muscles are relaxed.
      bool timeUp  = millis() - lockoutStartMs >= CONFLICT_LOCKOUT_MS;
      bool relaxed = ch[0].env < ch[0].thrLow && ch[1].env < ch[1].thrLow;
      if (timeUp && relaxed) {
        conflictLockout = false;
        Serial.println("Co-contraction lockout cleared - EMG live.");
      }
    } else if (!fingersActive) {
      bool trig0 = ch[0].env > ch[0].thrHigh;
      bool trig1 = ch[1].env > ch[1].thrHigh;
      if (trig0 && trig1) {
        startConflictLockout();
      } else if (trig0 || trig1) {
        activeCh = trig0 ? 0 : 1;
        startDrive(activeCh, emgToPwm(ch[activeCh]));
        fingersActive = true;
        activeSinceMs = millis();
      }
    } else {
      int other = 1 - activeCh;
      if (ch[other].env > ch[other].thrHigh) {
        startConflictLockout();         // the other side joined in
      } else if (ch[activeCh].env < ch[activeCh].thrLow) {
        stopAll();
        fingersActive = false;
      }
    }
  }

  // ---- failsafe: continuous EMG drive for too long -> stop and disable ----
  if (fingersActive && millis() - activeSinceMs >= MAX_ACTIVE_MS) {
    stopAll();
    fingersActive = false;
    emgEnabled = false;
    Serial.println("!! FAILSAFE: EMG drive exceeded 3 s. EMG OFF - send e to re-enable.");
  }

  // ---- live speed modulation, decimated to 200 Hz ----
  // Direction pins are already latched, so this only rewrites the enable
  // pins, and only when the duty actually changes. Speed follows the
  // channel that is driving, through that channel's own map.
  if (fingersActive && (sampleCount % SPEED_DECIM == 0)) {
    int pwm = emgToPwm(ch[activeCh]);
    if (pwm != currentPwm) applySpeedAll(pwm);
  }

  // ---- live stream, decimated to 100 Hz ----
  // label:value pairs are understood by the IDE 2 Serial Plotter.
  // 'centred' is a snapshot every 10th sample, so it is ALIASED - fine
  // for eyeballing amplitude, useless for judging waveform shape.
  if (streaming && (sampleCount % PRINT_DECIM == 0)) {
    Serial.printf("centred0:%.1f,env0:%.1f,centred1:%.1f,env1:%.1f,pwm:%d\n",
                  ch[0].centred, ch[0].env, ch[1].centred, ch[1].env, currentPwm);
  }
}
