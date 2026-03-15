/*
 RobotRuka Arduino firmware
 - Controls multiple hobby servos attached to Arduino pins
 - Parses serial commands from the PC GUI in the form:
     #<id>P<position> S<speed> A<accel>\n
   Example:  #1P512S80A100\n
   Meaning:
     id      - servo index (1-based)
     P       - position (0..1023) - matches GUI raw positions
     S       - speed percent (1..100) (optional)
     A       - accel percent (1..100) (optional)

 - Also supports plain text commands: HELLO, BYE, STOP
 - On startup prints "READY" so the GUI can detect device
 - Smooth (non-blocking) motion implemented with millis() updates
 - Edit SERVO_PINS and NUM_SERVOS below to match your wiring

 Notes / mapping decisions:
 - GUI uses raw 0..1023 positions. We map that linearly to 0..180 degrees for hobby servos.
 - Speed is interpreted as a percent: higher = faster. We translate speed into step delay.
 - Accel is used to ramp speeds smoothly (simple implementation: adjust step delay gradually).

 Hardware: Arduino Uno/Nano (Servo library)
 */

#include <Servo.h>

// ---------- Config: edit these to match your wiring ----------
// Provide one pin per servo. Example uses pins 2..8 inclusive (7 servos).
const int SERVO_PINS[] = {2, 3, 4, 5, 6, 7}; // edit/add/remove pins as needed
const int NUM_SERVOS = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);

// Min/max degrees to use when mapping raw (0..1023) -> degrees.
const int DEGREE_MIN = 0;
const int DEGREE_MAX = 180;

// Default behavior tuning
const unsigned long BASE_STEP_MS_AT_SPEED_100 = 3; // base delay per step at speed=100 (reduced for smoother motion)
const unsigned long MIN_STEP_MS = 1; // fastest allowed per-step delay

// ---------- Per-servo runtime state ----------
Servo servos[NUM_SERVOS];
volatile int targetRaw[NUM_SERVOS];     // target position in raw units (0..1023)
volatile int currentRaw[NUM_SERVOS];    // current simulated raw position
volatile int currentDeg[NUM_SERVOS];    // cached current deg for writing to servo
volatile int targetDeg[NUM_SERVOS];     // target degrees
volatile int activeSpeed[NUM_SERVOS];   // speed percent (1..100)
volatile int activeAccel[NUM_SERVOS];   // accel percent (1..100)
unsigned long lastStepTime[NUM_SERVOS];

// Parser buffer
String rxBuffer = "";

// Control flag
volatile bool stopAll = false;

// Helper: map raw (0..1023) to degree (DEGREE_MIN..DEGREE_MAX)
int rawToDeg(int raw) {
  raw = constrain(raw, 0, 1023);
  long deg = map(raw, 0, 1023, DEGREE_MIN, DEGREE_MAX);
  return (int)deg;
}

// Helper: compute per-degree step delay (ms) from speed percent and accel
unsigned long computeStepDelayMs(int speedPct, int accelPct, int remainingDelta) {
  // speedPct in 1..100, accelPct 1..100
  speedPct = constrain(speedPct, 1, 100);
  accelPct = constrain(accelPct, 1, 100);

  // Base delay scales inversely with speed
  // At speed=100 -> BASE_STEP_MS_AT_SPEED_100, at speed=50 -> about 2x delay
  float speedFactor = 100.0f / float(speedPct);
  float delay = float(BASE_STEP_MS_AT_SPEED_100) * speedFactor;

  // Simple accel model: when remaining distance is large, use faster effective accelPct
  // accelerate more when large remainingDelta by scaling the delay down according to accel
  float accelFactor = float(100) / float(accelPct);
  // We will reduce delay slightly when remainingDelta is large and accelPct high
  if (remainingDelta > 0) {
    float remFactor = 1.0f + (float(remainingDelta) / 50.0f); // larger rem -> bigger factor
    delay = delay / (1.0f + (float(accelPct) / 200.0f) * remFactor);
  }

  unsigned long d = (unsigned long)max((float)MIN_STEP_MS, delay);
  return d;
}

// Non-blocking move step for one servo
void servoStepIfNeeded(int idx) {
  if (idx < 0 || idx >= NUM_SERVOS) return;
  if (stopAll) return;

  int cur = currentRaw[idx];
  int tgt = targetRaw[idx];
  if (cur == tgt) return;

  int diff = tgt - cur;
  int dir = (diff > 0) ? 1 : -1;
  int remaining = abs(diff);

  unsigned long now = millis();
  unsigned long delayMs = computeStepDelayMs(activeSpeed[idx], activeAccel[idx] == 0 ? 100 : activeAccel[idx], remaining);

  if (now - lastStepTime[idx] < delayMs) return; // not yet

  // take a single raw step; step size 1 raw unit is slow (0..1023 -> 1024 steps) so we step in larger raw deltas
  // Larger steps = smoother motion when receiving frequent position updates from GUI
  int rawPerDeg = max(1, 1023 / (DEGREE_MAX - DEGREE_MIN));
  int rawStep = rawPerDeg * 3; // move roughly 3 degrees per update for smoother motion
  if (remaining < rawStep) rawStep = remaining;

  currentRaw[idx] += dir * rawStep;
  currentRaw[idx] = constrain(currentRaw[idx], 0, 1023);
  // update physical servo only if mapped degree changed
  int deg = rawToDeg(currentRaw[idx]);
  if (deg != currentDeg[idx]) {
    currentDeg[idx] = deg;
    servos[idx].write(deg);
  }
  lastStepTime[idx] = now;
}

// Parse a single command string (no trailing newline)
void handleCommand(const String &cmd) {
  if (cmd.length() == 0) return;
  // Trim whitespace
  String s = cmd;
  while (s.length() && isspace(s.charAt(0))) s.remove(0,1);
  while (s.length() && isspace(s.charAt(s.length()-1))) s.remove(s.length()-1,1);
  if (s.length() == 0) return;

  // Handle simple text commands
  if (s.equalsIgnoreCase("HELLO")) {
    Serial.println("READY");
    return;
  }
  if (s.equalsIgnoreCase("BYE")) {
    Serial.println("BYE");
    return;
  }
  if (s.equalsIgnoreCase("STOP")) {
    // Stop movements
    stopAll = true;
    // set targets to current to halt movements
    for (int i=0;i<NUM_SERVOS;i++) {
      targetRaw[i] = currentRaw[i];
    }
    Serial.println("STOPPED");
    return;
  }

  // Support commands starting with '#'
  if (s.charAt(0) != '#') return;

  // Parse pattern: #<id>P<pos>(S<speed>)(A<accel>)
  int i = 1;
  // parse id (1-based)
  long id = 0;
  while (i < s.length() && isDigit(s.charAt(i))) {
    id = id * 10 + (s.charAt(i) - '0');
    i++;
  }
  if (id < 1 || id > NUM_SERVOS) {
    // id out of range - ignore
    return;
  }
  int idx = (int)id - 1;

  // default extras
  long pos = -1;
  int speed = 100;
  int accel = 100;

  // scan the rest
  while (i < s.length()) {
    char c = s.charAt(i);
    if (c == 'P' || c == 'p') {
      i++;
      long v = 0;
      bool neg = false;
      while (i < s.length() && isDigit(s.charAt(i))) { v = v*10 + (s.charAt(i)-'0'); i++; }
      pos = v;
    } else if (c == 'S' || c == 's') {
      i++;
      long v = 0;
      while (i < s.length() && isDigit(s.charAt(i))) { v = v*10 + (s.charAt(i)-'0'); i++; }
      speed = (int)v;
    } else if (c == 'A' || c == 'a') {
      i++;
      long v = 0;
      while (i < s.length() && isDigit(s.charAt(i))) { v = v*10 + (s.charAt(i)-'0'); i++; }
      accel = (int)v;
    } else {
      // skip unknown
      i++;
    }
  }

  if (pos >= 0) {
    pos = constrain(pos, 0, 1023);
    targetRaw[idx] = (int)pos;
    targetDeg[idx] = rawToDeg((int)pos);
    // set active speed/accel
    activeSpeed[idx] = constrain(speed, 1, 200);
    activeAccel[idx] = constrain(accel, 1, 200);
    stopAll = false; // resume motion if previously stopped
    // Send acknowledgement with received command
    Serial.print("RX: #");
    Serial.print(id);
    Serial.print("P");
    Serial.print(pos);
    Serial.print("S");
    Serial.print(speed);
    if (accel != 100) {
      Serial.print("A");
      Serial.print(accel);
    }
    Serial.println(" OK");
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);
  // attach servos and initialize states
  for (int i=0;i<NUM_SERVOS;i++) {
    servos[i].attach(SERVO_PINS[i]);
    // init current to mid (511) or 0
    currentRaw[i] = 511;
    currentDeg[i] = rawToDeg(currentRaw[i]);
    targetRaw[i] = currentRaw[i];
    targetDeg[i] = currentDeg[i];
    activeSpeed[i] = 100;
    activeAccel[i] = 100;
    lastStepTime[i] = millis();
    servos[i].write(currentDeg[i]);
  }

  // send READY so GUI can detect
  Serial.println("READY");
}

void loop() {
  // read serial and accumulate until newline
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuffer.length() > 0) {
        handleCommand(rxBuffer);
        rxBuffer = "";
      }
    } else {
      rxBuffer += c;
      // keep buffer manageable
      if (rxBuffer.length() > 200) rxBuffer = rxBuffer.substring(rxBuffer.length()-200);
    }
  }

  // update servo steps non-blocking
  for (int i=0;i<NUM_SERVOS;i++) {
    servoStepIfNeeded(i);
  }
}
