// ==============================
// ELEVATOR CONTROL - 4 FLOORS
// Arduino Uno + Ultrasonic Sensor + TB6600 Microstep Driver
// Uses AccelStepper library for smooth, non-blocking motion
// ==============================
//
// REQUIRED LIBRARY:
// Install "AccelStepper" by Mike McCauley via Library Manager
// (Sketch -> Include Library -> Manage Libraries -> search "AccelStepper")
//
// WIRING NOTE:
// This sketch assumes common-anode wiring on the TB6600:
//   PUL+, DIR+, ENA+ -> Arduino 5V
//   PUL- -> pin 5, DIR- -> pin 6, ENA- -> pin 3 (or leave ENA disconnected)
// AccelStepper drives the pins with normal HIGH/LOW; the opto inversion
// is handled by setPinsInverted() below.
//
// If your motor turns the wrong direction, flip the first argument of
// setPinsInverted() (the DIR invert flag).
//
// ==============================

#include <AccelStepper.h>

// ---------- Ultrasonic Sensor Pins ----------
const int trigPin = 9;
const int echoPin = 10;

// ---------- Stepper Driver Pins ----------
const int stepPin = 5;      // PUL-
const int dirPin = 6;       // DIR-
const int enablePin = 3;    // ENA-  (optional; can be left disconnected)

// ---------- Motion Tuning ----------
// Steps the motor must turn to travel between adjacent floors.
// CALIBRATE THIS: jog the elevator from one floor to the next and count
// the steps. At 1/16 microstepping, one rev = 3200 steps. - Steps per floor should be around 10571
const long STEPS_PER_FLOOR = 8000;

// Speed and acceleration in steps/sec and steps/sec^2.
// Start conservative, then push these up once motion looks good.
const float MAX_SPEED = 6000.0;      // top speed (steps/sec)
const float ACCELERATION = 12000.0;   // ramp rate (steps/sec^2)

// ---------- Floor Button Pins ----------
const int floorButtonPins[4] = {2, 4, 7, 8};

// ---------- Elevator Configuration ----------
const int NUM_FLOORS = 4;

// ---------- Floor Indicator LED Pins ----------
const int floorLEDPins[NUM_FLOORS] = {A1, A2, A3, A4};

// Calibrated floor distances in cm from ultrasonic sensor (for sanity-check
// and startup floor detection only -- actual positioning is by step count).
float floorDistances[NUM_FLOORS] = {10.6, 21.2, 31.8, 42.4};

// Tolerance band for ultrasonic floor detection. Loose enough to actually
// match -- the HC-SR04 is not a 0.1 cm device.
const float SENSOR_TOLERANCE = 1.5;

// Step positions for each floor (computed in setup based on starting floor).
long floorStepPositions[NUM_FLOORS];

// Idle return time: 5 minutes
const unsigned long IDLE_RETURN_TIME = 300000UL;

// Door dwell time when arriving at a floor
const unsigned long DOOR_DWELL_MS = 3000UL;

// ---------- State ----------
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

bool requests[NUM_FLOORS] = {false, false, false, false};

int currentFloor = 0;       // 0 = floor 1
int targetFloor = -1;       // -1 = no active target

enum Direction { IDLE, UP, DOWN };
Direction direction = IDLE;

unsigned long lastActivityTime = 0;
unsigned long doorOpenedAt = 0;
bool doorsOpen = false;

// ==============================
// SETUP
// ==============================
void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  for (int i = 0; i < NUM_FLOORS; i++) {
    pinMode(floorButtonPins[i], INPUT_PULLUP);
  }

  for (int i = 0; i < NUM_FLOORS; i++) {
    pinMode(floorLEDPins[i], OUTPUT);
    digitalWrite(floorLEDPins[i], LOW);
  }

  // Configure stepper.
  // setPinsInverted(directionInvert, stepInvert, enableInvert)
  // Common-anode wiring inverts step and enable logic.
  stepper.setPinsInverted(false, true, true);
  stepper.setEnablePin(enablePin);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.enableOutputs();

  // Determine starting floor from sensor (best effort).
  delay(100);
  float distance = getFilteredDistance();
  int detected = determineFloorFromDistance(distance);
  if (detected < 0) detected = 0;
  currentFloor = detected;

  // Build step-position map relative to the starting floor.
  // Floor 0 is the bottom; positive steps move UP.
  for (int i = 0; i < NUM_FLOORS; i++) {
    floorStepPositions[i] = (long)(i - currentFloor) * STEPS_PER_FLOOR;
  }
  stepper.setCurrentPosition(floorStepPositions[currentFloor]);

  lastActivityTime = millis();

  Serial.print("Elevator initialized at Floor ");
  Serial.println(currentFloor + 1);
  updateFloorLEDs();
}

// ==============================
// MAIN LOOP
// ==============================
void loop() {
  // 1. ALWAYS service the stepper. This MUST be called as often as possible
  //    -- it is what actually generates step pulses. Anything that blocks
  //    this call (long delays, slow sensor reads) will slow the motor.
  stepper.run();

  // 2. Read buttons (non-blocking debounce inside).
  readButtons();

  // 3. If doors are open, wait out the dwell time before doing anything else.
  if (doorsOpen) {
    if (millis() - doorOpenedAt >= DOOR_DWELL_MS) {
      doorsOpen = false;
      Serial.println("Doors close.");
      chooseNextTarget();
    }
    return;  // keep stepper.run() ticking via next loop iteration
  }

  // 4. If the stepper just finished a move, we've arrived at the target.
  if (targetFloor >= 0 && stepper.distanceToGo() == 0) {
    arriveAtFloor(targetFloor);
    return;
  }

  // 5. If idle and there are requests, pick a target.
  if (targetFloor < 0 && hasAnyRequests()) {
    chooseNextTarget();
  }

  // 6. Idle return-to-lobby logic.
  if (!hasAnyRequests() && targetFloor < 0) {
    direction = IDLE;
    if ((millis() - lastActivityTime >= IDLE_RETURN_TIME) && currentFloor != 0) {
      requests[0] = true;
      Serial.println("Idle timeout. Returning to Floor 1.");
    }
  }

  // 7. Periodic status print (rate-limited inside).
  printStatus();
}

// ==============================
// BUTTON INPUT (non-blocking)
// ==============================
void readButtons() {
  static unsigned long lastPress[NUM_FLOORS] = {0, 0, 0, 0};
  const unsigned long debounceMs = 200;

  for (int i = 0; i < NUM_FLOORS; i++) {
    if (digitalRead(floorButtonPins[i]) == LOW) {
      if (millis() - lastPress[i] >= debounceMs) {
        lastPress[i] = millis();
        if (!requests[i]) {
          requests[i] = true;
          lastActivityTime = millis();
          Serial.print("Request added for Floor ");
          Serial.println(i + 1);
        }
      }
    }
  }
}

// ==============================
// REQUEST / DIRECTION LOGIC
// ==============================
bool hasAnyRequests() {
  for (int i = 0; i < NUM_FLOORS; i++) {
    if (requests[i]) return true;
  }
  return false;
}

bool hasRequestsAbove() {
  for (int i = currentFloor + 1; i < NUM_FLOORS; i++) {
    if (requests[i]) return true;
  }
  return false;
}

bool hasRequestsBelow() {
  for (int i = 0; i < currentFloor; i++) {
    if (requests[i]) return true;
  }
  return false;
}

// Pick the next floor to serve, set the stepper target, and update direction.
void chooseNextTarget() {
  // Already at a requested floor -> serve it now.
  if (requests[currentFloor]) {
    arriveAtFloor(currentFloor);
    return;
  }

  int next = -1;

  if (direction == UP || direction == IDLE) {
    // Prefer continuing up if there are requests above
    for (int i = currentFloor + 1; i < NUM_FLOORS; i++) {
      if (requests[i]) { next = i; direction = UP; break; }
    }
    if (next < 0) {
      for (int i = currentFloor - 1; i >= 0; i--) {
        if (requests[i]) { next = i; direction = DOWN; break; }
      }
    }
  } else { // direction == DOWN
    for (int i = currentFloor - 1; i >= 0; i--) {
      if (requests[i]) { next = i; direction = DOWN; break; }
    }
    if (next < 0) {
      for (int i = currentFloor + 1; i < NUM_FLOORS; i++) {
        if (requests[i]) { next = i; direction = UP; break; }
      }
    }
  }

  if (next >= 0) {
    targetFloor = next;
    stepper.moveTo(floorStepPositions[next]);
    Serial.print("Heading to Floor ");
    Serial.print(next + 1);
    Serial.print(" (");
    Serial.print(direction == UP ? "UP" : "DOWN");
    Serial.println(")");
  } else {
    targetFloor = -1;
    direction = IDLE;
  }
}

// ==============================
// ARRIVAL HANDLING
// ==============================
void arriveAtFloor(int floor) {
  currentFloor = floor;
  updateFloorLEDs();
  requests[floor] = false;
  targetFloor = -1;
  lastActivityTime = millis();

  // Sanity check against ultrasonic if you want -- log mismatch but trust steps.
  float d = getFilteredDistance();
  if (d > 0) {
    int detected = determineFloorFromDistance(d);
    if (detected >= 0 && detected != floor) {
      Serial.print("WARNING: sensor sees Floor ");
      Serial.print(detected + 1);
      Serial.print(" but step count says Floor ");
      Serial.println(floor + 1);
    }
  }

  Serial.print("Arrived at Floor ");
  Serial.println(floor + 1);
  Serial.println("Doors open...");
  doorsOpen = true;
  doorOpenedAt = millis();
}

// ==============================
// FLOOR INDICATOR LEDS
// ==============================
void updateFloorLEDs() {
  for (int i = 0; i < NUM_FLOORS; i++) {
    digitalWrite(floorLEDPins[i], (i == currentFloor) ? HIGH : LOW);
  }
}

// ==============================
// FLOOR DETECTION (sensor-based, used for startup + sanity)
// ==============================
int determineFloorFromDistance(float distance) {
  if (distance < 0) return -1;
  for (int i = 0; i < NUM_FLOORS; i++) {
    if (distance >= (floorDistances[i] - SENSOR_TOLERANCE) &&
        distance <= (floorDistances[i] + SENSOR_TOLERANCE)) {
      return i;
    }
  }
  return -1;
}

// ==============================
// ULTRASONIC SENSOR
// ==============================
// Single-shot read. Kept short and non-blocking-ish (pulseIn timeout = 30 ms).
float getDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1.0;
  return duration * 0.0343 / 2.0;
}

// Filtered read. Used at startup and at arrival -- NOT in the hot loop, so
// the 60ms inter-sample delays don't hurt motor performance.
float getFilteredDistance() {
  const int samples = 5;
  float total = 0.0;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    float d = getDistanceCM();
    if (d > 0) { total += d; valid++; }
    // 60ms gives the HC-SR04 transducer time to stop ringing between pings.
    // We still call stepper.run() in case a move is in progress.
    unsigned long t0 = millis();
    while (millis() - t0 < 60) stepper.run();
  }

  if (valid == 0) return -1.0;
  return total / valid;
}

// ==============================
// DEBUG OUTPUT
// ==============================
void printStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 500) return;
  lastPrint = millis();

  Serial.print("Floor: ");
  Serial.print(currentFloor + 1);
  Serial.print(" | Pos: ");
  Serial.print(stepper.currentPosition());
  Serial.print(" | Target: ");
  if (targetFloor >= 0) Serial.print(targetFloor + 1);
  else Serial.print("-");
  Serial.print(" | Dir: ");
  if (direction == UP) Serial.print("UP");
  else if (direction == DOWN) Serial.print("DOWN");
  else Serial.print("IDLE");
  Serial.print(" | Speed: ");
  Serial.print(stepper.speed(), 0);
  Serial.print(" | Requests: ");
  for (int i = 0; i < NUM_FLOORS; i++) {
    Serial.print(requests[i] ? "[X]" : "[ ]");
  }
  Serial.println();
}
