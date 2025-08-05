// Stepper motor configuration
const float stepAngle = 1.8;
const int microstepping = 16;
const int stepsPerRev = (360 / stepAngle) * microstepping;
const float stepsPerDegree = stepsPerRev / 360.0;

#define ELEV_DIR 18
#define ELEV_PUL 19
#define ELEV_EN  23

#define AZIM_DIR 21
#define AZIM_PUL 22
#define AZIM_EN  25

#define STEP_DELAY_US 800

// Compensation factor: elevation steps = azimuth steps × this factor
float elevationFactor = 1.0;  // Try 1.05, 0.95, etc.

void setup() {
  pinMode(ELEV_DIR, OUTPUT);
  pinMode(ELEV_PUL, OUTPUT);
  pinMode(ELEV_EN, OUTPUT);

  pinMode(AZIM_DIR, OUTPUT);
  pinMode(AZIM_PUL, OUTPUT);
  pinMode(AZIM_EN, OUTPUT);

  digitalWrite(ELEV_EN, LOW); // Enable drivers
  digitalWrite(AZIM_EN, LOW);

  Serial.begin(115200);
  Serial.println("Starting continuous test with elevation factor...");
}

void loop() {
  // Elevation: 30° up and down (independent)
  rotateAzimuthCW(100);
  delay(500);

  moveElevationUp(50);
  delay(500);
  moveElevationDown(50);
  delay(500);

  // Azimuth: 45° CW and CCW (with scaled elevation sync)
  rotateAzimuthCCW(100);
  delay(500);
   moveElevationUp(50);
  delay(500);
  moveElevationDown(50);
  delay(500);
}

// Simple single motor step
void stepMotor(int dirPin, int pulPin, bool direction, int steps) {
  digitalWrite(dirPin, direction);
  for (int i = 0; i < steps; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// Azimuth with elevation scaled sync
void stepMotorDualWithFactor(int dirA, int pulA, int dirB, int pulB, bool direction, int azSteps) {
  int elSteps = azSteps * elevationFactor;

  digitalWrite(dirA, direction);  // Azimuth direction
  digitalWrite(dirB, direction);  // Elevation direction (same)

  int maxSteps = max(azSteps, elSteps);

  for (int i = 0; i < maxSteps; i++) {
    if (i < azSteps) digitalWrite(pulA, HIGH);
    if (i < elSteps) digitalWrite(pulB, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(pulA, LOW);
    digitalWrite(pulB, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// Azimuth rotation commands
void rotateAzimuthCW(float degrees) {
  int azSteps = degrees * stepsPerDegree;
  stepMotorDualWithFactor(AZIM_DIR, AZIM_PUL, ELEV_DIR, ELEV_PUL, false, azSteps); // CCW motor = CW antenna
  Serial.print("Azimuth CW: "); Serial.print(degrees);
  Serial.print("°, Elevation Factor: "); Serial.println(elevationFactor);
}

void rotateAzimuthCCW(float degrees) {
  int azSteps = degrees * stepsPerDegree;
  stepMotorDualWithFactor(AZIM_DIR, AZIM_PUL, ELEV_DIR, ELEV_PUL, true, azSteps); // CW motor = CCW antenna
  Serial.print("Azimuth CCW: "); Serial.print(degrees);
  Serial.print("°, Elevation Factor: "); Serial.println(elevationFactor);
}

// Elevation-only controls
void moveElevationUp(float degrees) {
  int steps = degrees * stepsPerDegree;
  stepMotor(ELEV_DIR, ELEV_PUL, true, steps); // CW = up
  Serial.print("Elevation UP: "); Serial.println(degrees);
}

void moveElevationDown(float degrees) {
  int steps = degrees * stepsPerDegree;
  stepMotor(ELEV_DIR, ELEV_PUL, false, steps); // CCW = down
  Serial.print("Elevation DOWN: "); Serial.println(degrees);
}
