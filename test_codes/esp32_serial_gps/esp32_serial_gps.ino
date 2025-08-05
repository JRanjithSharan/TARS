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

// Elevation drift compensation factor (during azimuth)
float elevationFactor = 1.0;  // 1.0 = same number of steps

// Ground station location
const float groundLat = 30.89841339171403;
const float groundLon = -102.90897836824136;
const float groundAlt = 923;  // meters above sea level

// Current orientation of the antenna (degrees)
float currentAz = 0.0;
float currentEl = 0.0;

void setup() {
  pinMode(ELEV_DIR, OUTPUT);
  pinMode(ELEV_PUL, OUTPUT);
  pinMode(ELEV_EN, OUTPUT);

  pinMode(AZIM_DIR, OUTPUT);
  pinMode(AZIM_PUL, OUTPUT);
  pinMode(AZIM_EN, OUTPUT);

  digitalWrite(ELEV_EN, LOW);  // Enable stepper drivers
  digitalWrite(AZIM_EN, LOW);

  Serial.begin(115200);
  Serial.println("========================================");
  Serial.println("TARS - Tracking Antenna Reception System");
  Serial.println("Assumes initial orientation: NORTH (Az=0°, El=0°)");
  Serial.println("Step 1: Azimuth rotates first (with elevation compensation)");
  Serial.println("Step 2: Elevation rotates independently");
  Serial.println("Enter GPS as: latitude,longitude,altitude");
  Serial.println("========================================");
}

void loop() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      processGPS(input);
      input = "";
    } else {
      input += c;
    }
  }
}

// Parse and process GPS input
void processGPS(String line) {
  int idx1 = line.indexOf(',');
  int idx2 = line.lastIndexOf(',');

  if (idx1 == -1 || idx2 == -1 || idx1 == idx2) {
    Serial.println("❌ Invalid format. Use: lat,lon,alt");
    return;
  }

  float lat = line.substring(0, idx1).toFloat();
  float lon = line.substring(idx1 + 1, idx2).toFloat();
  float alt = line.substring(idx2 + 1).toFloat();

  float azimuth, elevation;
  calculateAzEl(groundLat, groundLon, groundAlt, lat, lon, alt, azimuth, elevation);

  Serial.println("------ Target Calculated ------");
  Serial.print("Target Azimuth: "); Serial.print(azimuth); Serial.println("°");
  Serial.print("Target Elevation: "); Serial.print(elevation); Serial.println("°");

  moveToAzimuthWithElevationComp(azimuth);  // Step 1: horizontal rotation (with compensation)
  delay(500);                               // small buffer
  moveToElevation(elevation);               // Step 2: independent elevation adjustment
}

// Calculate azimuth and elevation to target GPS
void calculateAzEl(float lat1, float lon1, float alt1,
                   float lat2, float lon2, float alt2,
                   float &azimuth, float &elevation) {
  const float R = 6371000;

  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float lat1Rad = radians(lat1);
  float lat2Rad = radians(lat2);

  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1Rad) * cos(lat2Rad) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float horizontalDistance = R * c;

  float deltaAlt = alt2 - alt1;
  elevation = atan2(deltaAlt, horizontalDistance) * 180.0 / PI;
  if (elevation < 0) elevation = 0;

  float y = sin(dLon) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) -
            sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
  azimuth = atan2(y, x) * 180.0 / PI;
  if (azimuth < 0) azimuth += 360;
}

// Rotate azimuth motor AND elevation simultaneously (compensated)
void moveToAzimuthWithElevationComp(float targetAz) {
  float deltaAz = targetAz - currentAz;
  if (deltaAz < -180) deltaAz += 360;
  if (deltaAz > 180) deltaAz -= 360;

  int azSteps = abs(deltaAz) * stepsPerDegree;
  int elSteps = azSteps * elevationFactor;

  bool azDir = (deltaAz >= 0) ? false : true; // false = CW, true = CCW

  Serial.print("Rotating AZ by "); Serial.print(deltaAz);
  Serial.print("°, EL Compensation Steps: "); Serial.println(elSteps);

  stepMotorDual(AZIM_DIR, AZIM_PUL, ELEV_DIR, ELEV_PUL, azDir, azSteps, elSteps);

  currentAz = targetAz;
}

// Rotate elevation only
void moveToElevation(float targetEl) {
  float deltaEl = targetEl - currentEl;
  int elSteps = abs(deltaEl) * stepsPerDegree;
  bool elDir = (deltaEl >= 0) ? true : false; // true = UP, false = DOWN
 // INVERTED: true = DOWN, false = UP

  Serial.print("Adjusting Elevation by "); Serial.print(deltaEl); Serial.println("°");

  stepMotor(ELEV_DIR, ELEV_PUL, elDir, elSteps);

  currentEl = targetEl;
}

// Standard single motor step
void stepMotor(int dirPin, int pulPin, bool direction, int steps) {
  digitalWrite(dirPin, direction);
  for (int i = 0; i < steps; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// Step both motors at once (azimuth + compensated elevation)
void stepMotorDual(int dirA, int pulA, int dirB, int pulB,
                   bool direction, int azSteps, int elSteps) {
  digitalWrite(dirA, direction);
  digitalWrite(dirB, direction);

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
