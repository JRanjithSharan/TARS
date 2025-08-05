#define PUL_PIN1 14  // D5 - Azimuth pulse pin
#define DIR_PIN1 12  // D6 - Azimuth direction pin
#define PUL_PIN2 13  // D7 - Elevation pulse pin
#define DIR_PIN2 15  // D8 - Elevation direction pin

// TB6600 requires specific timing parameters
const int pulseWidth = 10;       // Pulse width in microseconds for TB6600
const int pulsePause = 10;       // Pause between pulses
const int stepDelay = 800;       // Overall step delay for smooth movement

// Stepper motor configuration
const float stepAngle = 1.8;     // Standard for many steppers
const int microstepping = 8;     // Set to match your TB6600 microstepping setting
const int stepsPerRev = (360 / stepAngle) * microstepping;
const float stepsPerDegree = stepsPerRev / 360.0;

// Current position tracking (in steps)
long currentAzimuthSteps = 0;    // Azimuth position
long currentElevationSteps = 0;  // Elevation position

// Home position angles (in degrees)
const float homeAzimuth = 0.0;   // North
const float homeElevation = 30.0; // Default elevation

// Ground station coordinates (example: VIT Vellore)
const float lat0 = 12.9716;
const float lon0 = 79.1588;
const float alt0 = 0.0;

void setup() {
  // Initialize motor control pins
  pinMode(PUL_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(PUL_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  
  // Set initial state for all pins
  digitalWrite(PUL_PIN1, LOW);
  digitalWrite(PUL_PIN2, LOW);
  digitalWrite(DIR_PIN1, HIGH);
  digitalWrite(DIR_PIN2, HIGH);
  
  // Start serial communication
  Serial.begin(115200);
  delay(1000); // Give time for the serial connection to establish
  
  Serial.println("\n\n===== GPS Pointer System =====");
  Serial.println("Initializing...");
  
  // Perform homing sequence
  goToHomePosition();
  
  Serial.println("Ready! Enter GPS: lat,lon,alt");
}

void loop() {
  static String input = "";
  static bool commandReceived = false;

  // Check for incoming serial data
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      commandReceived = true;
    } else {
      input += c;
    }
    delay(1); // prevent watchdog reset
  }

  // Process complete command if received
  if (commandReceived) {
    input.trim();
    if (input.length() > 0) {
      if (input == "home") {
        goToHomePosition();
      } else {
        processGPS(input);
      }
    }
    input = "";
    commandReceived = false;
  }

  delay(10);
}

void processGPS(String gpsInput) {
  int c1 = gpsInput.indexOf(',');
  int c2 = gpsInput.indexOf(',', c1 + 1);
  if (c1 == -1 || c2 == -1) {
    Serial.println("Invalid input. Use: lat,lon,alt");
    return;
  }

  float lat = gpsInput.substring(0, c1).toFloat();
  float lon = gpsInput.substring(c1 + 1, c2).toFloat();
  float alt = gpsInput.substring(c2 + 1).toFloat();

  Serial.print("Target Coordinates: ");
  Serial.print(lat, 6); Serial.print(", ");
  Serial.print(lon, 6); Serial.print(", ");
  Serial.println(alt, 2);

  float azimuth, elevation;
  calculateAzEl(lat, lon, alt, azimuth, elevation);

  Serial.print("Calculated Azimuth: ");
  Serial.println(azimuth, 6);
  Serial.print("Calculated Elevation: ");
  Serial.println(elevation, 6);

  moveToPosition(azimuth, elevation);
}

void calculateAzEl(float lat, float lon, float alt, float &azimuth, float &elevation) {
  // Haversine formula to calculate distance and bearing
  float dLat = radians(lat - lat0);
  float dLon = radians(lon - lon0);
  float lat1 = radians(lat0);
  float lat2 = radians(lat);

  // Calculate distance between points
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = 6371000.0 * c; // Distance in meters

  // Calculate bearing (Azimuth)
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  azimuth = atan2(y, x);
  azimuth = degrees(azimuth);
  if (azimuth < 0) azimuth += 360;

  // Calculate elevation angle
  float deltaAlt = alt - alt0;
  elevation = atan2(deltaAlt, distance);
  elevation = degrees(elevation);
  if (elevation < 0) elevation = 0;  // Prevent pointing below horizon
  if (elevation > 90) elevation = 90; // Prevent pointing beyond vertical
}

void goToHomePosition() {
  Serial.println("Moving to home position (North, 30° elevation)...");
  
  // First move to zero position to ensure consistent positioning
  moveToPosition(0, 0);
  delay(1000);
  
  // Then move to the actual home position
  moveToPosition(homeAzimuth, homeElevation);
  
  Serial.println("Home position reached.");
}

void moveToPosition(float azimuthAngle, float elevationAngle) {
  // Convert angles to steps
  long targetAzimuthSteps = (long)(azimuthAngle * stepsPerDegree);
  long targetElevationSteps = (long)(elevationAngle * stepsPerDegree);
  
  // Safety limits for elevation (0-90 degrees)
  if (elevationAngle < 0) {
    targetElevationSteps = 0;
    Serial.println("Warning: Elevation limited to 0 degrees");
  } else if (elevationAngle > 90) {
    targetElevationSteps = (long)(90 * stepsPerDegree);
    Serial.println("Warning: Elevation limited to 90 degrees");
  }

  // Calculate steps to move for each motor
  long azimuthStepsToMove = targetAzimuthSteps - currentAzimuthSteps;
  long elevationStepsToMove = targetElevationSteps - currentElevationSteps;

  // Move both motors simultaneously
  moveMotorsSimultaneously(azimuthStepsToMove, elevationStepsToMove);
  
  // Update current positions
  currentAzimuthSteps = targetAzimuthSteps;
  currentElevationSteps = targetElevationSteps;

  Serial.print("Position reached - Azimuth: ");
  Serial.print(azimuthAngle, 6);
  Serial.print("° (");
  Serial.print(currentAzimuthSteps);
  Serial.print(" steps), Elevation: ");
  Serial.print(elevationAngle, 6);
  Serial.print("° (");
  Serial.print(currentElevationSteps);
  Serial.println(" steps)");
}

void moveMotorsSimultaneously(long azimuthSteps, long elevationSteps) {
  // Determine direction for each motor
  bool azimuthDir = (azimuthSteps >= 0);
  bool elevationDir = (elevationSteps >= 0);
  
  // Set direction pins
  digitalWrite(DIR_PIN1, azimuthDir ? HIGH : LOW);
  digitalWrite(DIR_PIN2, elevationDir ? HIGH : LOW);
  
  // Convert steps to absolute values
  azimuthSteps = abs(azimuthSteps);
  elevationSteps = abs(elevationSteps);
  
  // Track progress
  long azimuthDone = 0;
  long elevationDone = 0;
  long totalSteps = max(azimuthSteps, elevationSteps);
  
  // Move both motors in parallel with proper speed ratio
  for (long i = 0; i < totalSteps; i++) {
    bool azimuthStep = (azimuthDone < azimuthSteps);
    bool elevationStep = (elevationDone < elevationSteps);
    
    // Pulse azimuth motor if needed
    if (azimuthStep) {
      digitalWrite(PUL_PIN1, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(PUL_PIN1, LOW);
      azimuthDone++;
    }
    
    // Pulse elevation motor if needed
    if (elevationStep) {
      digitalWrite(PUL_PIN2, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(PUL_PIN2, LOW);
      elevationDone++;
    }
    
    // Delay between steps for smooth movement
    delayMicroseconds(stepDelay);
    
    // Report progress every 500 steps
    if (i % 500 == 0 && i > 0) {
      Serial.print("Moving... ");
      Serial.print((i * 100) / totalSteps);
      Serial.println("%");
      yield(); // Prevent watchdog timer reset on ESP8266
    }
  }
}

// Single motor movement function for simpler operations
void moveSingleMotor(int pulPin, int dirPin, long steps, long &currentPos) {
  bool direction = (steps >= 0);
  digitalWrite(dirPin, direction ? HIGH : LOW);
  steps = abs(steps);
  
  for (long i = 0; i < steps; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(stepDelay);
    
    if (i % 500 == 0 && i > 0) {
      yield(); // Prevent watchdog timer reset
    }
  }
  
  // Update current position
  currentPos = direction ? currentPos + steps : currentPos - steps;
}