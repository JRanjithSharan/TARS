#include <AccelStepper.h>

// Stepper Pins
#define AZ_STEP_PIN D1
#define AZ_DIR_PIN D2
#define EL_STEP_PIN D3
#define EL_DIR_PIN D4

const float stepAngle = 1.8;
const int microstepping = 8;
const int stepsPerRev = (360 / stepAngle) * microstepping;
const float stepsPerDegree = stepsPerRev / 360.0;
const float gearRatioAz = 1.0;
const float gearRatioEl = 1.0;

AccelStepper stepperAz(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
AccelStepper stepperEl(AccelStepper::DRIVER, EL_STEP_PIN, EL_DIR_PIN);

// Ground station reference
const float groundLat = 12.9721;
const float groundLon = 77.5933;
const float groundAlt = 900;
const float initialAzimuth = 0.0;

unsigned long lastPacketTime = 0;
const unsigned long packetTimeout = 5000;

void setup() {
  Serial.begin(115200);

  stepperAz.setMaxSpeed(1000);
  stepperAz.setAcceleration(500);
  stepperEl.setMaxSpeed(1000);
  stepperEl.setAcceleration(500);

  Serial.println("ESP8266 Antenna Tracker ready.");
}

void loop() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processLine(input);
      input = "";
    } else {
      input += c;
    }
  }

  stepperAz.run();
  stepperEl.run();

  if (millis() - lastPacketTime > packetTimeout) {
    Serial.println("No new GPS data. Holding position.");
  }
}

void processLine(String line) {
  lastPacketTime = millis();
  line.trim();
  int idx1 = line.indexOf(',');
  int idx2 = line.lastIndexOf(',');

  if (idx1 == -1 || idx2 == -1 || idx2 <= idx1) return;

  float lat = line.substring(0, idx1).toFloat();
  float lon = line.substring(idx1 + 1, idx2).toFloat();
  float alt = line.substring(idx2 + 1).toFloat();

  float azimuth, elevation;
  calculateAzEl(groundLat, groundLon, groundAlt, lat, lon, alt, azimuth, elevation);

  float correctedAz = azimuth - initialAzimuth;
  if (correctedAz < 0) correctedAz += 360;

  float azSteps = correctedAz * stepsPerDegree * gearRatioAz;
  float elSteps = elevation * stepsPerDegree * gearRatioEl;

  stepperAz.moveTo(azSteps);
  stepperEl.moveTo(elSteps);

  Serial.print("Target: ");
  Serial.print(lat, 6); Serial.print(", ");
  Serial.print(lon, 6); Serial.print(" | Alt: ");
  Serial.println(alt);

  Serial.print("Azimuth: ");
  Serial.print(azimuth); Serial.print(" | Elevation: ");
  Serial.println(elevation);

  Serial.print("Steps AZ: ");
  Serial.print(azSteps); Serial.print(" | Steps EL: ");
  Serial.println(elSteps);
}

void calculateAzEl(float lat1, float lon1, float alt1,
                   float lat2, float lon2, float alt2,
                   float &azimuth, float &elevation) {
  const float R = 6371000;
  float lat1Rad = radians(lat1);
  float lat2Rad = radians(lat2);
  float deltaLonRad = radians(lon2 - lon1);

  float y = sin(deltaLonRad) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(deltaLonRad);
  azimuth = atan2(y, x) * 180 / PI;
  if (azimuth < 0) azimuth += 360;

  float deltaLatRad = radians(lat2 - lat1);
  float a = sin(deltaLatRad / 2) * sin(deltaLatRad / 2) +
            cos(lat1Rad) * cos(lat2Rad) *
            sin(deltaLonRad / 2) * sin(deltaLonRad / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = R * c;

  float deltaAlt = alt2 - alt1;
  elevation = atan2(deltaAlt, d) * 180 / PI;
  if (elevation < 0) elevation = 0;
}
