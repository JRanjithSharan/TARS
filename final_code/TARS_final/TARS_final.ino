#include <SPI.h>
#include <LoRa.h>

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

// LoRa module connections (VSPI)
#define LORA_CS   5
#define LORA_RST  27
#define LORA_DIO0 26

#define LORA_SCK  14
#define LORA_MISO 12
#define LORA_MOSI 13

SPIClass spiLoRa(VSPI);  // Using VSPI for LoRa

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
  while (!Serial);

  Serial.println("========================================");
  Serial.println("TARS - Tracking Antenna Reception System");
  Serial.println("Using LoRa GPS telemetry input");
  Serial.println("Step 1: Azimuth rotates first (with elevation compensation)");
  Serial.println("Step 2: Elevation rotates independently");
  Serial.println("========================================");

  // Initialize LoRa with VSPI pins
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(spiLoRa);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(915E6)) {
    Serial.println("❌ LoRa init failed. Check wiring.");
    while (true);
  }
  Serial.println("✅ LoRa Receiver @ 915 MHz started!");
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    Serial.print("📡 Received: ");
    Serial.println(incoming);
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());
    Serial.print("SNR: ");
    Serial.println(LoRa.packetSnr());
    Serial.println("----------------------");

    // Parse GPS coords from incoming telemetry string
    // Assuming format like:
    // "188,22,0.43,30.897859,-102.907296,950.00,11,3:43:5,..."
    // So lat at index 3, lon at index 4, alt at index 5

    int indices[10];  // to store comma positions
    int idxCount = 0;
    for (int i = 0; i < incoming.length() && idxCount < 10; i++) {
      if (incoming.charAt(i) == ',') {
        indices[idxCount++] = i;
      }
    }

    if (idxCount >= 5) {
      String latStr = incoming.substring(indices[2] + 1, indices[3]);
      String lonStr = incoming.substring(indices[3] + 1, indices[4]);
      String altStr = incoming.substring(indices[4] + 1, indices[5]);

      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
      float alt = altStr.toFloat();

      float azimuth, elevation;
      calculateAzEl(groundLat, groundLon, groundAlt, lat, lon, alt, azimuth, elevation);

      Serial.println("------ Target Calculated ------");
      Serial.print("Target Azimuth: "); Serial.print(azimuth); Serial.println("°");
      Serial.print("Target Elevation: "); Serial.print(elevation); Serial.println("°");

      moveToAzimuthWithElevationComp(azimuth);  // Step 1: horizontal rotation (with compensation)
      delay(500);                               // small buffer
      moveToElevation(elevation);               // Step 2: independent elevation adjustment
    } else {
      Serial.println("❌ Received telemetry format unexpected, can't parse GPS");
    }
  }
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
