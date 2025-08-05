#include <SPI.h>
#include <LoRa.h>
#include <SPIFFS.h>

// ----- Stepper Motor Configuration -----
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

#define STEP_DELAY_US 800        // Default step delay for azimuth and dual steps
#define STEP_DELAY_EL_US 300     // Faster step delay for elevation-only movement

float elevationFactor = 1.0;

// Ground station coordinates
const float groundLat = 31.0367680;
const float groundLon = -103.5377536;
const float groundAlt = 900;

float currentAz = 0.0;
float currentEl = 0.0;

// ----- LoRa SPI & Module Pins -----
#define LORA_CS   5
#define LORA_RST  27
#define LORA_DIO0 26
#define LORA_SCK  14
#define LORA_MISO 12
#define LORA_MOSI 13

#define LOG_FILE "/TARS_data.csv"

SPIClass spiLoRa(VSPI);

// ---- Simulation Variables ----
bool simActive = false;
float simH = 0;
float targetAlt = 3048.0;      // 10,000 ft in meters
float simDistance = 1200.0;    // 1.2 km
float climbRate = 343.0;       // m/s
unsigned long lastSimTime = 0;
const int timeInterval = 100;  // in milliseconds

// ---- Telemetry Timeout ----
unsigned long lastLoRaTime = 0;
const unsigned long telemetryTimeout = 5000;
bool telemetryAvailable = false;

// ---- Telemetry Waiting Print Control ----
bool telemetryEverReceived = false;
unsigned long lastTelemetryCheckTime = 0;

// ----- Function Prototypes -----
void stepMotor(int dirPin, int pulPin, bool direction, int steps, int stepDelayUs = STEP_DELAY_US);
void stepMotorDual(int dirA, int pulA, int dirB, int pulB, bool direction, int azSteps, int elSteps);
void moveToElevation(float targetEl);
void moveToAzimuthWithElevationComp(float targetAz);
void calculateAzEl(float lat1, float lon1, float alt1, float lat2, float lon2, float alt2, float &azimuth, float &elevation);

void setup() {
  pinMode(ELEV_DIR, OUTPUT);
  pinMode(ELEV_PUL, OUTPUT);
  pinMode(ELEV_EN, OUTPUT);
  pinMode(AZIM_DIR, OUTPUT);
  pinMode(AZIM_PUL, OUTPUT);
  pinMode(AZIM_EN, OUTPUT);

  digitalWrite(ELEV_EN, LOW);
  digitalWrite(AZIM_EN, LOW);

  Serial.begin(115200);
  while (!Serial);

  Serial.println("========================================");
  Serial.println("TARS - Tracking Antenna Reception System");
  Serial.println("Using LoRa GPS telemetry input");
  Serial.println("========================================");

  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS Mount Failed");
  } else {
    Serial.println("✅ SPIFFS mounted successfully.");
  }

  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(spiLoRa);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(915E6)) {
    Serial.println("❌ LoRa init failed.");
    while (true);
  }
  Serial.println("✅ LoRa Receiver @ 915 MHz started!");
}

void loop() {
  // Command input
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "READ_LOG") {
      File file = SPIFFS.open(LOG_FILE, FILE_READ);
      if (file) {
        Serial.println("=== SPIFFS Log Dump Start ===");
        while (file.available()) Serial.write(file.read());
        Serial.println("\n=== SPIFFS Log Dump End ===");
        file.close();
      } else {
        Serial.println("❌ Cannot open log file");
      }
    } else if (cmd == "2005") {
      simActive = true;
      simH = 0;
      lastSimTime = millis();
      Serial.println("🚀 Elevation-only rocket tracking started...");
    }
  }

  // Periodic message if no telemetry received yet
  if (!telemetryEverReceived && millis() - lastTelemetryCheckTime >= 2000) {
    Serial.println("📡 Waiting for telemetry...");
    lastTelemetryCheckTime = millis();
  }

  // Simulation elevation motion (non-blocking)
  if (simActive && millis() - lastSimTime >= timeInterval) {
    lastSimTime = millis();
    if (simH <= targetAlt) {
      float el = atan2(simH, simDistance) * 180.0 / PI;
      moveToElevation(el);
      simH += climbRate * (timeInterval / 1000.0);
    } else {
      simActive = false;
      Serial.println("✅ Rocket reached max altitude");
    }
  }

  // LoRa packet handling
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String incoming = "";
    while (LoRa.available()) incoming += (char)LoRa.read();

    lastLoRaTime = millis();
    telemetryAvailable = true;
    telemetryEverReceived = true;

    Serial.println("📡 Received: " + incoming);
    Serial.print("RSSI: "); Serial.println(LoRa.packetRssi());
    Serial.print("SNR: "); Serial.println(LoRa.packetSnr());

    File logFile = SPIFFS.open(LOG_FILE, FILE_APPEND);
    if (logFile) {
      logFile.println(incoming);
      logFile.close();
      Serial.println("✅ Logged to SPIFFS");
    } else {
      Serial.println("❌ Failed to log to SPIFFS");
    }

    // Parse GPS data
    int indices[10], idxCount = 0;
    for (int i = 0; i < incoming.length() && idxCount < 10; i++) {
      if (incoming.charAt(i) == ',') indices[idxCount++] = i;
    }

    if (idxCount >= 5) {
      float lat = incoming.substring(indices[2] + 1, indices[3]).toFloat();
      float lon = incoming.substring(indices[3] + 1, indices[4]).toFloat();
      float alt = incoming.substring(indices[4] + 1, indices[5]).toFloat();

      float azimuth, elevation;
      calculateAzEl(groundLat, groundLon, groundAlt, lat, lon, alt, azimuth, elevation);

      Serial.println("------ 🛰️ Telemetry Info ------");
      Serial.print("Latitude: "); Serial.println(lat, 6);
      Serial.print("Longitude: "); Serial.println(lon, 6);
      Serial.print("Altitude: "); Serial.print(alt); Serial.println(" m");
      Serial.print("Azimuth: "); Serial.print(azimuth); Serial.println(" °");
      Serial.print("Elevation: "); Serial.print(elevation); Serial.println(" °");
      Serial.println("--------------------------------");

      moveToAzimuthWithElevationComp(azimuth);
      delay(500);
      moveToElevation(elevation);
    } else {
      Serial.println("❌ Telemetry format unexpected");
    }
  }

  // Check telemetry timeout
  if (millis() - lastLoRaTime > telemetryTimeout && telemetryAvailable) {
    Serial.println("⚠️ No telemetry received for 5 seconds.");
    telemetryAvailable = false;
  }
}

// ---- Azimuth and Elevation Calculation ----
void calculateAzEl(float lat1, float lon1, float alt1, float lat2, float lon2, float alt2, float &azimuth, float &elevation) {
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

// ---- Antenna Movement Functions ----
void moveToAzimuthWithElevationComp(float targetAz) {
  float deltaAz = targetAz - currentAz;
  if (deltaAz < -180) deltaAz += 360;
  if (deltaAz > 180) deltaAz -= 360;

  int azSteps = abs(deltaAz) * stepsPerDegree;
  int elSteps = azSteps * elevationFactor;
  bool azDir = (deltaAz >= 0) ? false : true;

  stepMotorDual(AZIM_DIR, AZIM_PUL, ELEV_DIR, ELEV_PUL, azDir, azSteps, elSteps);
  currentAz = targetAz;
}

void moveToElevation(float targetEl) {
  float deltaEl = targetEl - currentEl;
  int elSteps = abs(deltaEl) * stepsPerDegree;
  bool elDir = (deltaEl >= 0);
  stepMotor(ELEV_DIR, ELEV_PUL, elDir, elSteps, STEP_DELAY_EL_US);  // Faster elevation step delay
  currentEl = targetEl;
}

void stepMotor(int dirPin, int pulPin, bool direction, int steps, int stepDelayUs) {
  digitalWrite(dirPin, direction);
  for (int i = 0; i < steps; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(stepDelayUs);
  }
}

void stepMotorDual(int dirA, int pulA, int dirB, int pulB, bool direction, int azSteps, int elSteps) {
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
