#include <SPI.h>
#include <LoRa.h>
#include <FS.h>
#include <SPIFFS.h>

// Stepper configuration
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

float elevationFactor = 1.0;

const float groundLat = 30.89841339171403;
const float groundLon = -102.90897836824136;
const float groundAlt = 923;

float currentAz = 0.0;
float currentEl = 0.0;

// LoRa VSPI
#define LORA_CS   5
#define LORA_RST  27
#define LORA_DIO0 26
#define LORA_SCK  14
#define LORA_MISO 12
#define LORA_MOSI 13

SPIClass spiLoRa(VSPI);

void setup() {
  pinMode(ELEV_DIR, OUTPUT); pinMode(ELEV_PUL, OUTPUT); pinMode(ELEV_EN, OUTPUT);
  pinMode(AZIM_DIR, OUTPUT); pinMode(AZIM_PUL, OUTPUT); pinMode(AZIM_EN, OUTPUT);
  digitalWrite(ELEV_EN, LOW); digitalWrite(AZIM_EN, LOW);

  Serial.begin(115200);
  while (!Serial);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS Mount Failed");
    return;
  }

  // Create CSV header if not exists
  if (!SPIFFS.exists("/TARS_data.csv")) {
    File f = SPIFFS.open("/TARS_data.csv", FILE_WRITE);
    if (f) {
      f.println("Raw_Telemetry_String");
      f.close();
    }
  }

  // LoRa Init
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(spiLoRa);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(915E6)) {
    Serial.println("❌ LoRa init failed.");
    while (true);
  }
  Serial.println("✅ TARS initialized. Awaiting telemetry...");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String incoming = "";
    while (LoRa.available()) incoming += (char)LoRa.read();

    Serial.println("📡 Received: " + incoming);

    // === Store raw telemetry string to SPIFFS CSV ===
    File logFile = SPIFFS.open("/TARS_data.csv", FILE_APPEND);
    if (logFile) {
      logFile.println(incoming);
      logFile.close();
      Serial.println("✅ Logged to /TARS_data.csv");
    } else {
      Serial.println("❌ Failed to write to log file");
    }

    // === Parse GPS and track ===
    int indices[10], idxCount = 0;
    for (int i = 0; i < incoming.length() && idxCount < 10; i++) {
      if (incoming.charAt(i) == ',') indices[idxCount++] = i;
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

      Serial.print("Target Azimuth: "); Serial.println(azimuth);
      Serial.print("Target Elevation: "); Serial.println(elevation);

      moveToAzimuthWithElevationComp(azimuth);
      delay(500);
      moveToElevation(elevation);
    } else {
      Serial.println("⚠️ Could not parse telemetry packet");
    }
  }
}

void calculateAzEl(float lat1, float lon1, float alt1, float lat2, float lon2, float alt2, float &az, float &el) {
  const float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float lat1Rad = radians(lat1);
  float lat2Rad = radians(lat2);

  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1Rad) * cos(lat2Rad) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float horizontalDistance = R * c;
  float deltaAlt = alt2 - alt1;

  el = atan2(deltaAlt, horizontalDistance) * 180.0 / PI;
  if (el < 0) el = 0;

  float y = sin(dLon) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
  az = atan2(y, x) * 180.0 / PI;
  if (az < 0) az += 360;
}

void moveToAzimuthWithElevationComp(float targetAz) {
  float deltaAz = targetAz - currentAz;
  if (deltaAz < -180) deltaAz += 360;
  if (deltaAz > 180) deltaAz -= 360;

  int azSteps = abs(deltaAz) * stepsPerDegree;
  int elSteps = azSteps * elevationFactor;
  bool azDir = (deltaAz >= 0) ? false : true;

  Serial.print("Rotating AZ: "); Serial.print(deltaAz);
  Serial.print("°, EL Compensation Steps: "); Serial.println(elSteps);

  stepMotorDual(AZIM_DIR, AZIM_PUL, ELEV_DIR, ELEV_PUL, azDir, azSteps, elSteps);
  currentAz = targetAz;
}

void moveToElevation(float targetEl) {
  float deltaEl = targetEl - currentEl;
  int elSteps = abs(deltaEl) * stepsPerDegree;
  bool elDir = (deltaEl >= 0) ? true : false;

  Serial.print("Adjusting EL by "); Serial.println(deltaEl);
  stepMotor(ELEV_DIR, ELEV_PUL, elDir, elSteps);
  currentEl = targetEl;
}

void stepMotor(int dirPin, int pulPin, bool direction, int steps) {
  digitalWrite(dirPin, direction);
  for (int i = 0; i < steps; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
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
