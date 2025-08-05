#include <SPI.h>
#include <SD.h>

// SPI pin definitions
#define SD_CS    2   // Chip Select pin for SD card
#define SD_MOSI  13
#define SD_MISO   4
#define SD_SCK   14

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing SD card...");

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);  // Start SPI with custom pins

  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("❌ SD card initialization failed!");
    return;
  }
  Serial.println("✅ SD card initialized.");

  // Create or open test.txt
  File file = SD.open("/test.txt", FILE_WRITE);
  if (file) {
    file.println("This is a test from ESP32!");
    file.close();
    Serial.println("✅ Data written to test.txt");
  } else {
    Serial.println("❌ Failed to open test.txt");
  }

  // Read the file back
  file = SD.open("/test.txt");
  if (file) {
    Serial.println("📄 Reading from test.txt:");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
  } else {
    Serial.println("❌ Failed to open test.txt for reading");
  }
}

void loop() {
  // Nothing to do here
}
