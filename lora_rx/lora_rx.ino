#include <SPI.h>
#include <LoRa.h>

// LoRa module connections
#define LORA_CS   5
#define LORA_RST  27
#define LORA_DIO0 26

// Custom SPI pins (non-default)
#define LORA_SCK  14
#define LORA_MISO 12
#define LORA_MOSI 13

SPIClass spiLoRa(VSPI);  // You can use HSPI too, but VSPI is preferred for LoRa

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize custom SPI with your pins
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(spiLoRa);  // Use this SPI instance
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
  }
}
