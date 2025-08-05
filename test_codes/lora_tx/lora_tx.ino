  #include <SPI.h>
  #include <LoRa.h>

  #define LORA_CS 5
  #define LORA_RST 32
  #define LORA_DIO0 33

  void setup() {
    Serial.begin(115200);
    while (!Serial);

    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(915E6)) {
      Serial.println("LoRa init failed. Check wiring.");
      while (true);
    }

    Serial.println("LoRa Transmitter @ 915 MHz started!");
  }

  void loop() {
    Serial.println("Sending packet...");
    LoRa.beginPacket();
    LoRa.print("Hello from ESP32 Transmitter @ 915 MHz. Amaterasu");
    LoRa.endPacket();

    delay(2000);
  }
