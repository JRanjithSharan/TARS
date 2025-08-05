#define dirPin 2
#define stepPin 3
#define steps_per_revolution 200  // Change if using microstepping

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  Serial.begin(9600);  // For angle input via Serial Monitor
}

void loop() {
  if (Serial.available() > 0) {
    float angle = Serial.parseFloat();  // Read angle from Serial
    if (angle >= 0 && angle <= 360) {
      rotateStepper(angle);
    }
    // Clear any remaining data in buffer
    while (Serial.available() > 0) Serial.read();
  }
}

void rotateStepper(float angle) {
  int steps = (int)((angle / 360.0) * steps_per_revolution);
  digitalWrite(dirPin, HIGH);  // Set direction (HIGH or LOW)
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);  // Adjust for speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}
