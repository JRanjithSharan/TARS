#define PUL_PIN 3
#define DIR_PIN 2

const float stepAngle = 1.8;           // Motor step angle
const int microstepping = 8;          // Microstepping setting on TB6600
const int stepsPerRev = (360 / stepAngle) * microstepping;  // 3200 steps/rev
const float stepsPerDegree = stepsPerRev / 360.0;

long currentPosition = 0;

void setup() {
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Enter target angle (in degrees):");
}

void loop() {
  if (Serial.available() > 0) {
    float targetAngle = Serial.parseFloat();
    Serial.print("Target angle received: ");
    Serial.println(targetAngle);
    long targetSteps = targetAngle * stepsPerDegree;
    long stepsToMove = targetSteps - currentPosition;

    // Determine direction
    if (stepsToMove > 0) {
      digitalWrite(DIR_PIN, HIGH);
    } else {
      digitalWrite(DIR_PIN, LOW);
      stepsToMove = -stepsToMove;
    }

    // Move motor
    for (long i = 0; i < stepsToMove; i++) {
      digitalWrite(PUL_PIN, HIGH);
      delayMicroseconds(500);
      digitalWrite(PUL_PIN, LOW);
      delayMicroseconds(500);
    }

    currentPosition = targetSteps;

    Serial.print("Moved to angle: ");
    Serial.println(targetAngle);
  }
}