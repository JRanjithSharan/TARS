#define dirPin 2
#define stepPin 3
#define potPin A0  

const int stepsPerRevolution = 800; // Adjust for your stepper motor
int lastStepCount = 0;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
  Serial.begin(9600);
}

void stepMotor(int steps, int direction) {
  digitalWrite(dirPin, direction);
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1200); // Increased delay for smoother movement
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1200);
  }
}

void loop() {
  int potValue = analogRead(potPin);
  int targetAngle = map(potValue, 0, 1023, 0, 360);
  int targetSteps = map(targetAngle, 0, 360, 0, stepsPerRevolution);
  
  int stepDifference = targetSteps - lastStepCount;

  if (stepDifference != 0) {
    stepMotor(abs(stepDifference), (stepDifference > 0) ? HIGH : LOW);
    lastStepCount = targetSteps;
  }

  delay(50); // Stabilizes readings
}
