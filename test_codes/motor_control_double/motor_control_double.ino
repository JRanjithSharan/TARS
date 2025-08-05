#define PUL_PIN1 3
#define DIR_PIN1 2

#define PUL_PIN2 5
#define DIR_PIN2 4

const float stepAngle = 1.8;
const int microstepping = 8;
const int stepsPerRev = (360 / stepAngle) * microstepping;
const float stepsPerDegree = stepsPerRev / 360.0;

long currentPosition1 = 0;
long currentPosition2 = 0;

void setup() {
  pinMode(PUL_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(PUL_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Enter target angles (angle1 angle2):");
}

void loop() {
  if (Serial.available()) {
    float angle1 = Serial.parseFloat();
    float angle2 = Serial.parseFloat();

    Serial.print("Target angles received: ");
    Serial.print(angle1);
    Serial.print(", ");
    Serial.println(angle2);

    moveBothMotors(angle1, angle2);
  }
}

void moveBothMotors(float angle1, float angle2) {
  long targetSteps1 = angle1 * stepsPerDegree;
  long targetSteps2 = angle2 * stepsPerDegree;
  long stepsToMove1 = targetSteps1 - currentPosition1;
  long stepsToMove2 = targetSteps2 - currentPosition2;

  // Determine direction for motor 1
  if (stepsToMove1 >= 0) {
    digitalWrite(DIR_PIN1, HIGH);  // Rotate clockwise
  } else {
    digitalWrite(DIR_PIN1, LOW);   // Rotate counterclockwise
    stepsToMove1 = -stepsToMove1;  // Convert to positive steps
  }

  // Determine direction for motor 2
  if (stepsToMove2 >= 0) {
    digitalWrite(DIR_PIN2, HIGH);  // Rotate clockwise
  } else {
    digitalWrite(DIR_PIN2, LOW);   // Rotate counterclockwise
    stepsToMove2 = -stepsToMove2;  // Convert to positive steps
  }

  stepsToMove1 = abs(stepsToMove1);
  stepsToMove2 = abs(stepsToMove2);

  long maxSteps = max(stepsToMove1, stepsToMove2);

  for (long i = 0; i < maxSteps; i++) {
    if (i < stepsToMove1) digitalWrite(PUL_PIN1, HIGH);
    if (i < stepsToMove2) digitalWrite(PUL_PIN2, HIGH);
    delayMicroseconds(500);
    digitalWrite(PUL_PIN1, LOW);
    digitalWrite(PUL_PIN2, LOW);
    delayMicroseconds(500);
  }

  currentPosition1 = targetSteps1;
  currentPosition2 = targetSteps2;

  Serial.print("Motor 1 moved to angle: ");
  Serial.println(angle1);
  Serial.print("Motor 2 moved to angle: ");
  Serial.println(angle2);
}
