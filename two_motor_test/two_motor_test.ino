#define dirPin1 2
#define stepPin1 3

#define dirPin2 4
#define stepPin2 5

void setup() {
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  digitalWrite(dirPin1, HIGH); // Motor 1 direction
  digitalWrite(dirPin2, HIGH); // Motor 2 direction
}

void loop() {
  digitalWrite(stepPin1, HIGH);
  digitalWrite(stepPin2, HIGH);
  delayMicroseconds(500);

  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(500);
}
