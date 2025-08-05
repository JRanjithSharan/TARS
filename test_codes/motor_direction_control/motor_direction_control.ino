#include <AccelStepper.h>

// Stepper Pins
#define AZ_STEP_PIN D1
#define AZ_DIR_PIN D2
#define EL_STEP_PIN D3
#define EL_DIR_PIN D4

const float stepAngle = 1.8;
const int microstepping = 8;
const int stepsPerRev = (360 / stepAngle) * microstepping;
const float stepsPerDegree = stepsPerRev / 360.0;

AccelStepper stepperAz(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
AccelStepper stepperEl(AccelStepper::DRIVER, EL_STEP_PIN, EL_DIR_PIN);

void setup() {
  Serial.begin(115200);
  stepperAz.setMaxSpeed(1000);
  stepperAz.setAcceleration(500);
  stepperEl.setMaxSpeed(1000);
  stepperEl.setAcceleration(500);

  stepperAz.setCurrentPosition(0); // Assume facing North
  stepperEl.setCurrentPosition(0); // Assume level
  Serial.println("Stepper Manual Test Ready. Type N/E/S/W/U/D/R");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'N') moveAzTo(0);
    else if (c == 'E') moveAzTo(90);
    else if (c == 'S') moveAzTo(180);
    else if (c == 'W') moveAzTo(270);
    else if (c == 'U') moveElTo(45);     // Tilt up 45°
    else if (c == 'D') moveElTo(0);      // Level
    else if (c == 'R') resetMotors();
  }

  stepperAz.run();
  stepperEl.run();
}

void moveAzTo(float deg) {
  long steps = deg * stepsPerDegree;
  stepperAz.moveTo(steps);
  Serial.print("Moving Azimuth to ");
  Serial.print(deg);
  Serial.println(" degrees");
}

void moveElTo(float deg) {
  long steps = deg * stepsPerDegree;
  stepperEl.moveTo(steps);
  Serial.print("Moving Elevation to ");
  Serial.print(deg);
  Serial.println(" degrees");
}

void resetMotors() {
  stepperAz.moveTo(0);
  stepperEl.moveTo(0);
  Serial.println("Resetting to original position (North, Level)");
}
