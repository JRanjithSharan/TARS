#include <SPI.h>
#include <LoRa.h>
#include <math.h>
#include <string.h>

#define PUL_PIN1 4
#define DIR_PIN1 3

#define PUL_PIN2 6
#define DIR_PIN2 5

#define SCK 13
#define MISO 12
#define MOSI 11 
#define SS 5       // Chip Select
#define RST 32     // Reset
#define DIO0 2    // Interrupt


const float stepAngle = 1.8;
const int microstepping = 8;
const int stepsPerRev = (360 / stepAngle) * microstepping;
const float stepsPerDegree = stepsPerRev / 360.0;

long currentPosition1 = 0;
long currentPosition2 = 0;

double lat_base=12.9702572;
double lon_base=79.1563892;

// Alogrithm functions start

double EARTH_RADIUS=6371.0;

double toRadians(double degrees) {
  return degrees * (M_PI / 180.0);
}

double toDegrees(double radians) {
  return radians * (180.0 / M_PI);
}

// Fixed bearing calculation
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double lat1Rad = toRadians(lat1);
  double lon1Rad = toRadians(lon1);
  double lat2Rad = toRadians(lat2);
  double lon2Rad = toRadians(lon2);

  double deltaLon = lon2Rad - lon1Rad;
  
  double y = sin(deltaLon) * cos(lat2Rad);
  double x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(deltaLon);
  
  double bearingRad = atan2(y, x);
  double bearing = toDegrees(bearingRad);
  
  // Normalize to 0-360
  bearing = fmod(bearing + 360.0, 360.0);
  return bearing;
}

double haversineDistance(double lat1, double lon1, double lat2, double lon2) {

  double lat1Rad = toRadians(lat1);
  double lon1Rad = toRadians(lon1);
  double lat2Rad = toRadians(lat2);
  double lon2Rad = toRadians(lon2);

  // Calculate differences
  double dLat = lat2Rad - lat1Rad;
  double dLon = lon2Rad - lon1Rad;

  // Haversine formula
  double a = pow(sin(dLat / 2), 2) +
             cos(lat1Rad) * cos(lat2Rad) * pow(sin(dLon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  // Distance in kilometers
  return EARTH_RADIUS * c;
}

double calculateelevation(double alt, double dist){
  double x = toDegrees(atan2(alt,dist));
  return x;
}

//Alogrithm function end

// Algorithm for motor

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

//Algorithm for motor ends


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");

  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(915E6)) { // Frequency (915 MHz to match the transmitter)
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa initialized successfully");

  pinMode(PUL_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(PUL_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);

  Serial.println("Enter target angles (angle1 angle2):");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Received a packet
    Serial.print("Received packet '");

    // Read full packet
    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }

   
    // Split the received data by commas
    int firstComma = receivedData.indexOf(',');
    int secondComma = receivedData.indexOf(',', firstComma + 1);
    int thirdComma = receivedData.indexOf(',', secondComma + 1);

    // Extract individual values
    String value1Str = receivedData.substring(0, firstComma);
    String value2Str = receivedData.substring(firstComma + 1, secondComma);
    String value3Str = receivedData.substring(secondComma + 1, thirdComma);
    String value4Str = receivedData.substring(thirdComma + 1);

    // Convert to double
    double time = value1Str.toDouble();
    double alt = value2Str.toDouble();
    double lat = value3Str.toDouble();
    double lon = value4Str.toDouble();

    double bearing = calculateBearing(lat_base, lon_base, lat, lon);
    double distance = haversineDistance(lat_base, lon_base, lat, lon);
    double elevation = calculateelevation(alt, distance);

    moveBothMotors(bearing, elevation);

  Serial.print("Bearing: ");
  Serial.print(bearing, 4);
  Serial.print("°");
  Serial.print("  ");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" km");
  Serial.print("  ");
  Serial.print("Altitude");
  Serial.print(alt);
  Serial.print("Elevation");
  Serial.println(elevation);

  delay(2000);


    // Print the values
    //Serial.println("'");
    //Serial.print("Value 1: "); Serial.println(value1);
    //Serial.print("Value 2: "); Serial.println(value2);
    //Serial.print("Value 3: "); Serial.println(value3);
    //Serial.print("Value 4: "); Serial.println(value4);

    // Print RSSI
    //Serial.print("RSSI: ");
    //Serial.println(LoRa.packetRssi());
  }
}

