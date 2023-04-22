
#include <ESP32Servo.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>


// WiFi credentials
char ssid[] = "Cherry";
char password[] = "manuelito";

// Blynk authentication token
char auth[] = "04yUGBKe4OcaL6CuBMJvnPPuSLicWleX";

// Virtual Pins
WidgetLED trashBinStatusLED(V1);

// Hardware pins
#define TRIG_PIN1 12 // ultrasonic sensor 1 trigger pin
#define ECHO_PIN1 14 // ultrasonic sensor 1 echo pin
#define TRIG_PIN2 26 // ultrasonic sensor 2 trigger pin
#define ECHO_PIN2 27 // ultrasonic sensor 2 echo pin
#define SERVO_PIN 13 // servo motor pin
#define LED_PIN 25 // LED indicator pin


// Global variables

bool isPersonDetected = false;
const int PERSON_DISTANCE_THRESHOLD = 40; // in centimeters
const int TRASH_FULL_THRESHOLD = 14;

Servo trashLid; // servo object for trash bin lid

void setup() {
  // Setup serial communication
  Serial.begin(9600);

  // Setup ultrasonic sensors pins
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);

  // Setup servo motor
  trashLid.attach(SERVO_PIN);
  trashLid.write(0); // initial position (closed)

  // Setup LED indicator pin
  pinMode(LED_PIN, OUTPUT);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  // Start Blynk
  Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str());
}

void loop() {
  long personDuration, personDistance, trashDuration, trashDistance;

  // measure person distance
  digitalWrite(TRIG_PIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);
  float duration1 = pulseIn(ECHO_PIN1, HIGH);
  float distance1 = duration1 * 0.034 / 2;
  Serial.print("Distance Person: ");
  Serial.print(distance1);
  Serial.println(" cm");
 
  Blynk.virtualWrite(V1, distance1); // send person distance value to virtual pin V1
Blynk.run();
 delay(10);
  // measure trash level
  digitalWrite(TRIG_PIN2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN2, LOW);
  float duration2 = pulseIn(ECHO_PIN2, HIGH);
  float distance2 = duration2 * 0.034 / 2;
  Serial.print("Distance Trash: ");
  Serial.print(distance2);
  Serial.println(" cm");
  int trashLevelPercentage = map(distance2, 100, 14, 0, 100); // map the distance value to a percentage value
  Blynk.virtualWrite(V2, trashLevelPercentage);
  Blynk.run();
  delay(10);


  // Check if a person is detected
  if (distance1 < PERSON_DISTANCE_THRESHOLD) {
    isPersonDetected = true;
  } else {
    isPersonDetected = false;
  }


  // Control the servo based on person detection
  if (!isPersonDetected) {
    trashLid.write(0); // close the lid
    delay(100);
    trashBinStatusLED.off(); // turn off the LED widget
    Blynk.virtualWrite(V4, "Closed");
  } else {
    trashLid.write(90); // keep the lid open
    delay(100);
    trashBinStatusLED.on(); // turn on the LED widget
    Blynk.virtualWrite(V3, "Open");
  }

  // Check if the trash is full and update the LED indicator
  if (distance2 < TRASH_FULL_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH); // turn on the LED
    Blynk.logEvent("Full", "The trash bin is full!");



  } else {
    // Blink the LED on and off
    digitalWrite(LED_PIN, HIGH); // turn on the LED
    delay(500);
    digitalWrite(LED_PIN, LOW); // turn off the LED
    delay(500);

  }
}
