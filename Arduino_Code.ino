#include <avr/io.h>
#include <util/delay.h>
// Define pin assignments
#define WATER_SENSOR_PIN A0
#define VENT_DIRECTION_PIN 8
#define FAN_CONTROL_PIN 9
#define ON_OFF_BUTTON_PIN 10
#define START_BUTTON_PIN 2
#define STOP_BUTTON_PIN 3
#define YELLOW_LED_PIN 4
#define GREEN_LED_PIN 5
#define RED_LED_PIN 6
#define BLUE_LED_PIN 7
// Define threshold values
#define WATER_LEVEL_THRESHOLD 500
#define TEMP_HIGH_THRESHOLD 30
#define TEMP_LOW_THRESHOLD 20
// Function prototypes
void initialize();
void checkWaterLevel();
void readTemperatureHumidity();
void controlFan();
void controlVent();
void reportEvent(boolean motorStatus);
void disableSystem();
void enableSystem();
void handleStartButtonInterrupt();
// Global variables
boolean systemEnabled = true;
boolean fanOn = false;

enum State {
  DISABLED,
  IDLE,
  ERROR,
  RUNNING
};
State currentState = IDLE;

void setup() {
  initialize();
}
void loop() {
  switch (currentState) {
    case DISABLED:
      disableSystem();
      break;
    case IDLE:
      checkWaterLevel();
      readTemperatureHumidity();
      controlFan();
      controlVent();
      break;
    case ERROR:
// Handle error state
      break;
    case RUNNING:
      checkWaterLevel();
      readTemperatureHumidity();
      controlFan();
      controlVent();
      break;
  }
}
void initialize() {
// Set pin modes
  DDRD |= (1 << VENT_DIRECTION_PIN); // Set vent direction pin as output
  pinMode(FAN_CONTROL_PIN, OUTPUT);
  pinMode(ON_OFF_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
// Initialize serial communication
  Serial.begin(9600);
// Attach interrupt for start button
  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), handleStartButtonInterrupt, FALLING);
}
void handleError() {
  currentState = ERROR;
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  Serial.println("Error: What the sigma?!");
}

void handleStopButtonPress() {
  currentState = IDLE;
  digitalWrite(FAN_CONTROL_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
}

void checkWaterLevel() {
  int waterLevel = analogRead(WATER_SENSOR_PIN);
  if (currentState != DISABLED && waterLevel < WATER_LEVEL_THRESHOLD) {
    currentState = ERROR;
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    Serial.println("Error: Water level too low!");
  }
}

void readTemperatureHumidity() {
// Code to read temperature and humidity from DHT11 sensor and display on LCD
}

void controlFan() {
  float temperature = 25; // Example temperature, replace with actual reading
  if (currentState != DISABLED && temperature > TEMP_HIGH_THRESHOLD && !fanOn) {
    digitalWrite(FAN_CONTROL_PIN, HIGH); // Turn on fan
    fanOn = true;
    reportEvent(true);
  } else if (temperature < TEMP_LOW_THRESHOLD && fanOn) {
    digitalWrite(FAN_CONTROL_PIN, LOW); // Turn off fan
    fanOn = false;
    reportEvent(false);
  }
}

void controlVent() {
If vent button is pressed, open the vent
  if (digitalRead(VENT_BUTTON_PIN) == HIGH) {
// Open vent
    digitalWrite(VENT_DIR_PIN, HIGH); // Set direction
// Code to step motor to open vent
    ventOpen = true;
  } else {
// Close vent
    digitalWrite(VENT_DIR_PIN, LOW); // Set direction
    ventOpen = false;
  }
}

void reportEvent(boolean motorStatus) {
// Code to get current time and date from RTC module and transmit to host computer over USB
}

void disableSystem() {
  systemEnabled = false;
  digitalWrite(YELLOW_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
}

void enableSystem() {
  systemEnabled = true;
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
}

void handleStartButtonInterrupt() {
  if (!systemEnabled) {
    enableSystem();
    currentState = IDLE;
    Serial.println("System enabled.");
  }
}
