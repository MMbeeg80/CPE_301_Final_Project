#include <avr/io.h>
#include <util/delay.h>
#include <RTClib.h>
#include <DHT11.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
// Define pin assignments
#define WATER_SENSOR_PIN 0
#define VENT_BUTTON_PIN 4
#define FAN_CONTROL_PIN 7
#define ON_OFF_BUTTON_PIN 10
#define START_BUTTON_PIN 2
#define STOP_BUTTON_PIN 3
#define YELLOW_LED_PIN 48
#define GREEN_LED_PIN 47
#define RED_LED_PIN 45
#define BLUE_LED_PIN 46
// Define threshold values
#define WATER_LEVEL_THRESHOLD 200
#define TEMP_HIGH_THRESHOLD 25
#define TEMP_LOW_THRESHOLD 20
//water Level Sensor
#define RDA 0x80
#define TBE 0x20

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
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
boolean ventOpen = false;
DHT11 dht11(12);
//step motor variables
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 31, 33, 35, 37);
//rtc module
RTC_DS1307 rtc;
//lcd
const int rs = 5, en = 6, d4 = 8, d5 = 9, d6 = 10, d7 = 11;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

enum State {
  DISABLED,
  IDLE,
  ERROR,
  RUNNING
};
State currentState = RUNNING;

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
  lcd.begin(16, 2);
}
void handleError() {
  currentState = ERROR;
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  Serial.println("Error: What the sigma?!");
  lcd.setCursor(0,0);
  lcd.print("LOW WATER");
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
  unsigned int adc_value = adc_read(0);
  if (currentState != DISABLED && adc_value < WATER_LEVEL_THRESHOLD) {
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
  float temperature = dht11.readTemperature();
  float humidity = dht11.readHumidity();
  // Check if any reads failed
  if(isnan(temperature) || isnan(humidity)){
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // Print temperature and humidity to serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C");

  lcd.setCursor(0,1);
  lcd.print("Hum: ");
  lcd.print(humidity);
  lcd.print("%");
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
//Wait until vent control button is pressed
  while (!digitalRead(VENT_BUTTON_PIN)){}
//If vent is closed, open vent
Serial.println(ventOpen);
    if (ventOpen == false){
    myStepper.setSpeed(5);
	  myStepper.step(stepsPerRevolution/5);
    ventOpen = true;
  } else {
// Close vent
    myStepper.setSpeed(5);
	  myStepper.step(-stepsPerRevolution/5);
    ventOpen = false;
  }
  Serial.println(ventOpen);
}

void reportEvent(boolean motorStatus) {
// Code to get current time and date from RTC module and transmit to host computer over USB
  //sample code
  // Check if RTC is available
  if(!rtc.begin()){
    Serial.println("Couldn't find RTC");
    return;
  }

  // Get current time
  DateTime now = rtc.now();
  // Print timestamp and motor status to serial monitor
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(" - Event reported: Motor status is ");
  Serial.println(motorStatus ? "ON" : "OFF");
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

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}
