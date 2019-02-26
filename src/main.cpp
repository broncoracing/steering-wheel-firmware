#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <FlexCAN.h>

// Button Inputs
// Be sure to debounce with a cap, otherwise interrupt will trigger on button
// release bounces. 2.2uF works.

int downShiftPin = 14; // Left paddle (on rear)
int upShiftPin = 15;   // Right paddle (on rear)
int DRSPin = 16;       // Right front button
int settingPin = 17;   // Left front button

// CAN Status LED
int led = 13;

// NEOPIXELS
int pixelPin = 10;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, pixelPin, NEO_GRB + NEO_KHZ800);

// Neopixel Parameters
int wakeUp = 1500;
int shiftRpm = 9000;
int redline = 11250;
int brightness = 255; // 0 to 255
int delayVal = 35;    // set wakeup sequence speed

// CAN Frame Data
int rpm = 0;

// Necessary CAN frames
CAN_message_t inMsg;
CAN_message_t downShiftMsg;
CAN_message_t upShiftMsg;
CAN_message_t DRSPressedMsg;
CAN_message_t DRSReleasedMsg;

// Function Prototypes
void upShift();
void downShift();
void DRSPressed();
void DRSReleased();
void DRSChanged();
void setLights(int rpm);
void downTrig();
void upTrig();
void settingTrig();
void changeSetting();

volatile bool shouldUpShift = false;
volatile bool shouldDownShift = false;
volatile bool shouldChangeSetting = false;

volatile bool shouldEnableDRS = false;
volatile bool shouldDisableDRS = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Online");

  // Pull up all input pins
  pinMode(downShiftPin, INPUT_PULLUP);
  pinMode(upShiftPin, INPUT_PULLUP);
  pinMode(DRSPin, INPUT_PULLUP);
  pinMode(settingPin, INPUT_PULLUP);

  // input is 3.3v
  // other pin is commonground

  // LED setup
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(downShiftPin), downTrig, FALLING);
  attachInterrupt(digitalPinToInterrupt(upShiftPin), upTrig, FALLING);
  attachInterrupt(digitalPinToInterrupt(DRSPin), DRSChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(settingPin), settingTrig, FALLING);

  Can0.begin(250000);

  // Allow Extended CAN id's through
  CAN_filter_t allPassFilter;
  allPassFilter.ext = 1;
  for (uint8_t filterNum = 1; filterNum < 16; filterNum++) {
    Can0.setFilter(allPassFilter, filterNum);
  }

  //----- FRAME DEFINITIONS -----
  inMsg.ext = true;
  downShiftMsg.ext = true;
  upShiftMsg.ext = true;
  DRSPressedMsg.ext = true;
  DRSReleasedMsg.ext = true;

  upShiftMsg.len = 8;
  downShiftMsg.len = 8;
  DRSPressedMsg.len = 8;
  DRSReleasedMsg.len = 8;

  upShiftMsg.buf[0] = 10;
  downShiftMsg.buf[0] = 11;
  DRSPressedMsg.buf[0] = 12;
  DRSReleasedMsg.buf[0] = 13;

  //----- NEOPIXEL SETUP -----
  strip.begin();
  for (int i = 0; i < 16; i++) {
    strip.setPixelColor(i, 255, 0, 255);
  }
  strip.show();
}

void loop() {
  if (shouldUpShift == true) {
    delay(20);
    if (digitalRead(upShiftPin) == LOW) {
      shouldUpShift = false;
      upShift();
      delay(150);
    }
    shouldUpShift = false;
  }

  if (shouldDownShift == true) {
    delay(20);
    if (digitalRead(downShiftPin) == LOW) {
      shouldDownShift = false;
      downShift();
      delay(150);
    }
    shouldDownShift = false;
  }

  if (shouldChangeSetting == true) {
    delay(20);
    if (digitalRead(settingPin) == LOW) {
      shouldChangeSetting = false;
      changeSetting();
      delay(150);
    }
    shouldChangeSetting = false;
  }

  if (shouldDisableDRS == true) {
    DRSReleased();
    delay(100);
    shouldDisableDRS = false;
  }

  if (shouldEnableDRS == true) {
    DRSPressed();
    delay(100);
    shouldEnableDRS = false;
  }

  if (Can0.available()) {
    digitalWrite(led, !digitalRead(led));
    Can0.read(inMsg);

    if (inMsg.id == 218099784) { // This frame carries RPM and TPS
      int lowByte = inMsg.buf[0];
      int highByte = inMsg.buf[1];
      rpm = ((highByte * 256) + lowByte);
      setLights(rpm);
    }
  }
}

// ----- CAN FRAME SENDING ISR's -----
// REMOVE ALL SERIAL PRINTS ONCE INSTALLED!
void downTrig() { shouldDownShift = true; }
void upTrig() { shouldUpShift = true; }
void settingTrig() { shouldChangeSetting = true; }

void downShift() {
  Serial.println("DownShift");
  if (Can0.write(downShiftMsg)) {
    Serial.println("DownShift successful");
  }
}

void upShift() {
  Serial.println("UpShift");
  if (Can0.write(upShiftMsg)) {
    Serial.println("UpShift successful");
  }
}

void DRSChanged() {
  if (digitalRead(DRSPin) == 1) { // rising
    delay(20);
    if (digitalRead(DRSPin) == 1) { // rising
      shouldDisableDRS = true;
    }
  } else { // falling
    if (digitalRead(DRSPin) == 0) {
      delay(20);
      if (digitalRead(DRSPin) == 0) {
        shouldEnableDRS = true;
      }
    }
  }
}

void DRSPressed() {
  Serial.println("DRS Pressed");
  if (Can0.write(DRSPressedMsg)) {
    Serial.println("DRS Press successful");
  }
}

void DRSReleased() {
  Serial.println("DRS Released");
  if (Can0.write(DRSReleasedMsg)) {
    Serial.println("DRS Release successful");
  }
}

//----- NEOPIXEL FUNCTIONS -----

void setLights(int rpm) {

  if (rpm < shiftRpm) { // ----- NORMAL REVS -----

    strip.clear();

    int numLEDs = strip.numPixels();
    float rpmPerLED =
        ((redline - wakeUp) / numLEDs); // calculates how many rpm per led
    int ledsToLight = ceil(rpm / rpmPerLED);

    for (int i = 0; i <= ledsToLight; i++) {

      strip.setPixelColor(i, 0, 255, 0);
    }

    strip.show();
  }

  if ((rpm > shiftRpm) && (rpm < redline)) { // ----- SHIFT POINT-----
    strip.clear();

    int numLEDs = strip.numPixels();
    float rpmPerLED =
        ((redline - wakeUp) / numLEDs); // calculates how many rpm per led
    int ledsToLight = ceil(rpm / rpmPerLED);

    for (int i = 0; i <= ledsToLight; i++) {

      strip.setPixelColor(i, 255, 255, 0); // yellow
    }

    strip.show();
  }

  if (rpm > redline) { //----- REDLINE -----
    for (unsigned int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, 255, 0, 0);
    }

    strip.show();
    delay(20);
    strip.clear();
    strip.show();
    delay(20);
  }
}

void changeSetting() {
  Serial.println("Brightness Setting Changed");
  if (strip.getBrightness() == 255) {
    strip.setBrightness(30); // Night mode
  } else {
    strip.setBrightness(255);
  }
}