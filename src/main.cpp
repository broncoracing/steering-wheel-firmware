#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <FlexCAN.h>

// Button Inputs
int downShiftPin = 14; // Left paddle (on rear)
int upShiftPin = 15;   // Right paddle (on rear)
int launchPin = 16;    // Right front button
int settingPin = 17;   // Left front button

// CAN Status LED
int led = 13;

// NEOPIXELS
int pixelPin = 10;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, pixelPin, NEO_GRB + NEO_KHZ800);
int wakeUp = 1500;
int shiftRpm = 9000;
int redline = 11250;
int brightness = 255; // 0 to 255
int delayVal = 35;    // set wakeup sequence speed
long lastEcuMillis = 0;
bool ecuOn = false;
bool EngRunning = false;

elapsedMillis launchMessageTimer;

// CAN Frame Data
int rpm = 0;

// Necessary CAN frames
CAN_message_t inMsg;
CAN_message_t downShiftMsg;
CAN_message_t upShiftMsg;
CAN_message_t halfShiftMsg;
CAN_message_t launchMsg;

// Function Prototypes
void upShift();
void downShift();
void halfShift();

void setLights(int rpm);

void downTrig();
void upTrig();

void settingTrig();
void changeSetting();

bool checkPin(int pin, int activeState);

volatile bool shouldUpShift = false;
volatile bool shouldDownShift = false;
volatile bool shouldChangeSetting = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Online");

  // Pull up all input pins
  pinMode(downShiftPin, INPUT_PULLUP);
  pinMode(upShiftPin, INPUT_PULLUP);
  pinMode(settingPin, INPUT_PULLUP);
  pinMode(launchPin, INPUT_PULLUP);

  // onboard LED setup
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(downShiftPin), downTrig, FALLING);
  attachInterrupt(digitalPinToInterrupt(upShiftPin), upTrig, FALLING);
  attachInterrupt(digitalPinToInterrupt(settingPin), settingTrig, FALLING);

  // BR CAN speed
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
  halfShiftMsg.ext = true;
  launchMsg.ext = true;

  upShiftMsg.id = 0;
  downShiftMsg.id = 0;
  halfShiftMsg.id = 0;
  launchMsg.id = 1;

  upShiftMsg.len = 8;
  downShiftMsg.len = 8;
  halfShiftMsg.len = 8;
  launchMsg.len = 8;

  upShiftMsg.buf[0] = 10;   // 0x0A
  downShiftMsg.buf[0] = 11; // 0x0B
  halfShiftMsg.buf[0] = 14; // 0x0E
  launchMsg.buf[0] = 1;

  //----- NEOPIXEL SETUP -----
  strip.begin();
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < 16; i++) {
      strip.setPixelColor(i, 255, 255, 255);
      strip.setPixelColor(15 - i, 255, 255, 255);
      strip.show();
      delay(25);
      strip.setPixelColor(i, 0, 0, 0);
      strip.setPixelColor(15 - i, 0, 0, 0);
      strip.show();
    }
  }

  strip.setPixelColor(14, 255, 255, 255);
  strip.setPixelColor(1, 255, 255, 255);
  strip.show();
}

void loop() {

  if (launchMessageTimer > 100) {
    // Serial.println(digitalRead(launchPin));
    launchMsg.buf[0] = digitalRead(launchPin);
    if (Can0.write(launchMsg)) {
      Serial.println(launchMsg.buf[0]);
    }
    launchMessageTimer = 0;
  }

  if (shouldUpShift == true) {
    if (checkPin(upShiftPin, 0)) {
      upShift();
      delay(150);
    }
    shouldUpShift = false;
  }

  if (shouldDownShift == true) {
    if (checkPin(downShiftPin, 0)) {
      if (rpm < 10000) { // Anti Money Shift
        downShift();
      }
      delay(150);
    }
    shouldDownShift = false;
  }

  if (shouldChangeSetting == true) {
    // if (checkPin(settingPin, 0)) {
    //   changeSetting();
    //   delay(150);
    // }

    bool didBreak = false;

    if (checkPin(settingPin, 0)) {
      for (int i = 0; i < 20; i++) {
        if (checkPin(settingPin, 0) == false) {
          changeSetting();
          didBreak = true;
          break;
        }
      }

      if (didBreak == false) {
        halfShift();
      }
    }
    shouldChangeSetting = false;
  }

  if (Can0.available()) {
    digitalWrite(led, !digitalRead(led));
    Can0.read(inMsg);

    if (inMsg.id == 218099784) { // This frame carries RPM and TPS
      lastEcuMillis = millis();  // start a timer for the next frame
      ecuOn = true;

      int lowByte = inMsg.buf[0];
      int highByte = inMsg.buf[1];
      rpm = ((highByte * 256) + lowByte);
      if (rpm > 500) {
        EngRunning = true;
      }
      if (rpm > 6000) {
        setLights(rpm);
      }
    }

    if (inMsg.id == 6) {
      if (rpm < 6000) { // Show info on tach
        strip.clear();

        if (ecuOn == true) {
          strip.setPixelColor(1, 0, 255, 0);
          strip.setPixelColor(14, 0, 255, 0);
        } else {
          strip.setPixelColor(1, 255, 255, 255);
          strip.setPixelColor(14, 255, 255, 255);
        }

        if (inMsg.buf[0] == 0) { // Neutral
          for (int i = 6; i < 10; i++) {
            strip.setPixelColor(i, 0, 255, 0);
          }
        } else if (inMsg.buf[0] == 1) { // In gear
          for (int i = 6; i < 10; i++) {
            strip.setPixelColor(i, 255, 255, 255);
          }
        }
        strip.show();
      }
    }
  }

  if ((millis() - lastEcuMillis) > 500) {
    ecuOn = false;
    EngRunning = false;
  }

  if (ecuOn == false) {
    rpm = 0;
    strip.clear();
    strip.setPixelColor(14, 255, 255, 255);
    strip.setPixelColor(1, 255, 255, 255);
    strip.show();
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

void halfShift() {
  Serial.println("HalfShift");
  if (Can0.write(halfShiftMsg)) {
    Serial.println("HalfShift successful");
  }
}

//----- NEOPIXEL FUNCTIONS -----

void setLights(int rpm) {

  if (rpm == 0) {
    strip.clear();
    strip.setPixelColor(0, 0, 255, 0);
    strip.setPixelColor(15, 0, 255, 0);
  }

  if (rpm < shiftRpm) { // ----- NORMAL REVS -----

    strip.clear();

    int numLEDs = strip.numPixels();
    float rpmPerLED =
        ((redline - wakeUp) / numLEDs); // calculates how many rpm per led
    int ledsToLight = ceil(rpm / rpmPerLED);

    // FLIPPED STRIP
    for (int i = 15; i >= 15 - ledsToLight; i--) {

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

    // FLIPPED STRIP
    for (int i = 15; i >= 15 - ledsToLight; i--) {

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
    strip.setBrightness(25); // Night mode
    strip.show();
  } else {
    strip.setBrightness(255);
    strip.show();
  }
}

// Pass pin and wanted/active state as params
// True if clean press, false if not
bool checkPin(int pin, int activeState) {
  delay(5); // Allow pin to settle
  bool success = true;
  for (int i = 0; i < 20; i++) {
    if (digitalRead(pin) != activeState) {
      success = false;
    }
    delay(1);
  }
  return success;
}