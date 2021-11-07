#include <Arduino.h>
#include <U8x8lib.h>
#include "qdec.h"

#include <AceButton.h>
#include <AccelStepper.h>

using namespace ::SimpleHacks;
using namespace ace_button;

const int BUTTON_PIN = A1;
AceButton button(BUTTON_PIN);

const int ROTARY_PIN_A = A2; // the first pin connected to the rotary encoder
const int ROTARY_PIN_B = A3; // the second pin connected to the rotary encoder
QDecoder qdec;

U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8;
const uint8_t * DEFAULT_FONT = u8x8_font_torussansbold8_r;
const uint8_t ICON_PAUSE = 68;
const uint8_t ICON_PLAY = 69;

const int stepsPerRevolution = 2048;
#define FULLSTEP 4
// in1, in3, in2, in4
AccelStepper myStepper(FULLSTEP, 7, 5, 6, 4);

#define MODE_MINUTES_PRINT 1
#define MODE_MINUTES_USERINPUT 2
#define MODE_SPEED_PRINT 3
#define MODE_SPEED_USERINPUT 4
#define MODE_RUN_PREPARE 5
#define MODE_RUN 6
#define MODE_RUN_PAUSED 7
#define MODE_FINISH 8
#define MODE_FINISH_IDLE 9

#define STEPPER_MAXSPEED 500
#define STEPPER_SPEEDSTEPS 10

int mode = MODE_MINUTES_PRINT;
int durationInMin = 15;
unsigned long durationLeftoverMillis = 0;
int stepperSpeedPercentage = 20;
unsigned long prevTime = 0;

void initDisplay() {
  u8x8.begin();
  u8x8.setFont(DEFAULT_FONT);
}

void resetAll() {
  mode = MODE_MINUTES_PRINT;
  u8x8.setPowerSave(false);
}

void printStateIcon(uint8_t icon) {
  u8x8.setFont(u8x8_font_open_iconic_play_2x2);
  u8x8.drawGlyph(0, 0, icon);
  u8x8.setFont(DEFAULT_FONT);
}

void handleClick() {
  switch (mode) {
    case MODE_MINUTES_USERINPUT:
      mode = MODE_SPEED_PRINT;
      break;
    case MODE_SPEED_USERINPUT:
      mode = MODE_RUN_PREPARE;
      break;
    case MODE_RUN:
      mode = MODE_RUN_PAUSED;
      printStateIcon(ICON_PAUSE);
      break;
    case MODE_RUN_PAUSED:
      printStateIcon(ICON_PLAY);
      prevTime = millis();
      mode = MODE_RUN;
      break;
    case MODE_FINISH_IDLE:
      resetAll();
      break;
  }
}

void handleEvent(AceButton* /*button*/, uint8_t eventType, uint8_t /*buttonState*/) {
  switch (eventType) {
    case AceButton::kEventClicked:    
      handleClick();
      break;
    case AceButton::kEventLongPressed:
      resetAll();
      break;
  }
}

void initRotaryEncoder() {
  qdec.setPinA(ROTARY_PIN_A);
  qdec.setPinB(ROTARY_PIN_B);
  qdec.setFullStep(true);
  qdec.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.setEventHandler(handleEvent);
  ButtonConfig* config = button.getButtonConfig();
  config->setFeature(ButtonConfig::kFeatureClick);
  config->setFeature(ButtonConfig::kFeatureLongPress);
  config->setClickDelay(800);
}

void initStepper() {
  myStepper.setMaxSpeed(STEPPER_MAXSPEED);
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println("setup start");
  initDisplay();
  initRotaryEncoder();
  initStepper();
}

void printMinutes(bool clearAll) {
  if(clearAll) {
    u8x8.clear();
    u8x8.drawString(0, 0, "Duration");
    u8x8.draw1x2String(3, 2, "minutes");
  }

  char minStr[10];
  sprintf(minStr, "%02d", durationInMin);
  u8x8.draw1x2String(0, 2, minStr);
  mode = MODE_MINUTES_USERINPUT;
}

void printSpeed(bool clearAll) {
  if(clearAll) {
    u8x8.clear();
    u8x8.drawString(0, 0, "Speed");
    u8x8.draw1x2String(4, 2, "%");
  }

  char percStr[10];
  sprintf(percStr, "%3d", stepperSpeedPercentage);
  u8x8.draw1x2String(0, 2, percStr);
  mode = MODE_SPEED_USERINPUT;
}

int getEncoderMovement() {
  QDECODER_EVENT event = qdec.update();

  if (event & QDECODER_EVENT_CW) {
    return 1;
  } else if (event & QDECODER_EVENT_CCW) {
    return -1;
  }

  return 0;
}

int keepInBoundaries(int value, int min, int max) {
  if(value < min) {
    return max;
  } else if(value > max) {
    return min;
  }
  return value;
}

void adjustStepper() {
  int mappedSpeed = map(stepperSpeedPercentage, 0, 100, 0, STEPPER_MAXSPEED);
	myStepper.setSpeed(mappedSpeed);
}

void lightLeds(bool on) {
  if(on) {
    Serial.println("TODO LEDS on");
  } else {
    Serial.println("TODO LEDS off");
  }
}

void prepareRun() {
  durationLeftoverMillis = durationInMin * 60;
  durationLeftoverMillis *= 1000;
  prevTime = millis();
  lightLeds(true);
}

void printRun(bool clearAll) {
  if(clearAll) {
    u8x8.clear();
    u8x8.drawString(2, 0, "Time: ");
    u8x8.drawString(2, 2, "Speed: ");
    printStateIcon(ICON_PLAY);
  }

  uint32_t seconds = durationLeftoverMillis / 1000;
  uint16_t s = seconds % 60;
  seconds = (seconds - s) / 60;
  uint16_t m = seconds % 60;

  char timeStr[10];
  sprintf(timeStr, "%02d:%02d", m, s);
  u8x8.drawString(9, 0, timeStr);

  char percStr[10];
  sprintf(percStr, "%3d %%", stepperSpeedPercentage);
  u8x8.drawString(9, 2, percStr);
}

void cureRun(bool forceUpdate) {
  unsigned long passedMillis = millis() - prevTime;
  if(forceUpdate || passedMillis >= 1000) {
    prevTime = millis();
    if(passedMillis < durationLeftoverMillis) {
      durationLeftoverMillis -= passedMillis;
      printRun(false);
    } else {
      mode = MODE_FINISH;
    }
  }
  
  myStepper.runSpeed();
}

void finishRun() {
  u8x8.setPowerSave(true);
  lightLeds(false);
}

void loop(void)
{
  int movement = getEncoderMovement();
  switch(mode) {
    case MODE_MINUTES_PRINT:
      printMinutes(true);
      break;
    case MODE_MINUTES_USERINPUT:
      if(movement != 0) {
        durationInMin += movement;
        durationInMin = keepInBoundaries(durationInMin, 1, 90);
        printMinutes(false);
      }
      break;
    case MODE_SPEED_PRINT:
      printSpeed(true);
      adjustStepper();
      break;
    case MODE_SPEED_USERINPUT:
      if(movement != 0) {
        stepperSpeedPercentage += movement * STEPPER_SPEEDSTEPS;
        stepperSpeedPercentage = keepInBoundaries(stepperSpeedPercentage, 0, 100);
        printSpeed(false);
        adjustStepper();
      }
      myStepper.runSpeed();
      break;
    case MODE_RUN_PREPARE:
      prepareRun();
      printRun(true);
      mode = MODE_RUN;
      break;
    case MODE_RUN:
      if(movement != 0) {
        stepperSpeedPercentage += movement * STEPPER_SPEEDSTEPS;
        stepperSpeedPercentage = keepInBoundaries(stepperSpeedPercentage, 0, 100);
        adjustStepper();
        cureRun(true);
      } else {
        cureRun(false);
      }
      
      break;
    case MODE_FINISH:
      finishRun();
      mode = MODE_FINISH_IDLE;
      break;
    case MODE_FINISH_IDLE:
      // controlled by click
      break;
  }

  
  button.check();
}