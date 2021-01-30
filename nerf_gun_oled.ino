/**
 * set tools->board->arduino nano
 * set tools->processor->ATmega328P (Old Bootloader)
 * 
 * making the custom font:
 * use openGLCDFontCreator.zip
 * run start.bat
 * file->new font
 * name: cal96
 * start index: 48   (this is 0)
 * char count: 10 (we only want 0 thru 9)
 * import font: calibri
 * size: 96
 * click ok
 * click export
 * save file as cal96.h
 * copy cal96.h to Arduino\libraries\SSD1306Ascii-master\src\fonts
 * edit allFonts.h and add: 
 * #include "cal96.h"
 * code should build and run correctly
 * 
 * these were measured with an ohmmeter: r1 and r2
 */


#include <EEPROM.h>
#include <Wire.h> 
#include <Encoder.h>          // https://github.com/PaulStoffregen/Encoder (installed via library manager)
#include <SPI.h>
#include "SSD1306Ascii.h"     // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiAvrI2c.h"
#include <Adafruit_ADS1015.h> // https://github.com/adafruit/Adafruit_ADS1X15

// address in the EEPROM
// Arduino Uno:        1kb EEPROM storage
#define ADDRESS 0
#define MAGIC 123

struct Settings {
  byte magic;
  byte shotMode;
  byte speedMode;
  byte feedMode;
};

Settings settings;

// default uses 0x48 for the address (tie ADR to GND)
Adafruit_ADS1115 ads;
double gain;
double r1 = 3326;
double r2 = 2190;
int lastPercent = 0;
float percents[] = {
  12.6,
  12.45,
  12.33,
  12.25,
  12.07,
  11.95,
  11.86,
  11.74,
  11.62,
  11.56,
  11.51,
  11.45,
  11.39,
  11.36,
  11.3,
  11.24,
  11.18,
  11.12,
  11.06,
  10.83,
  9.82,
};

  
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiAvrI2c oled;


// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
// on the uno, only 2 and 3 have interrupt capability
// and d2 is used by the infrared led
Encoder myEnc(3, 4);
long oldPosition = -999;

// IR sensor on D2
const byte interruptPin = 2;
byte lastNumBalls = 0;


/*
 * PWM 3, 5, 6, 9, 10, 11
 * 
 * Pins 5 and 6 are paired on timer0, with base frequency of 62500Hz
 * Pins 9 and 10 are paired on timer1, with base frequency of 31250Hz
 * Pins 3 and 11 are paired on timer2, with base frequency of 31250Hz
 * 
 * Pins 5 and 6 have prescaler values of 1, 8, 64, 256, and 1024
 * Pins 9 and 10 have prescaler values of 1, 8, 64, 256, and 1024
 * Pins 3 and 11 have prescaler values of 1, 8, 32, 64, 128, 256, and 1024
 * 
 * For pins 6 and 5 (OC0A and OC0B):
 * If TCCR0B = xxxxx001, frequency is 64kHz
 * If TCCR0B = xxxxx010, frequency is 8 kHz
 * If TCCR0B = xxxxx011, frequency is 1kHz (this is the default from the Diecimila bootloader)
 * If TCCR0B = xxxxx100, frequency is 250Hz
 * If TCCR0B = xxxxx101, frequency is 62.5 Hz
 * 
 * For pins 9, 10, 11 and 3 (OC1A, OC1B, OC2A, OC2B):
 * If TCCRnB = xxxxx001, frequency is 32kHz
 * If TCCRnB = xxxxx010, frequency is 4 kHz
 * If TCCRnB = xxxxx011, frequency is 500Hz (this is the default from the Diecimila bootloader)
 * If TCCRnB = xxxxx100, frequency is 125Hz
 * If TCCRnB = xxxxx101, frequency is 31.25 Hz
 */
// flywheel motors on D6
// feed motor on D9
#define FLYWHEEL_MOTOR 6
#define FEED_MOTOR     9

// this is the amount of time we wait for the flywheel motors to spin up
// before turning on the feed motor  (msec)
// it may vary depending on the speed...but then again, maybe not
#define FLYWHEEL_MOTOR_SPINUP 1000

// rotary button on D5
// flywheel button on D8
// feed button on D7
#define ROTARY_BUTTON   5
#define FLYWHEEL_BUTTON 8
#define FEED_BUTTON     7

char msg[24];


// debug
//int speed = 0;


enum shotModes { SINGLE, BURST, FULL, MANUAL };
const char *shotModeNames[] = {"single", "burst", "full", "manual"};

//#define TEST_SPEED

#ifdef TEST_SPEED
const char *speedModeNames[] = {
    "10", "20", "30", "40", "50", "60", "70", "80", "90", "100" };
const byte speedPwm[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
#else
enum speedModes { LOW_SPEED, MEDIUM_SPEED, HIGH_SPEED };
const char *speedModeNames[] = {"low", "medium", "high"};
const byte speedPwm[] = {40, 70, 100};    // may need to fine tune this
#endif

enum feedModes { LOW_FEED, MEDIUM_FEED, HIGH_FEED };
const char *feedModeNames[] = {"low", "medium", "high"};
const byte feedPwm[] = {40, 70, 100};    // may need to fine tune this
enum adjustValues { SHOT, SPEED, FEED, EXIT, SAVE };
const char *adjustNames[] = {"shot", "speed", "feed", "exit", "save/exit"};
// this is an int since we check for negative values
int adjust = EXIT;

const byte buttonPins[] = {ROTARY_BUTTON, FLYWHEEL_BUTTON, FEED_BUTTON};
#define DEFAULT_BUTTON_STATE HIGH

byte numButtons;
byte *buttonStates;
byte *lastButtonStates;

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long *lastDebounceTimes = 0;  // the last time the output pin was toggled
#define DEBOUNCE_DELAY 50    // the debounce time in msec

bool isJammed = false;

unsigned long lastActionTime;
bool isSleep = false;
#define SLEEP_TIME 30*1000

bool isCount = true;

bool isFlywheelOn = false;
bool isFeedOn     = false;


void handleEncoderMovement(void);
void drawBall(void);
void updateBallCountAndDelay(unsigned long totalDelay);
void jammed(void);


volatile byte numBalls = 0;
volatile unsigned long lastBall = 0;
volatile unsigned long ballGap = 0;
#define TIME_BETWEEN_BALLS 10

unsigned long lastBatteryCheckTime = 0;
#define BATTERY_TIME 1000


void ball() {
  unsigned long startTime = millis();
  if ((startTime - lastBall) > TIME_BETWEEN_BALLS) {
    ++numBalls;
    ballGap = startTime - lastBall;
    lastBall = startTime;
  }
}


void setup() {
  // set up the pwm outputs
  pinMode(FLYWHEEL_MOTOR, OUTPUT);
  pinMode(FEED_MOTOR, OUTPUT);
  analogWrite(FLYWHEEL_MOTOR, 0);
  analogWrite(FEED_MOTOR, 0);

  // set the pwm frequency
  // to reduce motor whine...needed?
  // timer 0, pin D6, 8 kHz
//  TCCR0A = TCCR0A & 0b11111000 | 0b010;
  // timer 1, pin D9, 4 kHz
//  TCCR1A = TCCR1A & 0b11111000 | 0b010;
  
  Serial.begin(115200);
  Serial.println(F("nerf gun oled"));
  Serial.println(F("compiled:"));
  Serial.print(F(__DATE__));
  Serial.print(F(","));
  Serial.println(F(__TIME__));

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ball, FALLING);

  setUpBattery();
  
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(cal96);
  drawDisplay();

  numButtons = sizeof(buttonPins)/sizeof(buttonPins[0]);
  buttonStates = new byte[numButtons];
  lastButtonStates = new byte[numButtons];
  lastDebounceTimes = new unsigned long[numButtons];

  // initialize the trigger and encoder button pins as inputs
  for (byte i=0; i < numButtons; ++i) {
    buttonStates[i] = DEFAULT_BUTTON_STATE;
    lastButtonStates[i] = DEFAULT_BUTTON_STATE;
    lastDebounceTimes[i] = 0;
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  lastActionTime = millis();

  load();
}


void setUpBattery() {
  //                                                             ADS1015  ADS1115
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  gain = 0.0001873;  // 0.1875mV
  ads.begin();
}


int getPercent(double volts) {
  for (int i = 0; i < 21; ++i) {
    if (volts >= percents[i]) {
      if (i == 0) {
        return 100 - i * 5;
      }
      else {
        // calculate the middle point
        return 100 - i * 5 + (volts - percents[i]) * 5 / (percents[i-1] - percents[i]);
      }
    }
  }
  return 0;
}


void load() {
  Serial.println(F("load"));
  EEPROM.get(ADDRESS, settings);
  if (settings.magic != MAGIC) {
    settings.magic = MAGIC;
    settings.shotMode = MANUAL;
    settings.speedMode = MEDIUM_SPEED;
    settings.feedMode = MEDIUM_FEED;
    save();
  }
}


void save() {
  Serial.println(F("save"));
  EEPROM.put(ADDRESS, settings);
}


void drawDisplay() {
  if (isCount)
    drawBallCount();
  else
    drawMenu();
}


void drawMenu() {
  Serial.println(F("draw menu"));
  oled.clear();

  sprintf(msg, "%c Shot: %6s",
    (adjust == SHOT) ? '>' : ' ',
    shotModeNames[settings.shotMode]);
  oled.println(msg);

  sprintf(msg, "%c Speed: %4s",
    (adjust == SPEED) ? '>' : ' ',
    speedModeNames[settings.speedMode]);
  oled.println(msg);
  
  sprintf(msg, "%c Feed: %4s",
    (adjust == FEED) ? '>' : ' ',
    feedModeNames[settings.feedMode]);
  oled.println(msg);

  sprintf(msg, "%c Exit   %c Save/Exit",
    (adjust == EXIT) ? '>' : ' ', (adjust == SAVE) ? '>' : ' ');
  oled.println(msg);
}


void drawBallCount() {
  Serial.println(F("draw ball count"));
  // show ball count
  sprintf(msg, "%d", numBalls);
  Serial.println(msg);

  oled.clear();

  uint8_t cols = oled.strWidth(msg);
  uint8_t x = (128 - cols)/2;
  oled.setCursor(x,0);
  oled.print(msg);

  // also update the battery level since we cleared the screen
  drawBattery();
}


void loop() {
//  loopDebug();

  if (!isJammed) {
    handleEncoderMovement();

    // test code
//    unsigned long start = 0;
//    if (numBalls == 0) {
//      numBalls = 3;
//      start = millis();
//    }

    if (isCount)
      drawBall();

//    if (start != 0) {
//      unsigned long delta = millis() - start;
//      Serial.println(delta);   // took 6 msec to update the ball count
//    }
  }
  
  checkButtons();

  unsigned long mil = millis();
  if (!isSleep) {
    if ((mil - lastActionTime) > SLEEP_TIME) {
      isSleep = true;
      oled.clear();  
    }
  } 

  if ((mil - lastBatteryCheckTime) > BATTERY_TIME) {
    lastBatteryCheckTime = mil;
    checkBattery();
  }
}


void checkBattery() {
  // don't check the battery if we're sleeping
  // or if we're in the config menu
  if (isSleep || !isCount) {
    return;
  }
  
//  Serial.println(F("check battery"));
  int16_t adc0 = ads.readADC_SingleEnded(0);
  double volts = gain * adc0;
  double v2 = volts * (r1 + r2) / r2;
  int percent = getPercent(v2);
  Serial.print("AIN0: ");
  Serial.print(adc0);
  Serial.print(", volts: ");
  Serial.print(v2);
  Serial.print(", percent: ");
  Serial.println(percent);
  if (percent != lastPercent) {
    lastPercent = percent;
    drawBattery();
  }
}


void drawBattery() {
  sprintf(msg, " %3d%%", lastPercent);
  oled.setFont(Arial14);
  uint8_t cols = oled.strWidth(msg);
  uint8_t x = 128 - cols;
  oled.setCursor(x,0);
  oled.print(msg);
  oled.setFont(cal96);
}


void checkButtons() {
  for (byte i=0; i < numButtons; ++i) {
    byte reading = digitalRead(buttonPins[i]);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonStates[i]) {
      // reset the debouncing timer
      lastDebounceTimes[i] = millis();
    }

    if ((millis() - lastDebounceTimes[i]) > DEBOUNCE_DELAY) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;
        doButton(buttonPins[i], reading);
      }
    }

    // save the reading.  Next time through the loop,
    // it'll be the lastButtonState:
    lastButtonStates[i] = reading;
  }
}

void doButton(byte button, byte state) {
  if (isSleep) {
    wake();
    return;  
  }

  action();

  Serial.print(F("Button: "));
  Serial.print(button);
  Serial.print(F(", state: "));
  Serial.println(state);

  if (button == ROTARY_BUTTON) {
    // we only case about presses, not releases
    if (state == LOW) {
      doRotaryButton();
    }  
  }
  else
    doTriggerButton(button, state);
}


void doRotaryButton() {
  // if we're jammed, pressing the button resets it
  if (isJammed) {
    isJammed = false;
    Serial.println(F("jam reset"));
    drawDisplay();
    return;
  }

  if (isCount) {
    isCount = false;        // go into menu mode
    oled.setFont(Arial14);  // menu font
  }
  else {
    if (adjust == SHOT) {
      if (++settings.shotMode == sizeof(shotModeNames)/sizeof(shotModeNames[0]))
        settings.shotMode = 0;
      Serial.print(F("Shot mode:"));
      Serial.println(settings.shotMode);
      Serial.println(shotModeNames[settings.shotMode]);
    }
    else if (adjust == SPEED) {
      if (++settings.speedMode == sizeof(speedModeNames)/sizeof(speedModeNames[0]))
        settings.speedMode = 0;
      Serial.print(F("Speed mode:"));
      Serial.println(speedModeNames[settings.speedMode]);
    }
    else if (adjust == FEED) {
      if (++settings.feedMode == sizeof(feedModeNames)/sizeof(feedModeNames[0]))
        settings.feedMode = 0;
      Serial.print(F("Feed mode:"));
      Serial.println(feedModeNames[settings.feedMode]);
    }
    else if (adjust == EXIT) {
      isCount = true;
      oled.setFont(cal96);  // ball count font
    }
    else if (adjust == SAVE) {
      save();
      isCount = true;
      oled.setFont(cal96);  // ball count font
    }
  }
  
  drawDisplay();
}


void wake() {
  isSleep = false;
  lastActionTime = millis(); 

  if (isJammed)
    jammed();
  else
    drawDisplay();
}

void action() {
  lastActionTime = millis(); 
}


void fire() {
  Serial.println(F("fire"));
  
  // take note of the current shot count
  byte startBallCount = numBalls;
  
  // spin up the flywheel motors
  Serial.print(F("flywheel on "));
  Serial.println(speedPwm[settings.speedMode]);
  analogWrite(FLYWHEEL_MOTOR, speedPwm[settings.speedMode]);

  // wait for them to get up to speed
  delay(FLYWHEEL_MOTOR_SPINUP);

  // stop the motors if we've reached our shot count (single or burst)
  // or the shot counter is not incrementing
  //   i.e. the ball counter hasn't incremented in the last second
  // or the button is released (in full auto mode)
  byte jamCount = 0;
  if (settings.shotMode == FULL) {
    // spin up the feed motor
    Serial.print(F("feed on "));
    Serial.println(feedPwm[settings.feedMode]);
    analogWrite(FEED_MOTOR, feedPwm[settings.feedMode]);
  
    Serial.print(F("full mode"));
    while (jamCount < 10 && 
           (digitalRead(FLYWHEEL_BUTTON) == LOW || digitalRead(FEED_BUTTON) == LOW)) {
      startBallCount = numBalls;
      updateBallCountAndDelay(100);
      if (numBalls == startBallCount)
        ++jamCount;
      else
        jamCount = 0;
    }
  }
  else {
    // single or burst
    // spin up the feed motor
    // use low speed since if we use a higher feed rate, more balls
    // come out than we want
    // electric motor braking may fix this, but this works for now
    Serial.println(F("feed on low"));
    analogWrite(FEED_MOTOR, feedPwm[LOW_FEED]);
    
    Serial.print(F("start ball count "));
    Serial.println(startBallCount);
    byte target = startBallCount + ((settings.shotMode == SINGLE) ? 1 : 3);
    Serial.print(F("target ball count "));
    Serial.println(target);
    while (jamCount < 10 && numBalls < target) {
      startBallCount = numBalls;
      updateBallCountAndDelay(100);
      //I swear taking these prints out makes it stop working....wtf?
      Serial.print(F("current ball count "));
      Serial.println(numBalls);
      if (numBalls == startBallCount)
        ++jamCount;
      else
        jamCount = 0;
    }
  }

  // turn everything off
  Serial.println(F("flywheel and feed off"));
  analogWrite(FEED_MOTOR, 0);
  analogWrite(FLYWHEEL_MOTOR, 0);

  // if there was a jam, tell the user
  if (jamCount >= 10) {
    jammed();
  }
}


void doTriggerButton(byte button, byte state) {
  // if we're jammed, neither trigger button is functional
  if (isJammed)
    return;
    
  if (settings.shotMode == MANUAL) {
    // in manual mode the triggers control the motors directly
    if (button == FLYWHEEL_BUTTON) {
      if (state == LOW) {
        // flywheel trigger pressed
        if (!isFlywheelOn) {
          // turn flywheel motor on
          isFlywheelOn = true;
          analogWrite(FLYWHEEL_MOTOR, speedPwm[settings.speedMode]);
          Serial.println(F("flywheel on"));
        }
      }
      else {
        // flywheel trigger released
        if (isFlywheelOn) {
          // turn flywheel motor off
          isFlywheelOn = false;
          analogWrite(FLYWHEEL_MOTOR, 0);
          Serial.println(F("flywheel off"));
        }
      }
    }
    else if (button == FEED_BUTTON) {
      if (state == LOW) {
        // feed trigger pressed
        if (!isFeedOn) {
          // turn feed motor on
          isFeedOn = true;
          analogWrite(FEED_MOTOR, feedPwm[settings.feedMode]);
          Serial.println(F("feed on"));
        }
      }
      else {
        // feed trigger released
        if (isFeedOn) {
          // turn feed motor off
          isFeedOn = false;
          analogWrite(FEED_MOTOR, 0);
          Serial.println(F("feed off"));
        }
      }
    }
    return;
  }

  // fire if either button has been pressed  
  if (state == LOW) {
    fire(); 
  }
}


void updateBallCountAndDelay(unsigned long totalDelay) {
  // get the current millisec then call the update call, then delay
  // for totalDelay - time spent in update call.
  unsigned long start = millis();
  drawBall();    // this takes around 6 msec to run (if there is a ball update)
                 // maybe longer for ball counts of more than a single digit
  unsigned long delta = millis() - start;
  delay(totalDelay - delta);
}


void jammed(void) {
  Serial.println(F("jammed"));
  
  oled.clear();
  oled.setFont(Arial14);
  oled.set2X();
  oled.print(F("JAMMED"));

  oled.set1X();
  if (isCount)
    oled.setFont(cal96);    // ball count font
  else
    oled.setFont(Arial14);  // menu font

  isJammed = true;
}

void drawBall(void) {
  // check for a ball
  if (numBalls != lastNumBalls) {
    lastNumBalls = numBalls;
    drawBallCount();
//    Serial.print(numBalls);
//    Serial.print(": ");
//    Serial.println(ballGap);
  }
}

void handleEncoderMovement(void) {
  int dir = readEncoder();
  if (dir == 0)
    return;

  if (isSleep) {
    wake();
    return;  
  }
  else
    action();

  Serial.print(F("encoder moved "));
  Serial.println(dir);
  if (isCount) {
    if (dir > 0)
      numBalls = min(999, numBalls+1);
    else
      numBalls = max(0, numBalls-1);
  }
  else {
    if (dir > 0) {
      if (++adjust > SAVE)
        adjust = SHOT;
    }
    else {
      if (--adjust < SHOT)
        adjust = SAVE;
    }
    drawMenu();   
  }
}

int readEncoder() {
  // read rotary encoder
  int dir = 0;
  long newPosition = myEnc.read();
//  Serial.println(newPosition);
  if (newPosition != oldPosition) {
    if (oldPosition != -999) {
      if (newPosition < oldPosition-3) {
        oldPosition = newPosition;
        dir = 1;
      }
      else if (newPosition > oldPosition+3) {
        oldPosition = newPosition;
        dir = -1;
      }
    }
    else
      oldPosition = newPosition;
  }
  return dir;
}

//void lookDebug() {
//  // read rotary encoder
//  long newPosition = myEnc.read();
//  if (newPosition != oldPosition) {
//    if (oldPosition == -999)
//      // ignore
//      ;
//    else if (newPosition < oldPosition)
//      speed = max(0, speed-1);
//    else
//      speed = min(255, speed+1);
//    analogWrite(flywheelMotor, speed);
//    oldPosition = newPosition;
//    int speedPercent = round((float)speed * 100 / 255);
//    sprintf(msg, "%3d%%", speedPercent);
//    Serial.println(msg);
//    lcd.setCursor(7,0);
//    lcd.print(msg);
//  }
//
//  // check for a ball
//  if (numBalls != lastNumBalls) {
//    lastNumBalls = numBalls;
//    sprintf(msg, "%3d", numBalls);
//    Serial.println(msg);
//    lcd.setCursor(7,1);
//    lcd.print(msg);
//  }
//
//  // check for rotary encoder button
//  if (digitalRead(rotaryButton) == 0) {
//    numBalls = 0;
//  }
//}


/*
 * todo:
 * save the mode values in eeprom when they change, and read at startup
 * add electronic motor braking
 * can ball speed be measured with the isr (time btween rising and falling edges?)
 * add laser.   use button to turn is on/off
 * fix screen draw...rught now I clear the entire screen before drawing...this 
 * makes it flash...just erase the part of the screen that needs to be blank
 * hardware debouncing:  RC and 7414 IC
 * 
 * battery level.  on the oled or another display?
 */
