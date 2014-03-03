// Configurable Settings
/*****************************************************************************/

int verbose = 0;
unsigned long blinkSpeed = 200;

// Drink Actions consist of an array consisting of:
// - an id for the motor / servo (motor ids: 0-4, servo ids: 100, 101)
// - a start time in milliseconds
// - an end time in milliseconds

unsigned long drinkActions[2][10][3] = {
  {
    {  0,     0,  500},
    {  1,   500, 1000},
    {  2,  1000, 1500},
    {  3,  1500, 2000},
    {100,     0, 2000},
    {101,  1000, 3000}
  },
  {
    {  0,     0,  1000},
    {  3,     0,  3000}
  }
};

int drinkActionLengths[] = {
  6,
  2,
  0,
  0,
  0,
  0
};


// State Variables (Don't fuck with these yo.)
/*****************************************************************************/

unsigned long lastInfo = 0;
int showInfo = 1;
// int currentDrink = -1;
int currentDrink = 0;
unsigned long drinkStart = 0;
int actionNumber;


// Servo
/*****************************************************************************/

#include <Servo.h>

int servoPins [] = {9, 10};
int servoStates [] = {0, 0};
// indecies: position 0: off, position 1: on
int servoStatePositions [] = {0, 30}; // (0-180)
int nOfServos = 2;

// create servo object to control a servo
// a maximum of eight servo objects can be created
Servo servoA;
Servo servoB;
Servo servos [] = { servoA, servoB };

void setupServos()
{
  int i;
  for ( i=0; i<nOfServos; ++i )
  {
    servos[i].attach(servoPins[i]);
  }
}

void loopServos()
{
  int i;
  for ( i=0; i<nOfServos; ++i )
  {
    servos[i].write(servoStatePositions[servoStates[i]]);
  }
}


// Light Strip
/*****************************************************************************/

#include "LPD8806.h"
#include "SPI.h"

// Number of RGB LEDs in strand:
int nLEDs = 74;
int currentLED = 0;
int previousLED = 0;
unsigned long lastLEDTime = 0;

// Chose 2 pins for output; can be any valid output pins:
int dataPin  = SDA;
int clockPin = SCL;

// First parameter is the number of LEDs in the strand.  The LED strips
// are 32 LEDs per meter but you can extend or cut the strip.  Next two
// parameters are SPI data and clock pins:
LPD8806 strip = LPD8806(nLEDs, dataPin, clockPin);

// You can optionally use hardware SPI for faster writes, just leave out
// the data and clock pin parameters.  But this does limit use to very
// specific pins on the Arduino.  For "classic" Arduinos (Uno, Duemilanove,
// etc.), data = pin 11, clock = pin 13.  For Arduino Mega, data = pin 51,
// clock = pin 52.  For 32u4 Breakout Board+ and Teensy, data = pin B2,
// clock = pin B1.  For Leonardo, this can ONLY be done on the ICSP pins.
//LPD8806 strip = LPD8806(nLEDs);

void setupLightStrip() {
  // Start up the LED strip
  strip.begin();

  // Update the strip, to start they are all 'off'
  strip.show();
}

void loopLightStrip() {
  if (millis() - lastLEDTime > 30)
  {
    lastLEDTime = millis();
    previousLED = currentLED;
    currentLED = (currentLED + 1) % strip.numPixels();
  }
  strip.setPixelColor(previousLED, strip.Color(0, 0, 0));
  strip.setPixelColor(currentLED,  strip.Color(127, 0, 0));
  strip.show();
}

// void loopLightStrip() {
//   // Send a simple pixel chase in...
//   colorChase(strip.Color(127, 127, 127), 50); // White
//   colorChase(strip.Color(127,   0,   0), 50); // Red
//   colorChase(strip.Color(127, 127,   0), 50); // Yellow
//   colorChase(strip.Color(  0, 127,   0), 50); // Green
//   colorChase(strip.Color(  0, 127, 127), 50); // Cyan
//   colorChase(strip.Color(  0,   0, 127), 50); // Blue
//   colorChase(strip.Color(127,   0, 127), 50); // Violet

//   // Send a theater pixel chase in...
//   theaterChase(strip.Color(127, 127, 127), 50); // White
//   theaterChase(strip.Color(127,   0,   0), 50); // Red
//   theaterChase(strip.Color(127, 127,   0), 50); // Yellow
//   theaterChase(strip.Color(  0, 127,   0), 50); // Green
//   theaterChase(strip.Color(  0, 127, 127), 50); // Cyan
//   theaterChase(strip.Color(  0,   0, 127), 50); // Blue
//   theaterChase(strip.Color(127,   0, 127), 50); // Violet

//   // Fill the entire strip with...
//   colorWipe(strip.Color(127,   0,   0), 50);  // Red
//   colorWipe(strip.Color(  0, 127,   0), 50);  // Green
//   colorWipe(strip.Color(  0,   0, 127), 50);  // Blue

//   rainbow(10);
//   rainbowCycle(0);  // make it go through the cycle fairly fast
//   theaterChaseRainbow(50);
// }

// void rainbow(uint8_t wait) {
//   int i, j;

//   for (j=0; j < 384; j++) {     // 3 cycles of all 384 colors in the wheel
//     for (i=0; i < strip.numPixels(); i++) {
//       strip.setPixelColor(i, Wheel( (i + j) % 384));
//     }
//     strip.show();   // write all the pixels out
//     delay(wait);
//   }
// }

// // Slightly different, this one makes the rainbow wheel equally distributed
// // along the chain
// void rainbowCycle(uint8_t wait) {
//   uint16_t i, j;

//   for (j=0; j < 384 * 5; j++) {     // 5 cycles of all 384 colors in the wheel
//     for (i=0; i < strip.numPixels(); i++) {
//       // tricky math! we use each pixel as a fraction of the full 384-color wheel
//       // (thats the i / strip.numPixels() part)
//       // Then add in j which makes the colors go around per pixel
//       // the % 384 is to make the wheel cycle around
//       strip.setPixelColor(i, Wheel( ((i * 384 / strip.numPixels()) + j) % 384) );
//     }
//     strip.show();   // write all the pixels out
//     delay(wait);
//   }
// }

// // Fill the dots progressively along the strip.
// void colorWipe(uint32_t c, uint8_t wait) {
//   int i;

//   for (i=0; i < strip.numPixels(); i++) {
//       strip.setPixelColor(i, c);
//       strip.show();
//       delay(wait);
//   }
// }

// // Chase one dot down the full strip.
// void colorChase(uint32_t c, uint8_t wait) {
//   int i;

//   // Start by turning all pixels off:
//   for(i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, 0);

//   // Then display one pixel at a time:
//   for(i=0; i<strip.numPixels(); i++) {
//     strip.setPixelColor(i, c); // Set new pixel 'on'
//     strip.show();              // Refresh LED states
//     strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
//     delay(wait);
//   }

//   strip.show(); // Refresh to turn off last pixel
// }

// //Theatre-style crawling lights.
// void theaterChase(uint32_t c, uint8_t wait) {
//   for (int j=0; j<10; j++) {  //do 10 cycles of chasing
//     for (int q=0; q < 3; q++) {
//       for (int i=0; i < strip.numPixels(); i=i+3) {
//         strip.setPixelColor(i+q, c);    //turn every third pixel on
//       }
//       strip.show();

//       delay(wait);

//       for (int i=0; i < strip.numPixels(); i=i+3) {
//         strip.setPixelColor(i+q, 0);        //turn every third pixel off
//       }
//     }
//   }
// }

// //Theatre-style crawling lights with rainbow effect
// void theaterChaseRainbow(uint8_t wait) {
//   for (int j=0; j < 384; j++) {     // cycle all 384 colors in the wheel
//     for (int q=0; q < 3; q++) {
//         for (int i=0; i < strip.numPixels(); i=i+3) {
//           strip.setPixelColor(i+q, Wheel( (i+j) % 384));    //turn every third pixel on
//         }
//         strip.show();

//         delay(wait);

//         for (int i=0; i < strip.numPixels(); i=i+3) {
//           strip.setPixelColor(i+q, 0);        //turn every third pixel off
//         }
//     }
//   }
// }


/* Helper functions */

//Input a value 0 to 384 to get a color value.
//The colours are a transition r - g -b - back to r

uint32_t Wheel(uint16_t WheelPos)
{
  byte r, g, b;
  switch(WheelPos / 128)
  {
    case 0:
      r = 127 - WheelPos % 128;   //Red down
      g = WheelPos % 128;      // Green up
      b = 0;                  //blue off
      break;
    case 1:
      g = 127 - WheelPos % 128;  //green down
      b = WheelPos % 128;      //blue up
      r = 0;                  //red off
      break;
    case 2:
      b = 127 - WheelPos % 128;  //blue down
      r = WheelPos % 128;      //red up
      g = 0;                  //green off
      break;
  }
  return(strip.Color(r,g,b));
}


// Motor Shield
/*****************************************************************************/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motors [] = { AFMS.getMotor(1), AFMS.getMotor(2), AFMS.getMotor(3), AFMS.getMotor(4) };
int nOfMotors = 4;

void setupMotorShield() {
  int i;
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  for ( i=0 ; i<nOfMotors ; ++i )
  {
    // turn off motor
    motors[i]->run(RELEASE);
  }
}


// Buttons
/*****************************************************************************/

int buttonInputs[] = { 2, 4, 6, A2, 8, 12 };
int buttonLights[] = { 3, 5, 7, 11, A3, 13 };
int buttonStates[] = { 0, 0, 0, 0,  0,  0 };
int nOfButtons = 6;

void setupButtons() {
  int i;

  for ( i=0; i<nOfButtons; ++i )
  {
    // initialize the LED pin as an output:
    pinMode(buttonLights[i], OUTPUT);
    // initialize the pushbutton pin as an input:
    pinMode(buttonInputs[i], INPUT);
    digitalWrite(buttonInputs[i], HIGH);
  }
  // The Shiny Startup Sequence
  int timePerLight = 30;
  for ( i=0; i<nOfButtons; ++i )
  {
    digitalWrite(buttonLights[i], HIGH);
    delay(timePerLight);
  }
  for ( i=0; i<nOfButtons; ++i )
  {
    digitalWrite(buttonLights[i], LOW);
    delay(timePerLight);
  }
  delay(50);
  for ( i=0; i<nOfButtons; ++i )
  {
    digitalWrite(buttonLights[nOfButtons-i-1], HIGH);
    delay(timePerLight);
  }
  for ( i=0; i<nOfButtons; ++i )
  {
    digitalWrite(buttonLights[nOfButtons-i-1], LOW);
    delay(timePerLight);
  }
  delay(50);
}

void printButtonInfo(int i) {
  Serial.print("Button #");
  Serial.print(i);
  Serial.print(" on input pin ");
  if (buttonInputs[i] < 10) Serial.print(" ");
  Serial.print(buttonInputs[i]);
  Serial.print(" on output pin ");
  if (buttonLights[i] < 10) Serial.print(" ");
  Serial.print(buttonLights[i]);
  Serial.print(": ");
  Serial.print(buttonStates[i]);
  Serial.print("\n");
}

void loopButtons() {
  int i;
  int buttonState = 0;
  // Blink lights every 300 milliseconds while pouring drink
  int blink = millis()/blinkSpeed % 2;
  int output;
  // Serial.println(blink);

  for ( i=0; i<nOfButtons; ++i )
  {
    // read the state of the pushbutton value:
    buttonStates[i] = !digitalRead(buttonInputs[i]);

    // Verbose Button Info
    // if (showInfo) printButtonInfo(i, buttonState);
  }
  for ( i=0; i<nOfButtons; ++i )
  {
    // check if the pushbutton is pressed or it is the current drink
    output = ((buttonStates[i] != HIGH) && (currentDrink != i || blink) );
    // turn LED on or off
    digitalWrite(buttonLights[i], output);
  }
}

// Drink Mixing!
/*****************************************************************************/

void startDrink(int i)
{
  currentDrink = i;
  actionNumber = -1;
  drinkStart = millis();
}

void loopDrinkLogic() {
  int i, id, shouldRun;
  unsigned long now = millis() - drinkStart;
  unsigned long* action;
  unsigned long start, end;
  unsigned long drinkFinished = true;
  int motorEnables[4] = {0, 0, 0, 0};
  for ( i=0; i<nOfServos; ++i)
  {
    servoStates[i] = 0;
  }
  // Starting a new drink if one isn't being poured and a button is pushed.
  if (currentDrink == -1)
  {
    for ( i=0; i<nOfButtons; ++i )
    {
      if (buttonStates[i]) startDrink(i);
    }
  }
  if (currentDrink == -1) return;
  // Continuing to prep the current drink
  for ( i=0; i<drinkActionLengths[currentDrink]; ++i)
  {
    action = drinkActions[currentDrink][i];
    // ((void*)(drinkActions[currentDrink])[i])
    // action = (unsigned long[3])((void [])drinkActions[currentDrink])[i];
    id = (int) action[0];
    start = action[1];
    end = action[2];
    shouldRun = (now >= start) && (now < end);
    if (end > now) drinkFinished = false;
    // if (showInfo) Serial.print("Motor #");
    // if (showInfo) Serial.println(id);
    // if (showInfo) Serial.print("now ");
    // if (showInfo) Serial.println(now);
    // if (showInfo) Serial.print("start ");
    // if (showInfo) Serial.println(start);
    // if (showInfo) Serial.print("end ");
    // if (showInfo) Serial.println(end);
    // if (showInfo and shouldRun) Serial.println("!!active!!");

    if (id < 100)
    {
      motorEnables[id] |= shouldRun;
    }
    else
    {
      servoStates[id-100] |= shouldRun;
    }
  }
  if (showInfo) Serial.print("now ");
  if (showInfo) Serial.println(now);
  for (i =0; i< 4; ++i)
  {
    if (showInfo) Serial.print("motor #");
    if (showInfo) Serial.print(i);
    if (showInfo && motorEnables[i]) Serial.println(": ON");
    if (showInfo && !motorEnables[i]) Serial.println(": OFF");
    if (motorEnables[i])
    {
      motors[i]->setSpeed(200);
      motors[i]->run(FORWARD);
    }
    else
    {
      motors[i]->run(RELEASE);
    }
  }
  if (drinkFinished)
  {
    currentDrink = -1;
  }
}

// Setup
/*****************************************************************************/

void setup() {
  // Serial
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  setupMotorShield();
  // setupLightStrip();
  setupButtons();
  setupServos();
}


// Loop
/*****************************************************************************/
void loop() {
  showInfo = (millis() - lastInfo > 1000) and verbose;
  if (showInfo)
  {
    Serial.println("-------------------------");
    lastInfo = millis();
  }

  loopButtons();
  // loopLightStrip();
  loopDrinkLogic();
  loopServos();

  if (showInfo and currentDrink > -1)
  {
    Serial.print("Current Drink: #");
    Serial.println(currentDrink);
  }
}