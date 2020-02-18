// PFD for colour TFT screen
// Shamim Mohamed 2020-01-09

//#define USE_T3

#include <stdio.h>
#include <math.h>
#include <Phliar_utils.h>

#include <SPI.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#ifdef USE_T3
#include <ILI9341_t3.h>
#else
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#endif

#include "point.h"

const int refreshRate = 5;

// PFD vertical scale: 8 deg up, 8 deg down.
const float pitchFullScale = 8.0;

// I2C addresses
const uint8_t bn0055_addr = 0x28;
const uint8_t bmp085_addr = 0x77; // Unused, for information only.

const int tftDC = 4;
const int tftCS = 10;
const int tftSDCS = 5;
const int tftReset = 8;

// The "LEVEL" button.
const int8_t rstPin = 21;

// The backlight pin on the TFT
const int8_t tftBacklight = 15;

// Potentiometer input pin
const int8_t qnhInput = 23;

// Globals
char longbuf[132]; // For printf etc.
bool verbose = true; // To control printing to Serial
float qnh = 29.92; // To be modified by a rotary encoder...

// The TFT panel.
#ifdef USE_T3
ILI9341_t3 tft = ILI9341_t3(tftCS, tftDC);
#else
Adafruit_ILI9341 tft = Adafruit_ILI9341(tftCS, tftDC);
#endif

// The Baro sensor.
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// The IMU.
Adafruit_BNO055 bno = Adafruit_BNO055(55, bn0055_addr);

// The timer.
IntervalTimer loopTimer;

// Forward declarations.
void showPFD(float gx, float gy, float gz, sensors_vec_t *orient, sensors_vec_t *magnetic,
	     int alt);
void panic(const char* msg);

void setup() {
    tftInit();
    Serial.begin(115200);

    // Let things initialize.
    delay(1000);

    verbose = Serial;
    if (verbose)
	Serial.println("Started.");

    // Sensors
    bnoInit();
    bmpInit();

    pinMode(rstPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rstPin), resetHandler, FALLING);

    // Set up the QNH pin
    // ...

    // Let's go!
    loopTimer.begin(handler, (int)(0.5 + 1.0e6/refreshRate));
    showSensorValues();
}

volatile bool ding = false;
void handler() {
    ding = true;
}

volatile bool resetLevel = true; // Reset everything to level on start.
void resetHandler() {
    // Set the pitch and roll values to 0.
    resetLevel = true;
}

void loop() {
    // No need to disable interrupts since ding is one byte.
    if (!ding)
	return;
    ding = false;
    unsigned long loopStart = micros();

    showSensorValues();

    unsigned long elapsed = micros() - loopStart;
    if (ding && verbose)
	streamPrintf(Serial, "WARNING: loop took %.2f ms", elapsed/1000.0);
}

uint16_t heading(float hx, float hy, float hz) {
    uint16_t hdg = (uint16_t)(hx + 0.5);
    // Sensor is mounted at 90º
    if (hdg > 90)
        return hdg - 90;
    return hdg + 270;
}

// Print a message and don't return.
void panic(const char* msg) {
    if (verbose)
	Serial.print(msg);
    for(;;);
}
 
