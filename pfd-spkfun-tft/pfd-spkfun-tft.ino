// PFD for colour TFT screen
// Shamim Mohamed 2020-01-09

#include <stdio.h>
#include <math.h>
#include <Phliar_utils.h>

#include <SPI.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define HYPERDISPLAY_HAVE_CUSTOM_CONFIG 1
#define HYPERDISPLAY_DRAWING_LEVEL 4
#define HYPERDISPLAY_USE_MATH 1
#define HYPERDISPLAY_USE_PRINT 1
#define HYPERDISPLAY_INCLUDE_DEFAULT_FONT 1
#define HYPERDISPLAY_USE_RAY_TRACING 1

#include <HyperDisplay_KWH018ST01_4WSPI.h>

// Stupid Arduino -- all typedefs) MUST be in header files, and ALL those includes
// MUST be before any non-cpp lines (or the auto-prototyping breaks).

#include "point.h"
#include "config.h"
#include "txt-align.h"
#include "clip.h"

const int refreshRate = 5;

// I2C addresses
const uint8_t bn0055_addr = 0x28;
const uint8_t bmp085_addr = 0x77; // Unused, for information only.

// The "LEVEL" button.
const int8_t rstPin = 21;

// Potentiometer input pin
const int8_t qnhInput = 23;

// Globals
char longbuf[132]; // For printf etc.
bool verbose = true; // To control printing to Serial
float qnh = 29.92; // To be modified by a rotary encoder...

// The Baro sensor.
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// The IMU.
Adafruit_BNO055 bno = Adafruit_BNO055(55, bn0055_addr);

// The timer.
IntervalTimer loopTimer;

// Forward declarations.
void showPFD(float g, sensors_vec_t *orient, sensors_vec_t *magnetic);
void panic(const char* msg);

void setup() {
    // Set up the serial console.
    Serial.begin(115200);
    delay(1250);
    verbose = !!Serial;
    if (verbose)
	Serial.println("Started.");

    // Displays
    tftInit();

    // Sensors
    bnoInit();
    bmpInit();

    pinMode(rstPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rstPin), resetHandler, FALLING);

    // Set up the qnhInput pin
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
 
