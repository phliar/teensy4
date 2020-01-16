// PFD
// Shamim Mohamed 2019-12-30

#if 0
#define SSD1325 1
#else
#define SSD1325 0
#endif

#include <stdio.h>
#include <math.h>
#include <Phliar_utils.h>

#include <SPI.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_ADXL345_U.h>
 
#include <Adafruit_GFX.h>
#if SSD1325
#include <Adafruit_SSD1325.h>
#else
#include <Adafruit_SSD1306.h>
#endif

#include "point.h"

const int refreshRate = 5;

// PFD vertical scale: 8 deg up, 8 deg down.
const float pitchFullScale = 8.0;

// I2C addresses
const uint8_t bn0055_addr = 0x28;
const uint8_t bmp085_addr = 0x77; // Unused, for information only.

// OLED display dimensions
const int oledWidth = 128;
const int oledHeight = 64;

// OLED pins
const int oledDC = 5;
const int oledRST = 6;
const int SPCS = 10;

// The "LEVEL" button.
const int rstPin = 22;

// Globals
char longbuf[132]; // For printf etc.
bool verbose = true; // To control printing to Serial
float qnh = 29.92; // To be modified by a rotary encoder...

// The OLED display, using HW SPI.
#if SSD1325
Adafruit_SSD1325 oled(oledDC, oledRST, SPCS);
#else
Adafruit_SSD1306 oled(oledWidth, oledHeight, &SPI, oledDC, oledRST, SPCS);
#endif

// The Baro sensor.
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(7150085);

// ADXL305 accelerometer: assign a unique ID.
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(7150345);

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
    while (!Serial)
        ;
    Serial.println("Started.");

    oledInit();

    // Sensors
    adxlInit();
    bnoInit();
    bmpInit();

    pinMode(rstPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rstPin), resetHandler, FALLING);

    // Let's go!
    loopTimer.begin(handler, (int)(0.5 + 1.0e6/refreshRate));
    showSensorValues(); // Still in verbose, this will also print to console.
    verbose = false;
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
    if (ding)
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
    Serial.print(msg);
    for(;;);
}
 
