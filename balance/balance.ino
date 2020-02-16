// Balancing, using only the ADXL345 3x accelerometers to sense g

#include <Arduino.h>
#include <stdint.h>

#include <stdio.h>
#include <math.h>
#include <Phliar_utils.h>

#include "point.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1325.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Motor driver
#include "SCMD.h"
#include "SCMD_config.h"
const uint8_t SCMD_I2C_ADDR = 0x58; // jumpers "0011" on SCMD
const uint8_t SCMD_FRWD = 0;
const uint8_t SCMD_BKWD = 1;
SCMD motors;

const uint8_t displayCS = 10;
const uint8_t displayRST = 6;
const uint8_t displayDC = 5;
Adafruit_SSD1325 oled(displayDC, displayRST, displayCS);
const int OLED_W = 128;
const int OLED_H = 64;
const int FONT_W = 6;
const int FONT_H = 8;
const int OLED_COLS = OLED_W / FONT_W;
const int OLED_ROWS = OLED_H / FONT_H;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(820715);
const float Rad2Deg = 180 / M_PI;
const float pxPerDeg = 10.0;

bool verbose;

void setup() {
    oledInit();
    serialInit();
    accelInit();
    initMotors();
    initPitchBuf();
    if (verbose)
        Serial.println("Initialization complete.");
}

void loop() {
    oled.clearDisplay();

    float pitch, roll;
    sensors_event_t evt; 
    accel.getEvent(&evt);
    getAngles(evt.acceleration.x, evt.acceleration.y, evt.acceleration.z,
	      &pitch, &roll);
    logPitch(pitch);

    float p = meanPitch();
    if (verbose) {
	streamPrintf(Serial, "%.1f %.1f (%.2f)\n", pitch, roll, p);
    }

    rebalance(p);
    showScale(p, pitch, roll);

    oled.display();
}

/*
 * Initialization
 */

void serialInit() {
    Serial.begin(115200);
    delay(1200);
    verbose = !!Serial;
    if (verbose)
        Serial.println("BEGIN");
}

void oledInit() {
    oled.begin();
    oled.display();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
}

void accelInit() {
    if (!accel.begin())
        panic("No ADXL345 found");
    accel.setRange(ADXL345_RANGE_2_G);
}

void initMotors() {
    if (verbose)
        Serial.print("Motor driver");

    motors.settings.commInterface = I2C_MODE;
    motors.settings.I2CAddress = SCMD_I2C_ADDR;
    for (;;) {
        int id = motors.begin();
        if (id == 0xa9)
            break;
        if (verbose)
            streamPrintf(Serial, "Motor driver mismatch: 0x%02x\n", id);
        delay(500);
    }
    if (verbose)
        Serial.print(" initialized");

    // Wait for enumeration
    while (!motors.ready())
        ;
    if (verbose)
        Serial.print(" enumerated");

    while (motors.busy())
        ;
    motors.enable();
    if (verbose)
        Serial.print(" enabled");

    if (verbose)
        Serial.println();
}

/*
 * Sensors
 */

inline int sgn(float f) {
    if (f < 0)
        return -1;
    return 1;
}

// Return pitch and roll angles in degrees.
void getAngles(float gx, float gy, float gz, float* pitch, float* roll) {
    // cf. www.nxp.com/files/sensors/doc/app_note/AN_3461.pdf
    const float mu = 0.1;
    *pitch = Rad2Deg * atan2(-gx, sqrt(gy*gy + gz*gz));
    *roll =  Rad2Deg * atan2(gy, sgn(gz)*sqrt(gz*gz + mu * gx*gx));
}

/*
 * OLED graphics
 */

void markDiamond(point p) {
    oled.fillTriangle(p.x, p.y-3, p.x+3, p.y, p.x-3, p.y, WHITE);
    oled.fillTriangle(p.x, p.y+3, p.x+3, p.y, p.x-3, p.y, WHITE);
}

void markLine(point p) {
    oled.drawLine(p.x, p.y-5, p.x, p.y+5, WHITE);
}

void showScale(float pMean, float pitch, float roll) {
    oled.setCursor(0, 0);
    streamPrintf(oled, "%6.1f P", pMean);
    oled.setCursor(OLED_W-6*8, 0);
    streamPrintf(oled, "%6.1f", roll);

    oled.drawLine(0, OLED_H/2, OLED_W, OLED_H/2, WHITE);
    oled.drawLine(OLED_W/2, OLED_H/2-8, OLED_W/2, OLED_H/2+8, WHITE);
    oled.drawLine(OLED_W/2 - 5*pxPerDeg, OLED_H/2-8,
		     OLED_W/2 - 5*pxPerDeg, OLED_H/2+8, WHITE);
    oled.drawLine(OLED_W/2 + 5*pxPerDeg, OLED_H/2-8,
		     OLED_W/2 + 5*pxPerDeg, OLED_H/2+8, WHITE);

    point p = {0, OLED_H/2};
    for (p.x = 4; p.x < OLED_W; p.x += 10)
        oled.drawLine(p.x, p.y-3, p.x, p.y+3, WHITE);
    p.x = OLED_W/2 + int(0.5 + pitch * pxPerDeg);
    markLine(p);

    p.x = OLED_W/2 + int(0.5 + pMean * pxPerDeg);
    markDiamond(p);
}

/*
 * Low-pass filter for pitch angle
 */

const int filterDepth = 10;     // Use the mean of last few readings.
float readings[filterDepth];
int rIdx = 0;
float readingSum = 0.0;

void initPitchBuf() {
    for (int i = 0; i < filterDepth; i++)
        readings[i] = 0.0;
}

void logPitch(float d) {
    readingSum += d - readings[rIdx];
    readings[rIdx++] = d;
    rIdx %= filterDepth;
}

float meanPitch() {
    return readingSum/filterDepth;
}

/*
 * Utils
 */

void error(const char* fmt, ...) {
    char buf[OLED_COLS+1];

    va_list args;
    va_start(args, fmt);
    (void) vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    buf[OLED_COLS] = 0;

    oled.setCursor(0, OLED_ROWS-1);
    streamPrintf(oled, "%-20s", buf);
}

void panic(const char* msg) {
    if (verbose)
        Serial.println(msg);
    error("%s", msg);
    for (;;)
        ;
}
