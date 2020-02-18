/*
 * A Rangefinder, using the VL53L1X ToF sensor.
 */

#include <stdio.h>
#include <string.h>
#include <Phliar_utils.h>

#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>
#include <SparkFun_VL53L1X.h>

// I2C addresses
const uint8_t lcd_addr = 0x3f;
const uint8_t vl53_addr = 0x29;	// Unused, for information only.

// The ToF sensor.
SFEVL53L1X dist_sensor(Wire);   // for SCL0/SDA0 -- or Wire1 to use SCL1/SDA1
const int maxRange = 6000;

// Pin assignments for the i2c adapter for the LCD
const int bl=3 /* backlight */, en=2 /*enable*/, rs=0 /*reset*/,
    rw=1 /*read-write*/;
const int d4=4, d5=5, d6=6, d7=7;

// The LCD display.
const int lcdRows = 4;
const int lcdCols = 20;
const int lcdWidthPixels = lcdCols * 5;
LiquidCrystal_I2C lcd(lcd_addr, en, rw, rs, d4, d5, d6, d7, bl,
                      POSITIVE /* backlight polarity */);
const int mmPerPx = maxRange / lcdWidthPixels;

// Sensor params
const int displayRate = 50;	       // readings per second
const int filterDepth = displayRate/4; // for running average of last 0.25s

// Globals
bool verbose = true;		// To control printing to Serial

void setup() {
    // Set up the serial console.
    Serial.begin(115200);
    delay(1500);
    verbose = !!Serial;
    if (verbose)
        Serial.println("Started.");

    // Set up the LCD and the special chars.
    lcdInit(lcdCols, lcdRows);

    // Init the sensor.
    if (dist_sensor.begin() != 0)
        panic("No VL53L1X found.");

    Timer1.initialize(1e6/displayRate);
    Timer1.attachInterrupt(intrupt);
    loop();
}

volatile bool ding = true;
void intrupt() {
    ding = true;
}

void loop() {
    if (!ding)
        return;
    ding = false;

    dist_sensor.startRanging();
    int dist = dist_sensor.getDistance();
    dist_sensor.stopRanging();
    if (verbose)
        streamPrintf(Serial, "%dmm", dist);
    logReading(dist);

    showBargraph(0, dist);
    showNumeric(1, meanReading());
    if (verbose)
        Serial.println();
}

int readings[filterDepth];
int rIdx = 0;
int readingSum = 0;

void logReading(int d) {
    readingSum += d - readings[rIdx];
    readings[rIdx++] = d;
    rIdx %= filterDepth;
}

int meanReading() {
    return (int)(0.5 + readingSum/(float)filterDepth);
}

// Draw a bargraph on the LCD for the distance reported.
void showBargraph(int row, int d) {
    if (d < 0 || d > maxRange) {
	error("Error! %d", d);
	return;
    }

    byte pixels = (byte)(d / mmPerPx);
    byte stickPos = pixels % 5;
    byte bars = (stickPos == 0) ? pixels/5 - 1 : pixels/5;
    if (pixels == 0) {
	stickPos = 1;
	bars = 0;
    }

    lcd.setCursor(0, row);

    // Char 0xb0 is a horizontal line; chars 0..4 have a vertical bar
    // at the i'th position.
    for (int i = 0; i < bars; i++)
        lcd.write(0xb0);
    lcd.write(stickPos);
    clearLine(bars);
}

void showNumeric(int row, int d) {
    lcd.setCursor(0, row);
    streamPrintf(lcd,  "%.3fm", d/1000.0);
}

#define NUM_CHARS 7
char blankLine[lcdCols+1];

void lcdInit(int cols, int rows) {
    lcd.begin(cols, rows);

    byte barChars[NUM_CHARS][8] = {
        { 0x01,0x01,0x01,0x1f,0x01,0x01,0x01 }, // vertical bar col 5
        { 0x10,0x10,0x10,0x10,0x10,0x10,0x10 }, // vertical bar col 1
        { 0x08,0x08,0x08,0x18,0x08,0x08,0x08 }, // vertical bar col 2
        { 0x04,0x04,0x04,0x1c,0x04,0x04,0x04 }, // vertical bar col 3
        { 0x02,0x02,0x02,0x1e,0x02,0x02,0x02 }, // vertical bar col 4
        { 0x04,0x0e,0x15,0x04,0x04,0x04,0x04 }, // up arrow
        { 0x04,0x04,0x04,0x04,0x15,0x0e,0x04 }, // down arrow
    };
    for (uint8_t i = 0; i < NUM_CHARS; i++)
        lcd.createChar(i, barChars[i]);

    // Possibly too much machinery to be resolution-independent....
    memset(blankLine, ' ', cols);
    blankLine[cols] = 0;
}

void clearLine(int pos) {
    lcd.print(blankLine+pos);
}

void error(const char* fmt, ...) {
    char buf[lcdCols+1];

    va_list args;
    va_start(args, fmt);
    (void) vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    buf[lcdCols] = 0;

    lcd.setCursor(0, 3);
    streamPrintf(lcd, "%-20s", buf);
}

void panic(const char* msg) {
    if (verbose)
        Serial.println(msg);
    lcd.setCursor(0, 1);
    lcd.print(msg);
    for (;;)
        ;
}
