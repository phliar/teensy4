
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <Phliar_utils.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1325.h>

#include <Wire.h>

// Motor driver
#include "SCMD.h"
#include "SCMD_config.h"
const uint8_t SCMD_I2C_ADDR = 0x58; // jumpers "0011" on SCMD
const uint8_t SCMD_FRWD = 0;
const uint8_t SCMD_BKWD = 1;
SCMD motors;

// Encoder
const int countsPerRev = 99 * 48; // Pololu 25D 99:1 gear ratio, 48 CPR encoder
const uint8_t pinEnc0a = 20;
const uint8_t pinEnc0b = 21;
const uint8_t pinEnc1a = 22;
const uint8_t pinEnc1b = 23;
const uint8_t motor0 = 0, motor1 = 1;
const uint8_t encoderPins[2][2] =
    {
     {pinEnc0a, pinEnc0b},
     {pinEnc1a, pinEnc1b}
    };

// Updated by the interrupt handlers
volatile int counts[2];
volatile float speeds[2]; // RPM

const uint8_t displayCS = 10;
const uint8_t displayRST = 6;
const uint8_t displayDC = 5;
Adafruit_SSD1325 oled(displayDC, displayRST, displayCS);
const int FONT_W = 6;
const int FONT_H = 8;
int OLED_W;
int OLED_H;
int OLED_COLS;
int OLED_ROWS;

const uint8_t pinMonitor = 0;
bool verbose;

const uint8_t DRIVE_LEVEL = 128; // 32 seems to be the least usable value with 5V
const int DURATION_MS = 10000;
const int INTERVAL_MS = 500;

#define LEDPIN LED_BUILTIN

void setup() {
    oledInit();
    serialInit();
    initMotors();
    initInterrupts();
    if (verbose)
        Serial.println("Initialization complete.");
    loop1();
}

void loop1() {
    oled.clearDisplay();
    showData();
    oled.display();

    for (int i = 0; i < 2; i++) {
	drive(i, SCMD_FRWD, DRIVE_LEVEL);
	showData();
	delay(INTERVAL_MS);

	drive(i, SCMD_BKWD, DRIVE_LEVEL);
	showData();
	delay(INTERVAL_MS);
    }
}
void loop() {}

void showData() {
    for (int motor = 0; motor < 2; motor++) {
	oledLine(motor*2, "%d", counts[motor]);
	oledLine(motor*2 + 1, "%.1f/s", speeds[motor]);
    }
    oled.display();
}

void drive(uint8_t motor, uint8_t dir, uint8_t level) {
    digitalWrite(LEDPIN, HIGH);
    motors.setDrive(motor, dir, level);

    delay(DURATION_MS);

    digitalWrite(LEDPIN, LOW);
    motors.setDrive(motor, dir, 0);
}

/*
 * Initialization
 */

void serialInit() {
    pinMode(pinMonitor, OUTPUT); // For timing sections of code
    Serial.begin(115200);
    delay(1500);
    verbose = !!Serial;

    if (verbose)
        Serial.println("BEGIN");
}

void oledInit() {
    oled.begin();
    oled.display();
    oled.setRotation(3);
    OLED_W = oled.width();
    OLED_H = oled.height();
    OLED_COLS = OLED_W / FONT_W;
    OLED_ROWS = OLED_H / FONT_H;
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
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
 * Encoder interrupt handlers
 */

void initInterrupts() {
    pinMode(pinEnc0a, INPUT);
    attachInterrupt(digitalPinToInterrupt(pinEnc0a), handler0, CHANGE);
    pinMode(pinEnc0b, INPUT);
    pinMode(pinEnc1a, INPUT);
    attachInterrupt(digitalPinToInterrupt(pinEnc1a), handler1, CHANGE);
    pinMode(pinEnc1b, INPUT);
}

void handler0() {
    digitalWrite(pinMonitor, HIGH);
    encoderISR(motor0,
	       digitalRead(encoderPins[motor0][0]),
	       digitalRead(encoderPins[motor0][1]));
    digitalWrite(pinMonitor, LOW);
}

void handler1() {
    digitalWrite(pinMonitor, HIGH);
    encoderISR(motor1,
	       digitalRead(encoderPins[motor1][0]),
	       digitalRead(encoderPins[motor1][1]));
    digitalWrite(pinMonitor, LOW);
}

const int filterDepth = 10;     // Get average speed over the last N counts
int positions[2][filterDepth];
int timestamps[2][filterDepth];
int idx[2];

void resetDistances() {
    for (int m = 0; m < 2; m++) {
	counts[m] = 0;
	idx[m] = 0;
	speeds[m] = 0.0;
	for (int i = 0; i < filterDepth; i++)
	    timestamps[m][i] = 0;
    }
}

void encoderISR(uint8_t motor, int i, int q) {
    if (i == q)
	counts[motor]++;
    else
	counts[motor]--;

    int now = micros();
    int startTime = timestamps[motor][idx[motor]];
    int startPos = positions[motor][idx[motor]];
    int elapsed = now - startTime;
    int distance = counts[motor] - startPos;
    positions[motor][idx[motor]] = counts[motor];
    timestamps[motor][idx[motor]] = now;
    idx[motor] = (idx[motor] + 1) % filterDepth;

    // Two interrupts per count
    //     speeds[motor] = 60 * 1.0e6/(2.0 * filterDepth * elapsed * countsPerRev);
    speeds[motor] = 1.0e6 * distance/(2.0 * elapsed);
    if (idx[motor] == 0)
	showData();
}

/*
 * Utils
 */

void oledLine(int line, const char* fmt, ...) {
    char buf[OLED_COLS+1];

    va_list args;
    va_start(args, fmt);
    (void) vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    buf[OLED_COLS] = 0;

    oled.fillRect(0, line * FONT_H, OLED_W, FONT_H, BLACK);
    oled.setCursor(0, line * FONT_H);
    streamPrintf(oled, "%10s", buf);
}

void panic(const char* msg) {
    if (verbose)
        Serial.println(msg);
    oledLine(3, "%s", msg);
    for (;;)
        ;
}
