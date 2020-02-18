#define SMOOTHING 3

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <Phliar_utils.h>

#include <TimerOne.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1325.h>

#include <Wire.h>

#include "motor.h"
motor_t motors[2];

// Motor driver
#include "SCMD.h"
#include "SCMD_config.h"
const uint8_t SCMD_I2C_ADDR = 0x58; // jumpers "0011" on SCMD
const uint8_t SCMD_FRWD = 0;
const uint8_t SCMD_BKWD = 1;
SCMD motorCtl;

// Encoder
const int countsPerRev = 99 * 48; // Pololu 25D 99:1 gear ratio, 48 CPR encoder
const uint8_t pinEncLa = 20;
const uint8_t pinEncLb = 21;
const uint8_t pinEncRa = 22;
const uint8_t pinEncRb = 23;
const uint8_t encoderPins[2][2] =
    {
     {pinEncLa, pinEncLb},
     {pinEncRa, pinEncRb}
    };

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

// While we're trying to figure out the coefficients
bool shouldRun = false;
const uint8_t pinStartStop = 17;
const uint8_t pinSpeed = 14;
const uint8_t pinKp = 16;
const uint8_t pinKi = 15;
const float fullScaleKp = 1.0;
const float fullScaleKi = 0.1;


const uint8_t pinMonitor = 0;
bool verbose;

const int refreshRate = 10; // per second

const uint8_t START_DRIVE_LEVEL = 128;
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
    oled.clearDisplay();

    // Update displays
    showData();
    if (!shouldRun)
        stopRun(&motors[0]);
}

void loop() {
    motor_t* m = &motors[0];
    if (shouldRun) {
        digitalWrite(LEDPIN, HIGH);
        m->targetSpeed = 800.0 * analogRead(pinSpeed) / 4095.0;
        if (!m->isRunning()) {
            oled.clearDisplay();
            startRun(m);
        }
    } else {
        digitalWrite(LEDPIN, LOW);
        m->targetSpeed = 0.0;
        if (m->isRunning())
            stopRun(m);
    }
}

void startRun(motor_t* m) {
    if (verbose)
        Serial.println("STARTING");
    Timer1.attachInterrupt(runPID);
    m->driveLevel = START_DRIVE_LEVEL; // possibly negative if reverse
    m->start();
}

void stopRun(motor_t* m) {
    Timer1.detachInterrupt();
    m->stop();
    m->driveLevel = 0;
    showData();
    if (verbose)
        Serial.println("STOPPED");
}

void showData() {
    motor_t* m = &motors[0];
    const int N = 7;
    for (int i = 0; i < 1; m = &motors[++i]) {
        oledRLine(i*N, "%d", m->count);
        oledLine(i*N + 1, "%.1f/s:", m->targetSpeed);
        oledRLine(i*N + 2, "%.1f/s", m->speed);
        oledLine(i*N + 3, "P = %d", m->driveLevel);
        oledLine(i*N + 4, "Kp = %.3f", m->kp);
        oledLine(i*N + 5, "Ki = %.3f", m->ki);
        oledLine(i*N + 6, "-- %s", m->isRunning()? "running" : "stopped");
    }
    oled.display();
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

void initInterrupts() {
    pinMode(pinEncLa, INPUT);
    pinMode(pinEncLb, INPUT);
    pinMode(pinEncRa, INPUT);
    pinMode(pinEncRb, INPUT);
    attachInterrupt(digitalPinToInterrupt(pinEncLa), motorLhandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncRa), motorRhandler, CHANGE);

    Timer1.initialize(1e6/refreshRate);

    pinMode(pinStartStop, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinStartStop), startStop, FALLING);

    pinMode(pinSpeed, INPUT);
    pinMode(pinKp, INPUT);
    pinMode(pinKi, INPUT);
    analogReadRes(12); // Use 12-bit DAC reads
}

void startStop() {
    static int lastPress;
    int now = millis();
    if (now - lastPress < 500)
        return;
    lastPress = now;
    shouldRun = !shouldRun;
    if (verbose)
        Serial.println(shouldRun? "B: STOPPED" : "B: RUNNING");
    loop(); // shouldn't be needed, but....
}

/*
 * Utils
 */

void oledRLine(int line, const char* fmt, ...) {
    char buf[OLED_COLS+1];

    va_list args;
    va_start(args, fmt);
    (void) vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    buf[OLED_COLS] = 0;

    oled.fillRect(0, line * FONT_H, OLED_W, FONT_H, BLACK);
    oled.setCursor(0, line * FONT_H);
    streamPrintf(oled, "%10s", buf);
    if (verbose) {
        streamPrintf(Serial, "%10s\n", buf);
    }
}

void oledLine(int line, const char* fmt, ...) {
    char buf[OLED_COLS+1];

    va_list args;
    va_start(args, fmt);
    (void) vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    buf[OLED_COLS] = 0;

    oled.fillRect(0, line * FONT_H, OLED_W, FONT_H, BLACK);
    oled.setCursor(0, line * FONT_H);
    streamPrintf(oled, "%-10s", buf);
    if (verbose) {
        streamPrintf(Serial, "%-10s\n", buf);
    }
}

void panic(const char* msg) {
    if (verbose)
        Serial.println(msg);
    oledLine(13, "%s", msg);
    for (;;)
        ;
}
