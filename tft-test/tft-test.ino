// TFT test program
// Shamim Mohamed 2020-01-14

// Use https://github.com/PaulStoffregen/ILI9341_t3 instead?
#define USE_T3

#include <SPI.h>
#include <Wire.h>

#ifdef USE_T3
#include <ILI9341_t3.h>
#else
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#endif

const int tftWidth = 320;
const int tftHeight = 240;

const int tftDC = 5;
const int tftCS = 10;
const int tftSDCS = 4;
const int tftReset = 6;

// HW SPI on the Teensy 4.0:
// SCLK = 13
// MISO = 12
// MOSI = 11

// The backlight pin on the TFT
const int8_t tftBacklight = 15;

// The TFT panel, using HW SPI.
#ifdef USE_T3
ILI9341_t3 tft = ILI9341_t3(tftCS, tftDC);
#else
Adafruit_ILI9341 tft = Adafruit_ILI9341(tftCS, tftDC);
#endif

const int tftFG = ILI9341_ORANGE;
const int tftBG = ILI9341_WHITE;
const int tftTXT = ILI9341_DARKGREEN;

void setup() {
    digitalWrite(tftReset, LOW);
    delay(1);
    digitalWrite(tftReset, HIGH);
    digitalWrite(tftBacklight, HIGH);

    tft.begin();
    tft.fillScreen(ILI9341_BLACK);

    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Started.");

    // read diagnostics (optional but can help debug problems)
    uint8_t x = tft.readcommand8(ILI9341_RDMODE);
    Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDMADCTL);
    Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDPIXFMT);
    Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDIMGFMT);
    Serial.print("Image Format: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDSELFDIAG);
    Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); 

    tft.setTextColor(tftTXT);
    tft.setTextWrap(false);

    tft.fillRect(5, 5, 100, 100, tftBG);
    tft.drawRect(5, 5, 100, 100, tftFG);
    tft.drawLine(5, 5, 100, 100, tftFG);

    tft.setCursor(6, 90);
    tft.print("Hello, world!");
}

void loop() {}

