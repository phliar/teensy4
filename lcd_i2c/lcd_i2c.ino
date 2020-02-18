// Init LCD program
// Shamim Mohamed 2019-12-24

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// I2C addresses
const uint8_t lcd_addr = 0x3f;
const uint8_t tof_addr = 0x29;

// Fake params for the i2c adapter
const int bl=3, en=2, rs=0, rw=1, d4=4, d5=5, d6=6, d7=7;

LiquidCrystal_I2C lcd(lcd_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("Started.");
  lcd.begin(16, 2);
  
  lcd.print("Hello,");
  lcd.setCursor(0, 1);
  lcd.print("       world!");
  Serial.println("Hello, World!");
  delay(1000);
}

void loop() {
}
