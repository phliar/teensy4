
// Distance measurement with the VL53L1X ToF sensor.
// Shamim Mohamed 2019-12-24

#include <stdio.h>
#include <math.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

#include <SparkFun_VL53L1X.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP085_U.h>

bool verbose = true;

// I2C addresses
const uint8_t lcd_addr = 0x3f;
const uint8_t vl53_addr = 0x29; // Not actually needed.
const uint8_t bn0055_addr = 0x28;
const uint8_t bmp085_addr = 0x77;

// Pin assignments for the i2c adapter
const int bl=3 /* backlight */, en=2 /*enable*/, rs=0 /*reset*/, rw=1 /*read-write*/, d4=4, d5=5, d6=6, d7=7;

// For printing one line to the LCD which has 20 cols.
char buffer[21];

// The LCD display.
LiquidCrystal_I2C lcd(lcd_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE /* backlight polarity */);

// The ToF sensor.
SFEVL53L1X dist_sensor(Wire); // for SCL0/SDA0 -- or Wire1, to use SCL1/SDA1

// The Baro sensor.
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// The IMU.
Adafruit_BNO055 bno = Adafruit_BNO055(55, bn0055_addr);

void setup() {
  // Set up the serial console.
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Started.");

  // Set up the LCD panel.
  lcd.begin(20, 4);

  // Set up the VL53L1X.
  if (dist_sensor.begin() != 0) {
    panic("No VL53L1X found.");
  }
  Serial.println("VL53L1X online.");

  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    panic("No BNO055 found.");
  }
  Serial.println("BNO055 online.");

  if (!bmp.begin()) {
    Serial.print("No BMP085 detected.");
    panic("No BMP085 found.");
  }
  Serial.println("BMP085 online.");

  // A timer interrupt every 100ms.
  Timer1.initialize(100000);
  Timer1.attachInterrupt(handler);

  once(); // Still in verbose, this will also print to console.
  verbose = false;
}

volatile bool ding = false;
void handler() {
  ding = true;
}

void loop() {
  // No need to disable interrupts since ding is one byte.
  if (ding) {
    ding = false;
    once();
  }
}

void once() {
  lcd.setCursor(0, 1);
  lcd.print(getDistanceLine());

  lcd.setCursor(0, 2);
  lcd.print(getBaroLine());

  lcd.setCursor(0, 3);
  lcd.print(getEulerAnglesLine());
}

// Get the distance from the sensor.
const char * getDistanceLine() {
  dist_sensor.startRanging();
  int d = dist_sensor.getDistance();
  dist_sensor.stopRanging();

  sensors_event_t event; 
  bno.getEvent(&event);
  snprintf(buffer, sizeof(buffer), "Dist %.3fm  HDG %03d", d/1000.0,
    heading(event.magnetic.x, event.magnetic.y, event.magnetic.z));
  if (verbose)
    Serial.println(buffer);
  return buffer;
}

// Fetch pressure and temperature.
const char* getBaroLine() {
  sensors_event_t event;
  bmp.getEvent(&event);
  
  float pressure = -1;
  if (event.pressure) {
    pressure = event.pressure;
  }

  float temp;
  bmp.getTemperature(&temp);
  int alt = int(bmp.pressureToAltitude(1013.25, pressure, temp)  + 0.5);
  snprintf(buffer, sizeof(buffer), "%.1fhPa %.1fC %dm", pressure, temp, alt);
  if (verbose)
    Serial.println(buffer);
  return buffer;
}

// Get the euler angles from the IMU.
const char* getEulerAnglesLine() {
  sensors_event_t evt;
  bno.getEvent(&evt);

  memset(buffer, ' ', sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "X %.1f %.1f %.1f",
    evt.orientation.x, evt.orientation.y, evt.orientation.z);
  if (verbose)
    Serial.println(buffer);
  return buffer;
}

uint16_t heading(float hx, float hy, float hz) {
  // Doc says mag units are µT but it looks like they're degrees???
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
