
// 1013.25 mb = 29.92" of Hg
const float inHg2mb = 1013.25/29.92;
const float m2feet = 1000 / (12 * 25.4);

void vl53L1XInit() {
    if (dist_sensor.begin() != 0)
        panic("No VL53L1X found.");
}

void bnoInit() {
    if (!bno.begin())
        panic("No BNO055 found.");
}

void bmpInit() {
    if (!bmp.begin())
        panic("No BMP085 found.");
}

// Pitch and roll offsets for the IMU.
float pitchOffset = 0.0;
float rollOffset = 0.0;

void showSensorValues() {
    int dist = getDistance();
    sensors_event_t baroEvt; bmp.getEvent(&baroEvt);
    float temp; bmp.getTemperature(&temp);
    int alt = bmp.pressureToAltitude(qnh * inHg2mb,
				     baroEvt.pressure,
				     temp) * m2feet;
    sensors_event_t imuEvt; bno.getEvent(&imuEvt);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    // We know the temperature and pressure, we should also show density alt.

    if (resetLevel) {
        pitchOffset = imuEvt.orientation.z;
        rollOffset = imuEvt.orientation.y;
    }
    resetLevel = false;
    imuEvt.orientation.z -= pitchOffset;
    imuEvt.orientation.y -= rollOffset;

    showPFD(grav.z(), &imuEvt.orientation, &imuEvt.magnetic, alt);

    showBargraph(0, dist);
    showOrient(1, &imuEvt.orientation);
    showPhysical(2, &imuEvt.magnetic);
    showAirData(3, temp, baroEvt.pressure, alt);
}

// Get the distance from the sensor.
int getDistance() {
    dist_sensor.startRanging();
    int d = dist_sensor.getDistance();
    dist_sensor.stopRanging();
    return d;
}

void showBargraph(int row, int d) {
    blankRow(row);

    // 100 pixels across = 4m i.e. 40mm per pixel.
    byte pixels = (byte)(d / 40);
    byte stickPos = pixels % 5;
    byte bars = (stickPos == 0) ? pixels/5 - 1 : pixels/5;

    // Char 0xb0 is a horizontal line; chars 0..4 have a vertical bar
    // at the i'th position.
    for (int i = 0; i < bars; i++)
        lcd.write(0xb0);
    lcd.write(stickPos);

    int len = snprintf(longbuf, sizeof(longbuf), "%.2fm", d/1000.0);
    if (pixels > 50) {
        lcd.setCursor(0, row);
    } else {
        lcd.setCursor(20-len, row);
    }
    lcd.print(longbuf);
}

// Show the distance reported by the ToF sensor.
void showPhysical(int row, sensors_vec_t *mag) {
    blankRow(row);
    streamPrintf(lcd, "Hdg %03d\xdf   QNH %.2f",
		 heading(mag->x, mag->y, mag->z), qnh);

    if (verbose)
	streamPrintf(Serial, "Hdg %03d\xdf   QNH %.2f",
		     heading(mag->x, mag->y, mag->z), qnh);
}

// Fetch pressure and temperature.
void showAirData(int row, float temp, float pressure, float alt) {
    float press_inHg = pressure * 29.92 / 1013.25;
    blankRow(row);
    streamPrintf(lcd, "%.2f\" %.1fC %.1f'", press_inHg, temp, alt);

    if (verbose)
        streamPrintf(Serial, "%.2f\"Hg %.1fC %.1f'", press_inHg, temp, alt);
}

// Fetch pressure and temperature.
void showOrient(int row, sensors_vec_t *orient) {
    char rollDir = ' ', pitchDir = ' ';
    if (orient->y < 0) {
        rollDir = 0x7e; // right arrow
        orient->y = -orient->y;
    } else if (orient->y > 0) {
        rollDir = 0x7f; // left arrow
    }
    if (orient->z < 0) {
        pitchDir = 6; // down arrow
        orient->z = -orient->z;
    } else if (orient->z > 0) {
        pitchDir = 5; // up arrow
    }

    blankRow(row);
    streamPrintf(lcd, "LVL %c%.1f%c %c%.1f%c",
		 rollDir, orient->y, rollDir,
		 pitchDir, orient->z, pitchDir);

    if (verbose)
	streamPrintf(Serial, "Orientation: %c%.1f%c %c%.1f%c",
		     rollDir, orient->y, rollDir,
		     pitchDir, orient->z, pitchDir);
}
