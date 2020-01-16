
// 1013.25 mb = 29.92" of Hg
const float inHg2mb = 1013.25/29.92;
const float m2feet = 1000 / (12 * 25.4);

void bnoInit() {
    if (!bno.begin())
        panic("No BNO055 found.");
}

void bmpInit() {
    if (!bmp.begin())
        panic("No BMP085 found.");
}

void adxlInit() {
    if (!accel.begin())
	panic("No ADXL345 found.");
}

// Pitch and roll offsets for the IMU.
float pitchOffset = 0.0;
float rollOffset = 0.0;

void showSensorValues() {
    sensors_event_t baroEvt; bmp.getEvent(&baroEvt);
    sensors_event_t accelEvt; accel.getEvent(&accelEvt);
    float temp; bmp.getTemperature(&temp);
    int alt = bmp.pressureToAltitude(qnh * inHg2mb,
				     baroEvt.pressure,
				     temp) * m2feet;
    sensors_event_t imuEvt; bno.getEvent(&imuEvt);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    // Add ADXL....

    // We know the temperature and pressure, we should also show density alt.

    if (resetLevel) {
        pitchOffset = imuEvt.orientation.z;
        rollOffset = imuEvt.orientation.y;
    }
    resetLevel = false;
    imuEvt.orientation.z -= pitchOffset;
    imuEvt.orientation.y -= rollOffset;

    showPFD(grav.z(), &imuEvt.orientation, &imuEvt.magnetic, &accelEvt.acceleration,
	    alt);

    showOrient(&imuEvt.orientation);
    showPhysical(&imuEvt.magnetic);
    showAirData(temp, baroEvt.pressure, alt);
}

// Show heading.
void showPhysical(sensors_vec_t *mag) {
    if (!verbose) return;
    streamPrintf(Serial, "Hdg %03d\xdf   QNH %.2f",
		 heading(mag->x, mag->y, mag->z), qnh);
}

// Fetch pressure and temperature.
void showAirData(float temp, float pressure, float alt) {
    if (!verbose) return;
    float press_inHg = pressure * 29.92 / 1013.25;
    streamPrintf(Serial, "%.2f\"Hg %.1fC %.1f'", press_inHg, temp, alt);
}

// Fetch pressure and temperature.
void showOrient(sensors_vec_t *orient) {
    if (!verbose) return;

    char rollDir = ' ', pitchDir = ' ';
    if (orient->y < 0) {
        rollDir = '>';
        orient->y = -orient->y;
    } else if (orient->y > 0) {
        rollDir = '<';
    }
    if (orient->z < 0) {
        pitchDir = 'v';
        orient->z = -orient->z;
    } else if (orient->z > 0) {
        pitchDir = '^';
    }
    streamPrintf(Serial, "Orientation: %c%.1f%c %c%.1f%c",
		 rollDir, orient->y, rollDir,
		 pitchDir, orient->z, pitchDir);
}
