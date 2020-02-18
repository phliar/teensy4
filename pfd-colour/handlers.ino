
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

// Pitch and roll offsets for the IMU.
float pitchOffset = 0.0;
float rollOffset = 0.0;

void showSensorValues() {
    sensors_event_t baroEvt; bmp.getEvent(&baroEvt);
    float temp; bmp.getTemperature(&temp);
    int alt = bmp.pressureToAltitude(qnh * inHg2mb,
				     baroEvt.pressure,
				     temp) * m2feet;
    sensors_event_t imuEvt; bno.getEvent(&imuEvt);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    // We know the temperature and pressure, we should also show density alt.
    showAirData(temp, baroEvt.pressure, alt);

    if (resetLevel) {
        pitchOffset = imuEvt.orientation.z;
        rollOffset = imuEvt.orientation.y;
    }
    resetLevel = false;
    imuEvt.orientation.z -= pitchOffset;
    imuEvt.orientation.y -= rollOffset;

    if (verbose)
	streamPrintf(Serial, "O(%.1f %.1f %.1f) Mag(%.1f %.1f %.1f) G(%.1f %.1f %.1f)\n",
		     imuEvt.orientation.x, imuEvt.orientation.y, imuEvt.orientation.z, 
		     imuEvt.magnetic.x, imuEvt.magnetic.y, imuEvt.magnetic.z,
		     grav.x(), grav.y(), grav.z());
    showPFD(grav.x(), grav.y(), grav.z(), &imuEvt.orientation, &imuEvt.magnetic, alt);
}

// Fetch pressure and temperature.
void showAirData(float temp, float pressure, int alt) {
    if (!verbose)
	return;

    float press_inHg = pressure * 29.92 / 1013.25;
    streamPrintf(Serial, "%.2f\"Hg %.1fC %d'\n", press_inHg, temp, alt);
}
