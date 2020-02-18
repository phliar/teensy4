//
// TFT display stuff, including the PFD.

// The PFD assumes it's rendering to a 2:1 rectangle. The 320x240 rectangle is
// split into a 320x160 PFD and two 320x40 info strips.
const int tftWidth = 320;
const int tftHeight = 240;
const int tftInfoHeight = 40;
const int tftPFDYPos = tftInfoHeight;
const int tftPFDHeight = 160;
const int tftAltYPos = tftPFDYPos + tftPFDHeight;

const int tftBG = ILI9341_BLACK;
const int tftInfoFG = ILI9341_WHITE;
const int tftInfoBG = ILI9341_BLACK;
const int tftFixedPlaneBG = ILI9341_BLACK;
const int tftFixedPlaneFG = ILI9341_ORANGE;
const int tftLadderFG = ILI9341_RED;

const char* cardinalLabels[] =
    { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
      "N" };

const int centerX = tftWidth/2;
const int centerY = tftPFDHeight/2 + tftPFDYPos;

const float Deg2Rad = M_PI / 180;
const float pxPerDeg = tftPFDHeight / (2*pitchFullScale);
const int cardinalPitch = int(22.5 * pxPerDeg);

bool tftSplash = true; // To show the spash screen for a bit.

void transform(float (*M)[2][3], point* p1, point* p2);

void tftInit() {
    tft.begin();
    // A short delay may now be needed.
}

void showPFD(float gx, float gy, float gz, sensors_vec_t *orient, sensors_vec_t *mag,
	     int alt) {
    if (tftSplash) {
	// No splash screen on the ILI, but set up some defaults.
	tft.setTextColor(tftInfoFG);
	tft.setTextWrap(false);
	tftSplash = false;
    }
    tft.fillScreen(tftBG);

    // Transformation matrix: rotation by orient->y and xlation by orient->z.
    float M[2][3];
    getTransform(M, orient->y, orient->z);

    // Draw sky and ground
    // ...

    // Render the moving card.
    ladderLine(M, 0, 75);
    ladderLine(M, 2, 10);
    ladderLine(M, 4, 15);
    ladderLine(M, 6, 20);

    // Draw the fixed "airplane" at centerY.
    drawWing(0, 20, 4);
    drawWing(tftWidth-20, 20, 4);
    drawNose(10, 4);

    // Draw the heading and scales at 0.
    //drawHeading(heading(mag->x, mag->y, mag->z));

    // Draw altitude and altimeter setting on the lower info strip.
    //drawAlt(alt);

    // All done.
}

// Draw a line of the pitch ladder.
void ladderLine(float M[2][3], int yDeg, int width) {
    point pLeft, pRight;
    point p0;
    pLeft.x = -width; pRight.x = width;
    pLeft.y = pRight.y = yDeg * pxPerDeg;
    transformDrawLine(M, &pLeft, &pRight);

    p0.x = pLeft.x; p0.y = pLeft.y - 2;
    transformDrawLine(M, &pLeft, &p0);
    p0.x = pRight.x;
    transformDrawLine(M, &pRight, &p0);

    pLeft.y = -pLeft.y; pRight.y = -pRight.y;
    transformDrawLine(M, &pLeft, &pRight);
 
    p0.x = pLeft.x; p0.y = pLeft.y + 2;
    transformDrawLine(M, &pLeft, &p0);
    p0.x = pRight.x;
    transformDrawLine(M, &pRight, &p0);
}

void drawAlt(int alt) {
    tft.fillRect(0, tftAltYPos, 40, tftInfoHeight, tftInfoBG);
    tft.setCursor(0, tftAltYPos);
    streamPrintf(tft, "%.2f", qnh);

    int len = snprintf(longbuf, sizeof(longbuf), "%d", alt);
    tft.fillRect(tftWidth - len*6-1, tftAltYPos, len*6+1, tftInfoHeight, tftInfoBG);
    tft.setCursor(tftWidth - len*6, tftAltYPos);
    tft.print(longbuf);
}

void drawHeading(int hdg) {
    // Clear the top line
    tft.fillRect(0, 0, tftWidth, tftInfoHeight, tftInfoBG);

    // Some dots at 10° intervals. Find the closest dot, and draw out from
    // there (the background for the label will be erased).
    int dotPos = (closestMult(hdg, 36, NULL) - hdg) * pxPerDeg + centerX;
    int dotPitch = 5 * pxPerDeg;
    for (int i = 1; i < 4; i++) {
        tft.drawPixel(dotPos + dotPitch * i, tftInfoHeight/2, tftInfoFG);
        tft.drawPixel(dotPos - dotPitch * i, tftInfoHeight/2, tftInfoFG);
    }

    // There are 16 named cardinal directions.
    int closest;
    int closestDeg = closestMult(hdg, 16, &closest);
    int labelPos = (closestDeg - hdg) * pxPerDeg + centerX;
    showCardinal(labelPos, closest);
    showOffsetCardinal(labelPos, closest, 1);
    showOffsetCardinal(labelPos, closest, -1);

    // Font is 5x8, pitch is 6 px    
    tft.fillRect(centerX-10, 0, 20, tftInfoHeight, tftInfoBG);
    tft.setCursor(tftInfoHeight, 0);
    streamPrintf(tft, "%03d", hdg);
}

void showCardinal(int pos, int idx) {
    const char* label = cardinalLabels[idx];
    if (strlen(label) > 1)
        pos--;
    tft.setCursor(pos, 0);
    tft.print(label);
}

void showOffsetCardinal(int pos, int idx, int offset) {
    if (offset == 1) {
        if (++idx == 16)
            idx = 0;
        showCardinal(pos + cardinalPitch, idx);
    } else {
        if (--idx < 0)
            idx = 15;
        showCardinal(pos - cardinalPitch, idx);
    }
}

int closestMult(int deg, int steps, int* idx) {
    float div = 360.0/steps;
    int pos = int(deg/div + 0.5);
    if (idx)
        *idx = pos;
    return int(pos*div + 0.5);
}

// Draw a "wing": a vertically centered horizontal line that starts at lpos
// and goes for width pixels. The wing (a rectangle) is outlined by a 1-pixel
// line in the contrasting colour since it's a 1-bit display, so things need
// to be drawn in one colour and filled with another so they will show up
// against both ground and sky.
void drawWing(int lpos, int width, int thickness) {
    uint16_t y = centerY - thickness/2;
    tft.fillRect(lpos, y, width, thickness, tftFixedPlaneBG);
    tft.drawRect(lpos, y, width, thickness, tftFixedPlaneFG);
}

void drawNose(int height, int thickness) {
    // The "nose" is an inverted V that fits in the HxW box, with its point
    // at screen center. The aspect ratio of the box is 4 i.e. a 2:1 slope.

    int y1 = centerY;
    int y2 = centerY + height;
    int x2L = centerX - height*2;
    int x2R = centerX + height*2;
        for (int i = 0; i < thickness; i++) {
        tft.drawLine(centerX-1, y1, x2L, y2, tftFixedPlaneBG);
        tft.drawLine(centerX, y1, x2R, y2, tftFixedPlaneFG);
        y1++;
        x2L += 2; x2R -= 2;
    }
}

// A transformation matrix for a left-handed coordinate system.
void getTransform(float M[2][3], float roll, float pitch) {
    float radroll = roll * Deg2Rad;
    float cosRoll = cos(radroll);
    float sinRoll = sin(radroll);

    M[0][0] = cosRoll;
    M[0][1] = sinRoll;
    M[0][2] = tftWidth/2.0;

    M[1][0] = -sinRoll;
    M[1][1] = cosRoll;
    M[1][2] = tftPFDYPos + tftPFDHeight/2.0 - pxPerDeg * pitch;
}

void transformDrawLine(float M[2][3], point* p1, point* p2) {
    point p1t, p2t;
    transform(M, p1, &p1t);
    transform(M, p2, &p2t);
    //streamPrintf(Serial, "(%d, %d) -> (%d, %d) %08x\n", p1t.x, p1t.y, p2t.x, p2t.y, tftLadderFG);
    tft.drawLine(p1t.x, p1t.y, p2t.x, p2t.y, tftLadderFG);
}

void transform(float M[2][3], point* p1, point* p2) {
    p2->x = (int16_t)(p1->x * M[0][0] + p1->y * M[0][1] + M[0][2]);
    p2->y = (int16_t)(p1->x * M[1][0] + p1->y * M[1][1] + M[1][2]);
}
