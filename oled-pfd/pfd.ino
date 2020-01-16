//
// OLED stuff, including the PFD.

const char* cardinalLabels[] =
    { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
      "N" };

const int centerX = oledWidth/2;
const int centerY = oledHeight/2;

const float Deg2Rad = M_PI / 180;
const float pxPerDeg = oledHeight / (2*pitchFullScale);
const int cardinalPitch = int(22.5 * pxPerDeg);

bool oledSplash = true; // To show the spash screen for a bit

void transform(float (*M)[2][3], point* p1, point* p2);

void oledInit() {
#if SSD1325
    oled.begin();
#else
    if (!oled.begin(SSD1306_SWITCHCAPVCC))
	panic("Could not initialize OLED");
#endif
    oled.display();
    oled.setTextColor(WHITE);
    oled.setTextWrap(false);
}

void showPFD(float g, sensors_vec_t *orient, sensors_vec_t *mag, sensors_vec_t *accel,
	     int alt) {
    if (oledSplash) {
        if (millis() > 2000) {
            oledSplash = false;
            oled.clearDisplay();
            oled.display();
        }
        return;
    }

    oled.clearDisplay();

    // Transformation matrix: rotation by orient->y and xlation by orient->z.
    float M[2][3];
    getTransform(M, orient->y, orient->z);

    // Render the moving card.
    ladderLine(M, 0, 75);
    ladderLine(M, 2, 10);
    ladderLine(M, 4, 15);
    ladderLine(M, 6, 20);

    // Draw the fixed parts of the display.
    drawWing(0, 20, 4);
    drawWing(oledWidth-20, 20, 4);
    drawNose(10, 4);

    // Draw the heading and scales
    drawHeading(heading(mag->x, mag->y, mag->z));

    // Draw altitude and altimeter setting.
    drawAlt(alt);

    // All done.
    oled.display();
}

// Draw a line of the pitch ladder.
void ladderLine(float M[2][3], int ypos, int width) {
    point pLeft, pRight;
    point p0;
    pLeft.x = -width; pRight.x = width;
    pLeft.y = pRight.y = ypos * pxPerDeg;
    transformDraw(M, &pLeft, &pRight);

    p0.x = pLeft.x; p0.y = pLeft.y - 2;
    transformDraw(M, &pLeft, &p0);
    p0.x = pRight.x;
    transformDraw(M, &pRight, &p0);

    pLeft.y = -pLeft.y; pRight.y = -pRight.y;
    transformDraw(M, &pLeft, &pRight);
 
    p0.x = pLeft.x; p0.y = pLeft.y + 2;
    transformDraw(M, &pLeft, &p0);
    p0.x = pRight.x;
    transformDraw(M, &pRight, &p0);
}

void drawAlt(int alt) {
    oled.fillRect(0, oledHeight-9, 40, 9, BLACK);
    oled.setCursor(0, oledHeight-9);
    streamPrintf(oled, "%.2f", qnh);

    int len = snprintf(longbuf, sizeof(longbuf), "%d", alt);
    oled.fillRect(oledWidth - len*6 - 1, oledHeight-9, len*6+1, 9, BLACK);
    oled.setCursor(oledWidth - len*6, oledHeight-9);
    oled.print(longbuf);
}

void drawHeading(int hdg) {
    // Clear the top line
    oled.fillRect(0, 0, oledWidth, 8, BLACK);

    // Some dots at 10° intervals. Find the closest dot, and draw out from
    // there (the background for the label will be erased).
    int dotPos = (closestMult(hdg, 36, NULL) - hdg) * pxPerDeg + centerX;
    int dotPitch = 5 * pxPerDeg;
    for (int i = 1; i < 4; i++) {
        oled.drawPixel(dotPos + dotPitch * i, 4, WHITE);
        oled.drawPixel(dotPos - dotPitch * i, 4, WHITE);
    }

    // There are 16 named cardinal directions.
    int closest;
    int closestDeg = closestMult(hdg, 16, &closest);
    int labelPos = (closestDeg - hdg) * pxPerDeg + centerX;
    showCardinal(labelPos, closest);
    showOffsetCardinal(labelPos, closest, 1);
    showOffsetCardinal(labelPos, closest, -1);

    // Font is 5x8, pitch is 6 px    
    oled.fillRect(centerX-10, 0, 20, 8, BLACK);
    oled.setCursor(centerX-9, 0);
    streamPrintf(oled, "%03d", hdg);
}

void showCardinal(int pos, int idx) {
    const char* label = cardinalLabels[idx];
    if (strlen(label) > 1)
        pos--;
    oled.setCursor(pos, 0);
    oled.print(label);
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
    oled.fillRect(lpos, y, width, thickness, BLACK);
    oled.drawRect(lpos, y, width, thickness, WHITE);
}

void drawNose(int height, int thickness) {
    // The "nose" is an inverted V that fits in the HxW box, with its point
    // at screen center. The aspect ratio of the box is 4 i.e. a 2:1 slope.

    int y1 = centerY;
    int y2 = centerY + height;
    int x2L = centerX - height*2;
    int x2R = centerX + height*2;
    for (int i = 0; i < thickness; i++) {
        oled.drawLine(centerX-1, y1, x2L, y2, WHITE);
        oled.drawLine(centerX, y1, x2R, y2, WHITE);
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
    M[0][2] = oledWidth/2.0;

    M[1][0] = -sinRoll;
    M[1][1] = cosRoll;
    M[1][2] = oledHeight/2.0 - pxPerDeg * pitch;
}

void transformDraw(float M[2][3], point* p1, point* p2) {
    point p1t, p2t;
    transform(M, p1, &p1t);
    transform(M, p2, &p2t);
    oled.drawLine(p1t.x, p1t.y, p2t.x, p2t.y, WHITE);
}

void transform(float M[2][3], point* p1, point* p2) {
    p2->x = (int16_t)(p1->x * M[0][0] + p1->y * M[0][1] + M[0][2]);
    p2->y = (int16_t)(p1->x * M[1][0] + p1->y * M[1][1] + M[1][2]);
}
