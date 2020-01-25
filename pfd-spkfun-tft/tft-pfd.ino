//
// TFT display stuff, including the PFD.

// The PFD assumes it's rendering to a 2:1 rectangle. The  rectangle is
// split into a 160x80 PFD and two 160x24 info strips. (Here represented as
// landscape.)
const int paneWidth = 160;
const int paneHeight = 128;

// All drawing is done in "normalized" coordinates, with the origin at the
// center of the panel, and the board in landscape format with connectors to
// the left.
const int tftInfoHeight = 24;
const int tftPFDHeight = paneWidth/2; // 2:1 rectangle
const int tftAltYPos = paneHeight - tftInfoHeight;

const int PITCH_MIN = -30;
const int PITCH_MAX = 30;
const int FATLINE_W = 80;

const char* cardinalLabels[] =
    { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
      "N" };

const int charX = 6;
const int charY = 8;

// PFD vertical scale: 20 deg up, 20 deg down.
const float pitchFullScale = 20.0;

const float Deg2Rad = M_PI / 180;
const float pxPerDeg = tftPFDHeight / (2*pitchFullScale);
const int cardinalPitch = int(22.5 * pxPerDeg + 0.5);

// The TFT panel.
KWH018ST01_4WSPI tft;
ILI9163C_color_18_t fg, bg, ffg, fbg, lfg, sky, gnd;

color_t tftInfoFG = &fg;
color_t tftInfoBG = &bg;
color_t tftFixedPlaneBG = &fbg;
color_t tftFixedPlaneFG = &ffg;
color_t tftLadderFG = &lfg;
color_t tftSky = &sky;
color_t tftGround = &gnd;

bool tftSplash = true; // To show the spash screen for a bit.

point transform(float M[2][3], point p);

void tftInit() {
    tft.begin(DC_PIN, CS_PIN, PWM_PIN, SPI, SPI_SPEED);
    fg = tft.rgbTo18b(255, 255, 255);
    bg = tft.rgbTo18b(0, 0, 0);
    fbg = tft.rgbTo18b(255, 255, 255);
    ffg = tft.rgbTo18b(255, 127, 0);
    lfg = tft.rgbTo18b(255, 255, 0);
    sky = tft.rgbTo18b(100, 100, 255);
    gnd = tft.rgbTo18b(200, 50, 50);

    tft.clearDisplay();
}

// Graphics primitives

void drawLine(int x1, int y1, int x2, int y2, int width, color_t color) {
    if (verbose)
	streamPrintf(Serial, "(%d, %d) -> (%d, %d)\n", x1, y1, x2, y2);
    tft.line(x1, y1, x2, y2, width, color);
}

void drawString(int x, int y, int width, color_t fg, color_t bg, const char* txt) {
    if (bg != NULL)
 	tft.rectangle(x, y, x+width, y+charY, true, bg);
    if (fg != NULL)
	tft.setCurrentWindowColorSequence(fg);
    tft.setTextCursor(x, y);
    tft.print(txt);
}

void drawPixel(int x, int y, color_t color) {
    tft.pixel(x, y, color);
}

void drawRect(int x1, int y1, int x2, int y2, color_t color) {
    if (verbose)
	streamPrintf(Serial, "r (%d %d), %dx%d\n", x1, y1, x2, y2);
    tft.rectangle(x1, y1, x2, y2, false, color);
}

void fillRect(int x1, int y1, int x2, int y2, color_t color) {
    if (verbose)
	streamPrintf(Serial, "R (%d %d), %dx%d\n", x1, y1, x2, y2);
    tft.rectangle(x1, y1, x2, y2, true, color);
}

//
// The PFD
//

void showPFD(float g, sensors_vec_t *orient, sensors_vec_t *mag, int alt) {
    // Transformation matrix: rotation by orient->z and xlation by orient->y.
    float roll = -orient->z, pitch = -orient->y; 
    float M[2][3];
    // getTransform(M, 20.0, 2.5); // TMP
    getTransform(M, roll, pitch);

    // Paint sky and ground
    // ...

    // Render the moving card.
    if (verbose)
	streamPrintf(Serial, "Horizon\n");
    drawHorizon(M, pitch);
    if (verbose)
	streamPrintf(Serial, "Zero line\n");
    ladderLine(M, 0, 2);
    if (verbose)
	streamPrintf(Serial, "\n");
    ladderLine(M, 5, 4);
    ladderLine(M, 10, 15);
    ladderLine(M, 15, 4);
    ladderLine(M, 20, 25);
    ladderLine(M, 25, 4);
    ladderLine(M, 30, 35);

    // Draw the fixed "airplane" at the center.
    if (verbose)
	streamPrintf(Serial, "Left wing\n");
    drawWing(-paneWidth/2, 20, 4);
    if (verbose)
	streamPrintf(Serial, "Right wing\n");
    drawWing(paneWidth/2-20, 20, 4);
    if (verbose)
	streamPrintf(Serial, "Nose\n");
    drawNose(10, 4);

    // Draw the heading and scales at 0.
    //drawHeading(L, heading(mag->x, mag->y, mag->z));

    // Draw altitude and altimeter setting on the lower info strip.
    //drawAlt(L, alt);

    // All done.
    if (verbose)
	streamPrintf(Serial, "Done.\n\n");
}

// Draw a "wing": a vertically centered horizontal line that starts at
// lpos and goes for width pixels. The wing (a rectangle) is outlined
// by a 1-pixel line in the contrasting colour, so things need to be
// drawn in one colour and filled with another so they will show up
// against both ground and sky.
//
// Should just be a bitmap....
void drawWing(int lpos, int width, int thickness) {
    // After rotation the upper-right corner becomes the upper left.
    point p = {lpos+width, thickness/2};
    p = lscapeXform(p);
    // thickness and width get swapped because of the landscape xform.
    fillRect(p.x, p.y, p.x+thickness, p.y+width, tftFixedPlaneBG);
    drawRect(p.x, p.y, p.x+thickness, p.y+width, tftFixedPlaneFG);
}

void drawNose(int height, int thickness) {
    // The "nose" is an inverted V that fits in the HxW box, with its point
    // at screen center. The aspect ratio of the box is 4 i.e. a 2:1 slope.

    point p1 = {0, 1};
    point a = lscapeXform(p1);
    point p2l = {-height*2, -height};
    point p2r = {height*2, -height};
    for (int i = 0; i < thickness; i++) {
	point bl = lscapeXform(p2l);
	point br = lscapeXform(p2r);
	drawLine(a.x, a.y, bl.x, bl.y, 1, tftFixedPlaneBG);
	drawLine(a.x, a.y, br.x, br.y, 1, tftFixedPlaneBG);
	p2l.x += 2; p2r.x -= 2;
    }
}

// Horizon line
void drawHorizon(float M[2][3], float pitch) {
    // if pitch within limits then transform the endpoints and clipDraw()
    // else fill with either sky or ground colour.
    if (pitch < PITCH_MIN) {
	fillRect(0, 0, paneHeight, paneWidth, tftGround);
    } else if (pitch > PITCH_MAX) {
	fillRect(0, 0, paneHeight, paneWidth, tftSky);
    } else {
	// This doesn't quite work... round-off somewhere?
	point p1 = {-paneWidth*2, FATLINE_W/2};
	point p2 = {paneWidth*2, FATLINE_W/2};
	p1 = transform(M, p1);
	p2 = transform(M, p2);
	drawLine(p1.x, p1.y, p2.x, p2.y, FATLINE_W, tftSky);

	p1 = {-paneWidth*2, -FATLINE_W/2};
	p2 = {paneWidth*2, -FATLINE_W/2};
	p1 = transform(M, p1);
	p2 = transform(M, p2);
	drawLine(p1.x, p1.y, p2.x, p2.y, FATLINE_W, tftGround);

	p1 = {-paneWidth*2, 0};
	p2 = {paneWidth*2, 0};
	transformDrawLine(M, p1, p2);
    }
}

void paintSkyGround(int x1, int y1, int x2, int y2) {
    // Hmmm how do we know which side is sky?
    // ...
}

// Draw two lines of the pitch ladder.
void ladderLine(float M[2][3], int yDeg, int width) {
    point pLeft = {-width, yDeg * pxPerDeg};
    point pRight = {width, yDeg * pxPerDeg};
    transformDrawLine(M, pLeft, pRight);
    pLeft.y = -pLeft.y;
    pRight.y = -pRight.y;
    transformDrawLine(M, pLeft, pRight);

    if (width < 5)
	return;

    // Draw ticks at the ends of the lines
    point p0 = {pLeft.x, pLeft.y + 2};
    transformDrawLine(M, pLeft, p0);
    p0.x = pRight.x;
    transformDrawLine(M, pRight, p0);

    // Ticks for the mirrored line
    pLeft.y = -pLeft.y; pRight.y = -pRight.y;
    p0.x = pLeft.x; p0.y = pLeft.y - 2;
    transformDrawLine(M, pLeft, p0);
    p0.x = pRight.x;
    transformDrawLine(M, pRight, p0);
}

void transformDrawLine(float M[2][3], point p1, point p2) {
    p1 = transform(M, p1);
    p2 = transform(M, p2);
    drawLine(p1.x, p1.y, p2.x, p2.y, 1, tftLadderFG);
}

// Transform a point from portrait to landscape. The origin is in the
// center of the TFT.
point lscapeXform(point p) {
    point to;
    to.x = paneHeight/2 - p.y;
    to.y = paneWidth/2 - p.x;
    if (verbose)
	streamPrintf(Serial, "  L (%d, %d) => (%d, %d)\n", p.x, p.y, to.x, to.y);
    return to;
}

point transform(float M[2][3], point p) {
    point to;
    to.x = (int)(0.5 +  p.x * M[0][0]  +  p.y * M[0][1]  +  M[0][2]);
    to.y = (int)(0.5 +  p.x * M[1][0]  +  p.y * M[1][1]  +  M[1][2]);
    if (verbose)
	streamPrintf(Serial, "  M (%d, %d) => (%d, %d)\n", p.x, p.y, to.x, to.y);
    return to;
}

// A transformation matrix for a left-handed coordinate system.
void getTransform(float M[2][3], float roll, float pitch) {
    // Positive roll is to the left.
    // Positive pitch is up.

    // M = AI * LS
    // where AI is the "attitude indicator" transform:
    //   [ R P ] where R is the rotation by the roll angle, and P is a
    //   vertical translation by the pitch angle (scaled appropriately)
    // and LS is the "portrait to landscape" transform:
    //   [ R90 H ] a rotation by 90 followed by a translation that puts
    //   the origin at the center of the display.
    //
    // | cos(r) -sin(r)   0         |     | 0 -1 W/2 + H |
    // | sin(r)  cos(r)   scaled(p) |  X  | 1  0 H/2     |
    // | 0       0        1         |     | 0  0 1       |
    //

    // The first transformation is portrait->landscape (90 CW roll and
    // translate R by 160).

    float radroll = roll * Deg2Rad;
    float cosRoll = cos(radroll);
    float sinRoll = sin(radroll);

    if (verbose) {
	streamPrintf(Serial, "    %f %f\n", sinRoll, cosRoll);
    }

    M[0][0] = -sinRoll;
    M[0][1] = -cosRoll;
    M[0][2] = paneHeight/2.0 - pxPerDeg * pitch;

    M[1][0] = -cosRoll; 
    M[1][1] = sinRoll;
    M[1][2] = paneWidth/2.0;
}

//
// Info strips
//

void drawText(int vpos, txtAlign_t align, color_t fg, color_t bg,
	      const char* msg, int msgLen) {
    point p = {0, vpos};
    int w = msgLen * charX;
    switch (align) {
    case TA_LEFT:
	p.x = -paneWidth/2;
	break;
    case TA_CENTER:
	p.x = - w/2;
	break;
    case TA_RIGHT:
	p.x = paneWidth/2 - w;
	break;
    }
    p = lscapeXform(p);
    drawString(p.x, p.y, w, fg, bg, msg); // Text will go in the wrong direction....
}

void tftPrintf(int vpos, txtAlign_t align, color_t fg, color_t bg,
	       const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int rv = vsnprintf(longbuf, sizeof(longbuf), fmt, args);
    va_end(args);
    drawText(vpos, align, fg, bg, longbuf, rv);
}

void drawAlt(int32_t alt) {
    tftPrintf(tftAltYPos, TA_LEFT, tftInfoFG, tftInfoBG, "%.2f", qnh);
    tftPrintf(tftAltYPos, TA_RIGHT, tftInfoFG, tftInfoBG, "%d", alt);
}

void drawHeading(int hdg) {
    // Some dots at 10° intervals. Find the closest dot, and draw out from
    // there (the background for the label will be erased).
    int dotPos = (closestMult(hdg, 36, NULL) - hdg) * pxPerDeg;
    int dotPitch = 5 * pxPerDeg;
    for (int i = 1; i < 4; i++) {
	// Need to xform to landscape....
        drawPixel(dotPos + dotPitch * i, tftInfoHeight/2, tftInfoFG);
        drawPixel(dotPos - dotPitch * i, tftInfoHeight/2, tftInfoFG);
    }

    // There are 16 named cardinal directions.
    int closest;
    int closestDeg = closestMult(hdg, 16, &closest);
    int labelPos = (closestDeg - hdg) * pxPerDeg;
    drawCardinal(labelPos, closest);
    showOffsetCardinal(labelPos, closest, 1);
    showOffsetCardinal(labelPos, closest, -1);

    tftPrintf(paneHeight/2 - tftInfoHeight, TA_CENTER, tftInfoFG, tftInfoBG, "%03d", hdg);
}

void showOffsetCardinal(int pos, int idx, int offset) {
    if (offset == 1) {
        if (++idx == 16)
            idx = 0;
        drawCardinal(pos + cardinalPitch, idx);
    } else {
        if (--idx < 0)
            idx = 15;
        drawCardinal(pos - cardinalPitch, idx);
    }
}

void drawCardinal(int pos, int idx) {
    const char* label = cardinalLabels[idx];
    int lablen = strlen(label);
    if (lablen > 1)
        pos--;
    point p = {pos, paneHeight/2 - tftInfoHeight};
    p = lscapeXform(p);
    drawString(p.x, p.y, lablen * charX, tftInfoFG, tftInfoBG, label);
}

int closestMult(int deg, int steps, int* idx) {
    float div = 360.0/steps;
    int pos = int(deg/div + 0.5);
    if (idx)
        *idx = pos;
    return int(pos*div + 0.5);
}
