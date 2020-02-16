// Motor interfaces

// Do whatever it takes to make pitch == 0.
void rebalance(float pitch) {
    oled.setCursor(0, OLED_H - FONT_H);
    if (pitch > 0.0) {
        streamPrintf(oled, "Left %.1f", pitch);
    }
    if (pitch < 0.0) {
         streamPrintf(oled, "Right %.1f", -pitch);
    }
}
