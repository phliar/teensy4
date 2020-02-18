
void wheelEvent(motor_t* m, uint8_t dir) {
    if (dir)
        m->count++;
    else
        m->count--;
    int now = micros();

    int startTime = m->timestamp[m->idx];
    int startPos = m->position[m->idx];
    int elapsed = now - startTime;
    int distance = m->count - startPos;
    m->position[m->idx] = m->count;
    m->timestamp[m->idx] = now;
    m->idx = (m->idx + 1) % filterDepth;

    m->meanSpeed = 1.0e6 * distance/(2.0 * elapsed);
    m->speed = (dir? 1e6 : -1e6) / (float)(now - m->prevEventTS);
    m->prevEventTS = now;
    
    // Show current speed and position every 2˚ (99*48/360 = 13.3 counts/deg.)
    // if (m->idx % 27 == 0) showData();
}

void runPID() {
    int now = micros();
    runStep(&motors[motorL], now);
    showData();
}

void runStep(motor_t* m, int now) {
    float err = m->targetSpeed - m->speed;
    static int lastRun;
    float dt = (now - lastRun)/1.0e6;
    lastRun = now;

    // Read ADCs (12-bit)
    int kpv = analogRead(pinKp);
    int kiv = analogRead(pinKi);
    m->kp = fullScaleKp * kpv / 4095.0;
    m->ki = fullScaleKi * kiv / 4095.0;

    float dErr = err - m->prevError;
    m->prevError = err;
    m->sumError += err;
    m->driveLevel = clamp(-255, 255,
                          m->kp * err +
                          m->ki * m->sumError * dt +
                          m->kd * dErr/dt);
    m->update();
}

int16_t clamp(int minVal, int maxVal, float x) {
    if (x < minVal)
        return minVal;
    if (x > maxVal)
        return maxVal;
    return (int16_t)(0.5 + x);
}
