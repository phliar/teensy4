
#ifndef MOTOR_H
#define MOTOR_H
#pragma once

typedef enum { STOPPED, RUNNING } runMode_t;

const int filterDepth = SMOOTHING; // Get average speed over the last N counts
const uint8_t motorL = 0, motorR = 1;

struct motor_t {
    uint8_t id;

    volatile int count;
    volatile float speed;       // used by the controller
    volatile float meanSpeed;   // for display

    volatile float targetSpeed;
    volatile int16_t driveLevel;

    uint8_t encoderPins[2];

    int position[filterDepth];
    int timestamp[filterDepth];
    int idx;

    int prevEventTS;
    int prevStepTS;

    // PID params

    float kp; // 0.3
    float ki; // 0.05
    float kd;

    float prevError;
    float sumError;

    void start();
    void update();
    void stop();
    bool isRunning() { return mode == RUNNING; }

    runMode_t mode;
};

#endif
