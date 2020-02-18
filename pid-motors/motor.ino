
#include "motor.h"

void initMotors() {
    if (verbose)
        Serial.print("Motor driver");

    motorCtl.settings.commInterface = I2C_MODE;
    motorCtl.settings.I2CAddress = SCMD_I2C_ADDR;
    for (;;) {
        int id = motorCtl.begin();
        if (id == 0xa9)
            break;
        if (verbose)
            streamPrintf(Serial, "Motor driver mismatch: 0x%02x\n", id);
        delay(500);
    }
    if (verbose)
        Serial.print(" initialized");

    // Wait for enumeration
    while (!motorCtl.ready())
        ;
    if (verbose)
        Serial.print(" enumerated");

    while (motorCtl.busy())
        ;
    motorCtl.enable();
    if (verbose)
        Serial.print(" enabled");

    // Reverse the direction of the right motor
    motorCtl.inversionMode(motorR, 1);

    // Initialize the motor structs
    initStructs();

    if (verbose)
        Serial.println();
}

void initStructs() {
    memset(motors, 0, 2 * sizeof(motor_t));
    
    motor_t* m = &motors[0];
    for (int i = 0; i < 2; m = &motors[++i]) {
        m->id = i;
        m->speed = 0.0;
        m->meanSpeed = 0.0;
        m->targetSpeed = 0.0;
        m->prevError = 0.0;
        m->sumError = 0.0;
        m->mode = STOPPED;

        m->encoderPins[0] = encoderPins[i][0];
        m->encoderPins[1] = encoderPins[i][1];

        // PID starting values
        m->kp = 0.3;
        m->ki = 0.05;
        m->kd = 0.0;
    }
}

void motor_t::start() {
    mode = RUNNING;
    update();
}

void motor_t::stop() {
    mode = STOPPED;
    motorCtl.setDrive(id, 0, 0);
}

void motor_t::update() {
    if (mode == STOPPED)
        return;
    if (driveLevel >= 0)
        motorCtl.setDrive(id, 0, driveLevel);
    else
        motorCtl.setDrive(id, 1, -driveLevel);
}

void motorLhandler() {
    digitalWrite(pinMonitor, HIGH);
    motor_t* m = &motors[motorL];
    wheelEvent(m,
               digitalRead(m->encoderPins[0]) == digitalRead(m->encoderPins[1]));
    digitalWrite(pinMonitor, LOW);
}

void motorRhandler() {
    digitalWrite(pinMonitor, HIGH);
    motor_t* m = &motors[motorR];
    wheelEvent(m,
               // The right motor runs in the opposite direction.
               digitalRead(m->encoderPins[0]) != digitalRead(m->encoderPins[1]));
    digitalWrite(pinMonitor, LOW);
}
