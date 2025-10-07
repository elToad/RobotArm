#include "PID.h"

PID::PID(double p, double i, double d, unsigned long *DT):
    kp(p), ki(i), kd(d), previous_error(0), setpoint(0), DT(DT), integral(0) {}

PID::~PID() {}

void PID::setP(double p) {
    kp = p;
}

void PID::setI(double i) {
    ki = i;
}
void PID::setD(double d) {
    kd = d;
}

double PID::compute(double measured_value) {
    double error = setpoint - measured_value;

    double dt_seconds = (*DT) / 1000000.0; // Convert to seconds but don't modify original DT

    double P = kp * error;

    integral += error * dt_seconds; // Accumulate error over time
    double I = ki * integral;

    if (integral > 255) integral = 255;
    if (integral < -255) integral = -255;

    double D = kd * (error - previous_error) / (dt_seconds + 1e-6); // Avoid division by zero

    previous_error = error;
    return P + I + D;
}

void PID::reset() {
    integral = 0;
    previous_error = 0;
}

void PID::setSetpoint(double newpoint){
    setpoint = newpoint;
}