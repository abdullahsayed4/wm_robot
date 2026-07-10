#include <Arduino.h>
#include "CytronMotorDriver.h"
#include <Encoder.h>
#include <PID_v1.h>

// ================= MOTORS =================
CytronMD motor1(PWM_DIR, 9, 8);
CytronMD motor2(PWM_DIR, 10, 7);

// ================= ENCODER =================
Encoder myEnc(2, 3);

// ================= VARIABLES =================
long prevCount = 0;
unsigned long lastTime = 0;
double N = 1440.0;

// ================= FILTER =================
double alpha = 0.728490;
double beta  = 0.135755;

double xn1 = 0.0;
double yn1 = 0.0;

// ================= PID =================
double Setpoint, Input, Output;

double Kp = 17.7;
double Ki = 350;
double Kd = 0.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  delay(3000);

  Setpoint = -6.0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(-250, 250);

  lastTime = millis();
}

void loop()
{
  unsigned long now = millis();

  if (now - lastTime >= 20)
  {
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // ===== Encoder Speed =====
    long count = myEnc.read();
    long delta = count - prevCount;
    prevCount = count;

    double rawSpeed =
        ((double)delta / N) * (2.0 * PI / dt);

    // ===== Low Pass Filter =====
    double xn = rawSpeed;

    double yn =
        (alpha * yn1) +
        (beta * xn) +
        (beta * xn1);

    xn1 = xn;
    yn1 = yn;

    // ===== PID =====
    Input = yn;

    myPID.Compute();

    // ===== Motors =====
    motor1.setSpeed((int)Output);
    motor2.setSpeed((int)Output);

    // ===== Serial Monitor =====
    Serial.print("Time:");
    Serial.print(now);

    Serial.print(",Target:");
    Serial.print(Setpoint);

    Serial.print(",FilteredSpeed:");
    Serial.print(yn, 4);

    Serial.print(",RawSpeed:");
    Serial.print(rawSpeed, 4);

    Serial.print(",PWM:");
    Serial.println((int)Output);
  }
}
