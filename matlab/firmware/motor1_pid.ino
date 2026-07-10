#include <Arduino.h>
#include "CytronMotorDriver.h"
#include <Encoder.h>
#include <PID_v1.h>

// ================= MOTOR & ENCODER =================
CytronMD motor(PWM_DIR, 3,4) ;
Encoder myEnc(1, 2);

// ================= VARIABLES =================
long prevCount = 0;
unsigned long lastTime = 0;
double N = 1440.0;

// ================= FILTER COEFFICIENTS =================
// Fc = 2.5 Hz, dt = 0.020s (Bilinear Transform)
double alpha = 0.728490;
double beta  = 0.135755;
double xn1 = 0.0;   // السرعة الخام السابقة
double yn1 = 0.0;   // السرعة المفلترة السابقة

// ================= PID SETTINGS =================
double Setpoint, Input, Output;
double Kp = 19.15;
double Ki = 306.0;
double Kd = 0.055;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  //pinMode(2, INPUT_PULLUP);
 // pinMode(3, INPUT_PULLUP);
  delay(3000);

  Setpoint = 5.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(-250, 250);
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastTime >= 20) {
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // حساب السرعة الخام
    long count = myEnc.read();
    long delta = count - prevCount;
    prevCount = count;
    double rawSpeed = ((double)delta / N) * (2.0 * PI / dt);

    // تطبيق الفلتر
    double xn = rawSpeed;
    double yn = (alpha * yn1) + (beta * xn) + (beta * xn1);

    // تحديث القيم السابقة
    xn1 = xn;
    yn1 = yn;

    // الـ PID يشتغل على السرعة المفلترة
    Input = yn;
    myPID.Compute();
    motor.setSpeed((int)Output);

    // طباعة البيانات
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
