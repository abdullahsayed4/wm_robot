# Motor Control — PID Tuning & Low-Pass Filter

This folder contains all MATLAB/Simulink work done for motor system identification, PID tuning, and digital low-pass filter design.

---

## Folder Structure

```
matlab/
├── pid_tuning/
│   ├── pid_tuning.slx       # Simulink model for PID tuning (System Identification)
│   └── importfile.m         # MATLAB script to import collected motor data
│
├── low_pass_filter/
│   ├── LowPassFilter.ipynb              # Low-pass filter design
│   ├── ButterworthFilter.ipynb          # Butterworth filter implementation
│   ├── Analytic Derivation for Low-Pass 2.0.ipynb  # Mathematical derivation
│   └── filter_response.png              # Filter frequency response plot
│
└── firmware/
    ├── motor1_pid.ino        # Motor 1 — Custom PID + Digital Filter (Teensy)
    └── motor2_pid.ino        # Motor 2 — Two-motor PID implementation
```

---

## PID Parameters (Motor 1)

Tuned via MATLAB System Identification Toolbox from real motor step response data:

| Parameter | Value |
|-----------|-------|
| **Kp** | 19.15 |
| **Ki** | 306.0 |
| **Kd** | 0.055 |
| **Sample Time** | 20 ms (50 Hz) |
| **Output Limits** | -250 to +250 (PWM) |

---

## Low-Pass Filter Design

A 2nd-order digital low-pass filter was derived analytically using the **Bilinear Transform** (Tustin method):

- **Cutoff Frequency:** Fc = 2.5 Hz
- **Sample Time:** dt = 0.020 s
- **Method:** Butterworth → Bilinear Transform

**Filter Coefficients:**
```
alpha = 0.728490
beta  = 0.135755
```

**Difference Equation:**
```
y[n] = alpha * y[n-1] + beta * x[n] + beta * x[n-1]
```

The filter smooths raw encoder velocity readings before feeding them into the PID controller, eliminating electrical noise without introducing significant lag.

---

## Methodology

1. **Data Collection** — Motor driven open-loop, encoder data logged via Serial at 50 Hz
2. **System Identification** — MATLAB `System Identification Toolbox` used to fit a transfer function to the collected step response data
3. **PID Tuning** — Simulink PID Tuner used to find optimal Kp, Ki, Kd
4. **Filter Design** — Butterworth filter analytically derived and discretized via Bilinear Transform
5. **Validation** — PID + filter tested on real hardware with square wave setpoint tracking
