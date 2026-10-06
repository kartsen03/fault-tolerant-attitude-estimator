# Requirements

Status: **draft**. IDs and scope are fixed; acceptance criteria marked
*(to be finalised)* are completed with the verification work in step 5.

Each requirement is verified by one or more tests whose suite or test name
contains its ID (for example `Req005GyroBias` verifies REQ-005).

| ID | Area | Requirement (summary) |
|----|------|-----------------------|
| REQ-001 | Timing | Release the estimation cycle every 10 ms (100 Hz) on an absolute schedule, so late cycles do not shift later releases. |
| REQ-002 | Timing | Detect and count deadline overruns and report the count in telemetry. |
| REQ-003 | Accuracy | Roll and pitch error within per-scenario RMS and max limits in SIL *(to be finalised)*. |
| REQ-004 | Calibration | Estimate the gyroscope bias at start-up while stationary; restart if motion is detected. |
| REQ-005 | Bias | With the accelerometer valid and a constant residual gyro bias b (abs(b) <= 2 deg/s), the error settles to abs(b)·tau ± 0.01 deg within 5·tau and does not grow afterwards. |
| REQ-006 | Ranges | Roll in (-180, 180] deg, pitch in [-90, 90] deg; continuous through ±180 deg roll; corrections take the short way round; finite near ±90 deg pitch. |
| REQ-007 | Validation | Every public core function rejects NULL pointers, non-finite or out-of-bounds inputs and use before initialisation, and leaves its state unchanged on error. |
| REQ-008 | Validation | Reject samples whose time step is zero, negative, non-finite or > 50 ms; handle 32-bit microsecond timestamp wraparound. |
| REQ-009 | Driver | MPU6050 driver checks WHO_AM_I, applies the configuration, converts units, and reports I2C timeout, NACK and bad data as distinct errors. |
| REQ-010 | Faults | Detect dropped-out samples *(latency to be finalised)*. |
| REQ-011 | Faults | Detect a stuck sensor *(latency to be finalised)*. |
| REQ-012 | Faults | Detect and reject single-sample spikes *(thresholds to be finalised)*. |
| REQ-013 | Faults | Detect gyroscope bias drift *(threshold and latency to be finalised)*. |
| REQ-014 | Degraded modes | Gyro-only operation for a bounded time when the accelerometer is unusable; accelerometer-only when the gyro is unusable; recovery when the sensor returns *(timings to be finalised)*. |
| REQ-015 | Faults | No fault flags on fault-free scenarios *(false-positive budget to be finalised)*. |
