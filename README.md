# Quaternion Library Usage Example: Gyro Integration and Accelerometer Correction

This document provides an example of how to use the Quaternion library for a common task in orientation sensing: integrating gyroscope data and correcting for drift using accelerometer data. The example is styled after typical Arduino `setup()` and `loop()` functions for clarity.

**Convention Note:** In this example, `orientation_q` represents the rotation from the **body frame to the world frame**. So, `world_vector = orientation_q.rotate(body_vector)`.

## Prerequisites

- This Quaternion library.
- A sensor system providing:
    - Gyroscope readings (`gx_rad_s, gy_rad_s, gz_rad_s`) in radians per second (body frame).
    - Accelerometer readings (`ax_m_s2, ay_m_s2, az_m_s2`) in m/s^2 (body frame).

## Arduino-style Example

This example demonstrates a complementary filter approach.

```cpp
// main.cpp (Example, assumes Quaternion.h is available)
#include "Quaternion.h"
#include <cmath>    // For std::sqrt
#include <iostream> // For printing
#include <cstdlib>  // For rand()

// --- Global variables ---
Quaternion orientation_q; // Stores the current orientation (body frame to world frame)

// Sensor data (conceptual - replace with actual sensor reads)
float gx_rad_s, gy_rad_s, gz_rad_s; // Gyro readings in rad/s from body frame
float ax_m_s2, ay_m_s2, az_m_s2;    // Accelerometer readings in m/s^2 from body frame

// Timing
float dt = 0.01f; // Sample period (e.g., 100Hz / 0.01s) - adjust to your loop rate

// Filter parameters
float accel_correction_factor = 0.02f; // How strongly accelerometer corrects gyro drift

// Constants for accelerometer correction window
const float GRAVITY_MSS = 9.80665f;         // Standard gravity
const float ACCEL_WINDOW_MSS = 1.0f;      // Allowable deviation from 1g (e.g., +/- 1.0 m/s^2)

// --- Function declarations (conceptual for Arduino style) ---
void setup();
void loop();
void read_sensors_and_update_state(); // Placeholder for actual sensor reading logic

// --- Main (for non-Arduino testing) ---
int main() {
    setup();
    for (int i = 0; i < 1000; ++i) { // Simulate 1000 loops
        read_sensors_and_update_state();
        loop();
        if (i % 100 == 0) {
             std::cout << "Loop " << i << ". Orientation: w=" << orientation_q.a
                       << " x=" << orientation_q.b << " y=" << orientation_q.c
                       << " z=" << orientation_q.d << std::endl;
        }
    }
    return 0;
}

// --- Arduino-style setup ---
void setup() {
    orientation_q = Quaternion(1.0f, 0.0f, 0.0f, 0.0f); // Initialized to identity

    std::cout << "Setup complete. Initial orientation (body to world): w=" << orientation_q.a
              << " x=" << orientation_q.b << " y=" << orientation_q.c
              << " z=" << orientation_q.d << std::endl;
}

// --- Arduino-style loop ---
void loop() {
    // 1. Gyro Integration
    float rx = gx_rad_s * dt;
    float ry = gy_rad_s * dt;
    float rz = gz_rad_s * dt;

    Quaternion delta_q = Quaternion::from_axis_angle(rx, ry, rz);
    // Since orientation_q is body-to-world, and delta_q is a rotation in the current body frame:
    orientation_q = orientation_q * delta_q;
    orientation_q.normalize();

    // 2. Accelerometer-based Drift Correction
    float accel_norm = std::sqrt(ax_m_s2*ax_m_s2 + ay_m_s2*ay_m_s2 + az_m_s2*az_m_s2);

    // Only apply correction if accelerometer reading is close to 1g (i.e., mostly sensing gravity)
    if (accel_norm > (GRAVITY_MSS - ACCEL_WINDOW_MSS) &&
        accel_norm < (GRAVITY_MSS + ACCEL_WINDOW_MSS)) {

        // Normalized accelerometer vector (measured "up" in body frame)
        Quaternion measured_up_body_q(0.0f, ax_m_s2 / accel_norm, ay_m_s2 / accel_norm, az_m_s2 / accel_norm);

        // World's "up" vector (e.g., positive Z axis).
        // If your sensor is mounted upside down (e.g., its +Z points to world -Z),
        // this reference should be (0,0,0,-1).
        Quaternion world_up_reference_q(0.0f, 0.0f, 0.0f, 1.0f);

        // What the accelerometer *should* be reading if orientation_q were perfect.
        // This transforms the world "up" vector into the body's coordinate frame using current orientation.
        // (orientation_q.conj() is world-to-body)
        Quaternion expected_up_body_q = orientation_q.conj().rotate(world_up_reference_q);
        // The .rotate() method applies q*v*q.conj. So orientation_q.conj().rotate(world_up_reference_q)
        // is (q_conj * world_up * q_conj.conj()) = (q_conj * world_up * q). This is correct.

        // Calculate the corrective rotation to align what the gyro thinks "up" is in the body frame
        // (expected_up_body_q) with what the accelerometer measures "up" as in the body frame (measured_up_body_q).
        Quaternion error_correction_q = expected_up_body_q.rotation_between_vectors(measured_up_body_q);

        // Fractional application of correction
        error_correction_q.fractional(accel_correction_factor);

        // Apply the small correction to the body-to-world orientation
        orientation_q = orientation_q * error_correction_q;
        orientation_q.normalize();
    }
    // else: Accelerometer reading is not reliable for gravity correction (e.g., during high linear acceleration).
    //       Orientation will continue to be updated by gyro only.
}

// --- Placeholder for sensor reading logic & state update ---
void read_sensors_and_update_state() {
    static float true_angle_y_rad = 0.0f;
    float true_rotation_rate_y_rad_s = 0.5f; // rad/s

    // Simulate time passing by updating the true angle for the simulation
    true_angle_y_rad += true_rotation_rate_y_rad_s * dt;

    // Simulate Gyro: true rate + bias + noise
    float gyro_bias = 0.01f;
    float gyro_noise_scale = 0.05f;
    gx_rad_s = 0.0f + gyro_bias + gyro_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
    gy_rad_s = true_rotation_rate_y_rad_s + gyro_bias + gyro_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
    gz_rad_s = 0.0f + gyro_bias + gyro_noise_scale * (rand() / (float)RAND_MAX - 0.5f);

    // Simulate Accelerometer:
    // True orientation for simulation (body-to-world)
    Quaternion current_true_orientation_q = Quaternion::from_axis_angle(0.0f, true_angle_y_rad, 0.0f);

    // World "up" vector (points along +Z world axis)
    Quaternion world_up_vector(0.0f, 0.0f, 0.0f, 1.0f);

    // Ideal accelerometer reading: world "up" vector transformed into body frame.
    // Accel measures the reaction force, which is opposite to gravity.
    // If Z-world is up, gravity pulls along -Z world. Accel measures force along +Z body when level.
    Quaternion ideal_accel_in_body_q = current_true_orientation_q.conj().rotate(world_up_vector);

    float accel_noise_scale = 0.2f; // m/s^2
    ax_m_s2 = ideal_accel_in_body_q.b * GRAVITY_MSS + accel_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
    ay_m_s2 = ideal_accel_in_body_q.c * GRAVITY_MSS + accel_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
    az_m_s2 = ideal_accel_in_body_q.d * GRAVITY_MSS + accel_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
}
```

## Notes

*   **Coordinate Systems & `orientation_q` Convention:** This example defines `orientation_q` as representing the rotation from the **body frame to the world frame**. This means `world_vector = orientation_q.rotate(body_vector)`. If you use a world-to-body convention, quaternion multiplication orders and conjugations for transformations will need to be adjusted. The world frame is assumed to be Z-up (gravity acts along negative Z). The accelerometer is assumed to measure the reaction to gravity, so when level and Z-body is up, it reads `(0,0,+G)`.
*   **Gyro Bias:** Real gyroscopes have bias. Implement gyro calibration and subtract bias from `gx_rad_s, gy_rad_s, gz_rad_s` before integration.
*   **Accelerometer Limitations:** The accelerometer-based correction is most effective when the device is quasi-static (not undergoing significant linear acceleration). The example includes a basic check to only apply correction when the accelerometer norm is close to 1g.
*   **`from_axis_angle_approx`:** For performance-critical applications where rotation angles per step are very small, `Quaternion::from_axis_angle_approx(rx, ry, rz)` can be used. It now normalizes its result.
*   **Normalization Frequency:** Normalizing `orientation_q` after updates is crucial to counteract floating-point error accumulation.
*   **`accel_correction_factor`:** This (e.g., 0.02) tunes how quickly the accelerometer corrects gyro drift. Smaller values trust the gyro more in the short term and make corrections smoother.
*   **Sensor Data:** The `read_sensors_and_update_state()` function uses simulated data. Replace with actual sensor interfacing.
*   **Timing (`dt`):** An accurate and consistent `dt` is vital.
*   **Initial Orientation & Yaw:** This example initializes to an identity quaternion. For absolute yaw, a magnetometer is typically integrated.

This example provides a foundational complementary filter. More advanced sensor fusion algorithms (like Kalman filters) can provide better performance in dynamic conditions.
