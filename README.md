# Quaternion Library Usage Example: Gyro Integration and Accelerometer Correction

This document provides an example of how to use the Quaternion library for a common task in orientation sensing: integrating gyroscope data and correcting for drift using accelerometer data. The example is styled after typical Arduino `setup()` and `loop()` functions for clarity.

## Prerequisites

- This Quaternion library.
- A sensor system providing:
    - Gyroscope readings (`gx, gy, gz`) in radians per second.
    - Accelerometer readings (`ax, ay, az`) - typically in m/s^2 or g's, but used here as a vector indicating the direction of "up" or "down".

## Arduino-style Example

This example demonstrates a complementary filter approach.

```cpp
// main.cpp (Example, assumes Quaternion.h is available)
#include "Quaternion.h"
#include <cmath>    // For std::sqrt, std::acos
#include <iostream> // For printing
#include <vector>   // For conceptual sensor data history
#include <numeric>  // For std::accumulate
#include <cstdlib>  // For rand()

// --- Global variables ---
Quaternion orientation_q; // Stores the current orientation

// Sensor data (conceptual - replace with actual sensor reads)
float gx_rad_s, gy_rad_s, gz_rad_s; // Gyro readings in rad/s
float ax_m_s2, ay_m_s2, az_m_s2;    // Accelerometer readings

// Timing
float dt = 0.01f; // Sample period (e.g., 100Hz / 0.01s) - adjust to your loop rate

// Filter parameters
// gyro_confidence determines how much of the correction is applied.
// A smaller value (e.g., 0.02) means accelerometer corrects slowly (long time constant).
// A larger value (e.g., 0.1) means accelerometer corrects faster (shorter time constant).
// This is effectively the 'alpha' in a simple complementary filter where:
// angle = (1-alpha)* (angle + gyro * dt) + alpha * accel_angle
// Here, we apply a fraction of the quaternion error.
float accel_correction_factor = 0.02f;


// --- Function declarations (conceptual for Arduino style) ---
void setup();
void loop();
void read_sensors_and_update_state(); // Placeholder for actual sensor reading logic

// --- Main (for non-Arduino testing) ---
int main() {
    setup();
    for (int i = 0; i < 1000; ++i) { // Simulate 1000 loops
        read_sensors_and_update_state(); // Simulate reading sensors
        loop(); // Process sensor data
        // In a real scenario, delay or manage loop timing for 'dt'
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
    // Initialize orientation quaternion to identity (no rotation, aligned with world frame)
    orientation_q = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

    // Conceptual: Initialize sensors here
    // conceptual_sensor_init();

    std::cout << "Setup complete. Initial orientation: w=" << orientation_q.a
              << " x=" << orientation_q.b << " y=" << orientation_q.c
              << " z=" << orientation_q.d << std::endl;
}

// --- Arduino-style loop ---
void loop() {
    // 1. Gyro Integration
    // Calculate rotation vector components from gyro rates and time delta
    float rx = gx_rad_s * dt;
    float ry = gy_rad_s * dt;
    float rz = gz_rad_s * dt;

    Quaternion delta_q = Quaternion::from_axis_angle(rx, ry, rz);
    orientation_q = orientation_q * delta_q;
    orientation_q.normalize();

    // 2. Accelerometer-based Drift Correction
    float accel_mag = std::sqrt(ax_m_s2*ax_m_s2 + ay_m_s2*ay_m_s2 + az_m_s2*az_m_s2);

    if (accel_mag > 0.1f) { // Only correct if acceleration is significant (e.g., not in freefall)
        Quaternion accel_body_q(0.0f, ax_m_s2 / accel_mag, ay_m_s2 / accel_mag, az_m_s2 / accel_mag);

        // Gravity vector in world frame (assuming Z is up, so gravity acts along -Z)
        // The accelerometer measures the reaction force, so it points "up" from the device.
        // So, the world reference for "up" is (0,0,1)
        Quaternion world_up_reference_q(0.0f, 0.0f, 0.0f, 1.0f);

        // Predicted "up" vector in the body frame, based on current orientation_q
        // This is where the body's Z-axis (if it were aligned with world Z) would point
        // if rotated by orientation_q.conj() (world to body transformation).
        // Or, how the world's Z vector appears in the body's frame.
        Quaternion predicted_up_body_q = orientation_q.conj().rotate(world_up_reference_q);
        predicted_up_body_q.normalize(); // ensure it's pure vector and normalized after rotation

        // Calculate the corrective rotation from predicted "up" to measured "up" (accel vector)
        Quaternion error_correction_q = predicted_up_body_q.rotation_between_vectors(accel_body_q);

        // Fractional application of correction
        // The `fractional` method computes: q_new = normalize((1-f)*Identity + f*error_correction_q)
        // This is equivalent to slerping from Identity towards error_correction_q by factor 'f'.
        error_correction_q.fractional(accel_correction_factor);

        orientation_q = orientation_q * error_correction_q;
        orientation_q.normalize();
    }
}

// --- Placeholder for sensor reading logic ---
void read_sensors_and_update_state() {
    static float true_angle_y_rad = 0.0f;
    // Simulate a true rotation rate (e.g., around Y axis)
    float true_rotation_rate_y_rad_s = 0.5f; // rad/s
    true_angle_y_rad += true_rotation_rate_y_rad_s * dt;

    // Simulate Gyro: true rate + bias + noise
    float gyro_bias = 0.01f; // rad/s
    float gyro_noise_scale = 0.05f; // rad/s
    gx_rad_s = 0.0f + gyro_bias + gyro_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
    gy_rad_s = true_rotation_rate_y_rad_s + gyro_bias + gyro_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
    gz_rad_s = 0.0f + gyro_bias + gyro_noise_scale * (rand() / (float)RAND_MAX - 0.5f);

    // Simulate Accelerometer:
    // It measures the direction opposite to gravity (+ any linear acceleration).
    // If world Z is up, gravity is (0,0,-G). Accelerometer measures (0,0,G) if level.
    // We need to rotate this world gravity vector into the body frame.
    // True orientation (for simulation):
    Quaternion true_orientation = Quaternion::from_axis_angle(0, true_angle_y_rad, 0);

    Quaternion world_gravity_vector(0.0f, 0.0f, 0.0f, -1.0f); // Points along -Z world
    Quaternion body_accel_ideal = true_orientation.conj().rotate(world_gravity_vector);
                                                           // This rotates world -Z to body frame.
                                                           // Accel measures reaction, so invert.

    float accel_noise_scale = 0.2f; // m/s^2
    // Accelerometer measures force that counteracts gravity. So if world Z is up,
    // a level accelerometer measures (0,0, +G).
    // Let's define world_up = (0,0,1) and find its representation in body frame.
    Quaternion world_up_vector(0.0f, 0.0f, 0.0f, 1.0f);
    Quaternion body_accel_measures = true_orientation.conj().rotate(world_up_vector);


    ax_m_s2 = body_accel_measures.b + accel_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
    ay_m_s2 = body_accel_measures.c + accel_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
    az_m_s2 = body_accel_measures.d + accel_noise_scale * (rand() / (float)RAND_MAX - 0.5f);
     // Ensure some magnitude if noise makes it zero, to avoid div by zero later
    if (std::sqrt(ax_m_s2*ax_m_s2 + ay_m_s2*ay_m_s2 + az_m_s2*az_m_s2) < 0.01f) {
        az_m_s2 = 1.0f;
    }
}

```

## Notes

*   **Coordinate Systems:** Be very mindful of your sensor, body, and world coordinate systems. This example assumes a common convention (e.g., Z-up world frame, sensor axes aligned with body axes). Adjust vector definitions (like `world_up_reference_q`) and quaternion multiplication order accordingly. The example uses Z-up for the world frame, where an accelerometer resting flat on a table would measure `(0,0,+G)`.
*   **Gyro Bias:** Real gyroscopes have bias. Implement gyro calibration and subtract bias before integration. The example simulates a small bias.
*   **Accelerometer Noise & External Accelerations:** Accelerometers measure proper acceleration (gravity + linear acceleration). The correction works best when the device is not undergoing significant linear acceleration (or when such accelerations are filtered out or accounted for).
*   **`from_axis_angle_approx`:** For performance-critical applications where rotation angles per step are very small, `Quaternion::from_axis_angle_approx(rx, ry, rz)` can be used instead of `from_axis_angle`. Remember that `from_axis_angle_approx` now normalizes its result.
*   **Normalization Frequency:** Normalizing the `orientation_q` quaternion periodically (e.g., every step or every few steps) is crucial to counteract floating-point error accumulation.
*   **`accel_correction_factor`:** This tuning parameter (e.g., 0.02) determines how aggressively the accelerometer corrects the gyro. A smaller value makes the correction slower and smoother, relying more on the gyro in the short term. A larger value corrects faster but can make the orientation estimate more susceptible to accelerometer noise and jitter from linear accelerations.
*   **Sensor Data:** The `read_sensors_and_update_state()` function is a placeholder with simulated data. You'll need to implement actual sensor communication and data processing.
*   **Timing (`dt`):** Accurate and consistent `dt` is important for gyro integration.
*   **Initial Orientation:** The example initializes to an identity quaternion. If the device starts in a known orientation, initialize `orientation_q` accordingly. Magnetometers are often used for absolute yaw correction, which is not covered in this example.

This example provides a starting point for a complementary filter. Robust sensor fusion often involves more sophisticated algorithms like Kalman filters (e.g., EKF, UKF) for optimal state estimation.
