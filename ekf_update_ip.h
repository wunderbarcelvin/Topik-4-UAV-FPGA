#ifndef EKF_UPDATE_IP_H
#define EKF_UPDATE_IP_H

#include <stdbool.h>
#include <stdint.h>

// Uncomment in W11 for fixed-point quantization:
// #include <ap_fixed.h>
// typedef ap_fixed<32, 16> data_t;
typedef float data_t;

#define STATE_DIM 7
#define MAHALANOBIS_THRESH_1DOF 9.0f  // 3-sigma gate (3^2 = 9.0) for 1D scalar measurement
#define BUFFER_CAPACITY 32            // Circular buffer depth for 100Hz IMU (covers up to 320 ms delay)

// Sensor Channel Identifiers
typedef enum {
    CH_GNSS_X = 0,
    CH_GNSS_Y = 1,
    CH_GNSS_Z = 2,
    CH_BARO_Z = 3,
    CH_MAG_YAW = 4
} SensorChannel_t;

// Top-Level HLS IP Operation Modes (Controlled by ARM PS via AXI4-Lite)
typedef enum {
    MODE_INIT = 0,               // Initialize state x, UD covariance (U, D), and clear ring buffer
    MODE_PUSH_IMU = 1,           // Log IMU step into BRAM ring buffer & propagate state
    MODE_UPDATE_SCALAR = 2,      // Run fast Bierman scalar update (Barometer / Magnetometer)
    MODE_UPDATE_DELAYED_GNSS = 3 // Rewind to delayed GNSS timestamp, fuse X/Y/Z, and replay IMU
} EkfMode_t;

// Top-Level Synthesizable HLS Function Prototype
void ekf_meas_top(
    uint8_t mode,
    uint8_t channel,
    bool data_valid,
    data_t timestamp_ms,
    data_t dt,
    // Scalar measurement inputs (Baro / Mag)
    data_t z_meas,
    data_t R_var,
    // IMU inputs (for buffer push & replay)
    data_t ax, data_t ay, data_t az, data_t wz,
    // Delayed GNSS packet inputs
    data_t gnss_x, data_t gnss_y, data_t gnss_z,
    data_t R_gnss_xy, data_t R_gnss_z,
    // Initial state input (used only in MODE_INIT)
    const data_t x_init[STATE_DIM],
    // Outputs read back by ARM PS / Testbench
    data_t x_out[STATE_DIM],
    data_t P_diag_out[STATE_DIM],
    data_t *nis_out,
    bool *update_applied
);

#endif // EKF_UPDATE_IP_H