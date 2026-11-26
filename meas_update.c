#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

//MILESTONE: OBSERVATION MODEL
//Mapping internal state vector $x$ to what each sensor actually measures.

//assuming 7-state UAV vector $x = [p_x, p_y, p_z, v_x, v_y, v_z, \psi]^T$ (3D Position, 3D Velocity, and Yaw heading $\psi$).

//GNSS (3 channels for position), Barometer (depending on NED/ENU frame)
#define STATE_DIM 7
#define MAHALANOBIS_THRESH_1DOF 9.0f  // 3-sigma gate (3^2 = 9.0) for 1D scalar measurement
#define BUFFER_CAPACITY 32            // Circular buffer depth for 100Hz IMU (covers up to 320 ms delay)

// 1. Enum automatically assigns 0, 1, 2, 3, 4 (no manual assignment needed)
typedef enum {
    CH_GNSS_X = 0,
    CH_GNSS_Y,
    CH_GNSS_Z,
    CH_BARO_Z,
    CH_MAG_YAW
} SensorChannel_t;

// 2. Observation model function
void get_observation_model(
    const float x[STATE_DIM], 
    SensorChannel_t channel, 
    float *h_x, 
    float H_row[STATE_DIM]
) {
    for (int i = 0; i < STATE_DIM; i++) {
        H_row[i] = 0.0f;
    }

    switch (channel) {
        case CH_GNSS_X:
            *h_x = x[0];
            H_row[0] = 1.0f;
            break;

        case CH_GNSS_Y:
            *h_x = x[1];
            H_row[1] = 1.0f;
            break;

        case CH_GNSS_Z:

        case CH_BARO_Z:
            *h_x = x[2];
            H_row[2] = 1.0f;
            break;

        case CH_MAG_YAW:
            *h_x = x[6];
            H_row[6] = 1.0f;
            break;
    }
}

// Helper: Wrap angle innovation to [-pi, pi] for Magnetometer Yaw
float wrap_pi(float angle) {
    const float PI = 3.14159265f;
    const float TWO_PI = 6.28318530f;
    while (angle > PI)  angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

// Helper: Factorize symmetric positive-definite P into P = U * diag(D) * U^T
void cov_to_ud(const float P_in[STATE_DIM][STATE_DIM], float U[STATE_DIM][STATE_DIM], float D[STATE_DIM]) {
    float A[STATE_DIM][STATE_DIM];
    memcpy(A, P_in, sizeof(float) * STATE_DIM * STATE_DIM);

    for (int i = 0; i < STATE_DIM; i++) {
        D[i] = 0.0f;
        for (int j = 0; j < STATE_DIM; j++) {
            U[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int j = STATE_DIM - 1; j >= 0; j--) {
        D[j] = A[j][j];
        float d_inv = (D[j] > 1e-12f) ? (1.0f / D[j]) : 0.0f;
        for (int i = 0; i < j; i++) {
            U[i][j] = A[i][j] * d_inv;
        }
        for (int i = 0; i < j; i++) {
            for (int k = 0; k <= i; k++) {
                A[k][i] -= U[k][j] * A[i][j];
            }
        }
    }
}

// Helper: Reconstruct full covariance matrix P = U * diag(D) * U^T (for inspection/debugging)
void ud_to_cov(const float U[STATE_DIM][STATE_DIM], const float D[STATE_DIM], float P_out[STATE_DIM][STATE_DIM]) {
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = i; j < STATE_DIM; j++) {
            float sum = 0.0f;
            for (int k = j; k < STATE_DIM; k++) {
                sum += U[i][k] * D[k] * U[j][k];
            }
            P_out[i][j] = sum;
            P_out[j][i] = sum;
        }
    }
}

// --PART 2 & PART 3 : Bierman UD Scalar Update with Pre-Replacement Outlier Gating

/*
Bierman UD Observation Update (Factorized Covariance P = U * D * U^T):
1. Compute f = U^T * H^T and v = D * f
2. Compute scalar innovation variance S = R + f^T * v and residual y = z - h(x)
3. OUTLIER GATING BEFORE ERROR REPLACEMENT: Check y^2 > gate_threshold * S before altering x, U, or D
4. Perform Bierman rank-1 recursive update on U and D (guarantees positive-definiteness without 0.5*(P+P^T))
5. Update state x = x + (v / alpha) * y
*/


/*LEVEL UP
No full update of covariance matrix; Bierman algorithm factors P into a unit upper-triangular matrix  U (diagonal entries = 1) and 
a diagonal variance vector D

Because Dj^new = Dj 8 (alpha_j-1/alpha_j) with alpha_0 = R >0*/

bool update_scalar_bierman(
    float x[STATE_DIM],
    float U[STATE_DIM][STATE_DIM], // [UPDATE: 23/11/2026] Unit upper-triangular covariance factor
    float D[STATE_DIM],            // [UPDATE: 23/11/2026] Diagonal covariance factor
    float z_meas,
    float R_var,
    SensorChannel_t channel,
    bool data_valid,               // [PART 3A] Asynchronous ready flag
    float gate_threshold,           // [PART 3B] Chi-square threshold (9.0f = 3-sigma)
    float *nis_out                 // [UPDATE: 23/11/2026] Export covariance entries and raw innovation stats (y^2 / S) into ekf_output.csv)
){
    // [PART 3A] Asynchronous Gating: 0-cycle bypass if sensor packet is not ready
    if (!data_valid) {
        return false;
    }

    if (nis_out) *nis_out = -1.0f; // -1 indicates inactive/skipped step
    if (!data_valid) {
        return false;
    }

    float h_x = 0.0f;
    float H[STATE_DIM];
    float f[STATE_DIM]; // f = U^T * H^T
    float v[STATE_DIM]; // v = D * f (becomes unnormalized Kalman gain after loop)

    get_observation_model(x, channel, &h_x, H);

    // Step 1: Compute f = U^T * H^T and v = D * f (exploiting upper-triangular U)
    for (int j = 0; j < STATE_DIM; j++) {
        float sum_f = H[j]; // Diagonal U[j][j] is always 1.0f
        for (int i = 0; i < j; i++) {
            sum_f += U[i][j] * H[i];
        }
        f[j] = sum_f;
        v[j] = D[j] * sum_f;
    }

    // Step 2: Compute scalar innovation covariance S = R + f^T * v
    float S = R_var;
    for (int j = 0; j < STATE_DIM; j++) {
        S += f[j] * v[j];
    }

    // Guard against non-positive innovation variance
    if (S <= 1e-9f || R_var <= 1e-9f) {
        return false;
    }

    // Compute innovation residual y = z - h(x) BEFORE touching x, U, or D
    float y_residual = z_meas - h_x;
    if (channel == CH_MAG_YAW) {
        y_residual = wrap_pi(y_residual);
    }

    // Compute and export NIS = y^2 / S BEFORE gating
    float nis = (y_residual * y_residual) / S;
    if (nis_out) *nis_out = nis;

    // Outlier Gating BEFORE Error Replacement
    if ((y_residual * y_residual) > (gate_threshold * S)) {
        return false;
    }


    // Step 3: [PART 3B] Outlier Gating BEFORE Error Replacement
    // Rejects spikes before modifying x, U, D or running Bierman's update loop
    if ((y_residual * y_residual) > (gate_threshold * S)) {
        return false;
    }

    // Step 4: Bierman Rank-1 UD Covariance Update (no explicit S^-1 matrix/scalar inversion)
    float alpha = R_var;
    for (int j = 0; j < STATE_DIM; j++) {
        float alpha_prev = alpha;
        float fj = f[j];
        float vj = v[j];

        alpha = alpha_prev + fj * vj;
        D[j]  = D[j] * (alpha_prev / alpha);

        float lambda = -fj / alpha_prev;
        for (int i = 0; i < j; i++) {
            float u_ij_old = U[i][j];
            U[i][j] = u_ij_old + v[i] * lambda;
            v[i]    = v[i] + u_ij_old * vj; // Accumulates unnormalized Kalman gain
        }
    }

    // Step 5: State Correction x = x + K * y, where K = v / alpha (note: final alpha == S)
    float scale = y_residual / alpha;
    for (int i = 0; i < STATE_DIM; i++) {
        x[i] += v[i] * scale;
    }

    return true;
}

// --PART 4 : Delayed-Aiding Ring Buffer (Rewind & Replay for 100-200 ms GNSS Latency)

typedef struct {
    float timestamp_ms;            // Epoch timestamp of IMU propagation step
    float x[STATE_DIM];            // Saved state snapshot
    float U[STATE_DIM][STATE_DIM]; // Saved UD upper-triangular factor
    float D[STATE_DIM];            // Saved UD diagonal factor
    float ax, ay, az;              // Saved IMU acceleration input (for replay)
    float wz;                      // Saved IMU yaw rate input (for replay)
    float dt;                      // Time step delta_t (e.g., 0.01s for 100Hz)
} HistorySlot_t;

typedef struct {
    HistorySlot_t slots[BUFFER_CAPACITY];
    int head;                      // Next write index
    int count;                     // Active stored history slots
} DelayBuffer_t;

void buffer_init(DelayBuffer_t *buf) {
    buf->head = 0;
    buf->count = 0;
}

// Save each 100Hz IMU prediction step into BRAM-friendly ring buffer
void buffer_push(
    DelayBuffer_t *buf,
    float timestamp_ms,
    const float x[STATE_DIM],
    const float U[STATE_DIM][STATE_DIM],
    const float D[STATE_DIM],
    float ax, float ay, float az, float wz, float dt
) {
    HistorySlot_t *slot = &buf->slots[buf->head];
    slot->timestamp_ms = timestamp_ms;
    memcpy(slot->x, x, sizeof(float) * STATE_DIM);
    memcpy(slot->U, U, sizeof(float) * STATE_DIM * STATE_DIM);
    memcpy(slot->D, D, sizeof(float) * STATE_DIM);
    slot->ax = ax;
    slot->ay = ay;
    slot->az = az;
    slot->wz = wz;
    slot->dt = dt;

    buf->head = (buf->head + 1) % BUFFER_CAPACITY;
    if (buf->count < BUFFER_CAPACITY) {
        buf->count++;
    }
}

// Lightweight kinematics propagation step used during replay (matches Person 1's predict block)
void propagate_imu_step(
    float x[STATE_DIM],
    float U[STATE_DIM][STATE_DIM],
    float D[STATE_DIM],
    float ax, float ay, float az, float wz, float dt
) {
    (void)U; // U remains unchanged under additive diagonal process noise approximation

    // Nominal state kinematic integration
    x[0] += x[3] * dt + 0.5f * ax * dt * dt;
    x[1] += x[4] * dt + 0.5f * ay * dt * dt;
    x[2] += x[5] * dt + 0.5f * az * dt * dt;
    x[3] += ax * dt;
    x[4] += ay * dt;
    x[5] += az * dt;
    x[6] = wrap_pi(x[6] + wz * dt);

    // Simplified diagonal process noise growth Q added directly to UD diagonal D
    const float q_pos = 0.001f * dt;
    const float q_vel = 0.01f * dt;
    const float q_yaw = 0.005f * dt;
    D[0] += q_pos; D[1] += q_pos; D[2] += q_pos;
    D[3] += q_vel; D[4] += q_vel; D[5] += q_vel;
    D[6] += q_yaw;
}

// Rewind to GNSS measurement timestamp, apply Bierman scalar updates, and replay IMU to present
bool apply_delayed_gnss_update(
    DelayBuffer_t *buf,
    float x_current[STATE_DIM],
    float U_current[STATE_DIM][STATE_DIM],
    float D_current[STATE_DIM],
    float meas_timestamp_ms,
    float gnss_x, float gnss_y, float gnss_z,
    float R_gnss_xy, float R_gnss_z,
    bool data_valid,
    float* nis_x_out
) {
    if (!data_valid || buf->count == 0) {
        return false;
    }

    // 1. Search ring buffer backwards to find slot closest to meas_timestamp_ms
    int best_idx = -1;
    int steps_back = 0;
    float min_time_diff = 1e9f;

    for (int k = 0; k < buf->count; k++) {
        int idx = (buf->head - 1 - k + BUFFER_CAPACITY) % BUFFER_CAPACITY;
        float diff = fabsf(buf->slots[idx].timestamp_ms - meas_timestamp_ms);
        if (diff < min_time_diff) {
            min_time_diff = diff;
            best_idx = idx;
            steps_back = k;
        }
    }

    // Reject if measurement is older than the entire buffer window (>320 ms)
    if (best_idx < 0 || min_time_diff > 20.0f) {
        return false;
    }

    // 2. REWIND: Load historical state & UD factors from the delayed timestamp
    float x_work[STATE_DIM];
    float U_work[STATE_DIM][STATE_DIM];
    float D_work[STATE_DIM];
    memcpy(x_work, buf->slots[best_idx].x, sizeof(float) * STATE_DIM);
    memcpy(U_work, buf->slots[best_idx].U, sizeof(float) * STATE_DIM * STATE_DIM);
    memcpy(D_work, buf->slots[best_idx].D, sizeof(float) * STATE_DIM);

    // 3. UPDATE: Apply sequential Bierman scalar updates for GNSS X, Y, Z at that past instant
    //[UPDATE: 23/11/2026 19.25] Add float_nis_x_out to its parameters and pass it to CH_GNSS_X:
    
    bool ux = update_scalar_bierman(x_work, U_work, D_work, gnss_x, R_gnss_xy, CH_GNSS_X, true, MAHALANOBIS_THRESH_1DOF, nis_x_out);
    bool uy = update_scalar_bierman(x_work, U_work, D_work, gnss_y, R_gnss_xy, CH_GNSS_Y, true, MAHALANOBIS_THRESH_1DOF, NULL);
    bool uz = update_scalar_bierman(x_work, U_work, D_work, gnss_z, R_gnss_z,  CH_GNSS_Z, true, MAHALANOBIS_THRESH_1DOF, NULL);

    if (!ux && !uy && !uz) {
        return false; // All channels rejected as outliers
    }

    // 4. REPLAY: Re-propagate stored IMU inputs forward from (best_idx + 1) up to present
    for (int k = steps_back; k >= 0; k--) {
        int idx = (buf->head - 1 - k + BUFFER_CAPACITY) % BUFFER_CAPACITY;
        HistorySlot_t *slot = &buf->slots[idx];

        // Overwrite historical snapshot with corrected trajectory
        memcpy(slot->x, x_work, sizeof(float) * STATE_DIM);
        memcpy(slot->U, U_work, sizeof(float) * STATE_DIM * STATE_DIM);
        memcpy(slot->D, D_work, sizeof(float) * STATE_DIM);

        // Propagate forward using the stored IMU sample
        propagate_imu_step(x_work, U_work, D_work, slot->ax, slot->ay, slot->az, slot->wz, slot->dt);
    }

    // Write final replayed state & UD factors back to live filter state
    memcpy(x_current, x_work, sizeof(float) * STATE_DIM);
    memcpy(U_current, U_work, sizeof(float) * STATE_DIM * STATE_DIM);
    memcpy(D_current, D_work, sizeof(float) * STATE_DIM);

    return true;
}

//Quick test harness to verify compilation and output for each step

//OUTPUT DEBUG FOR STEP 2: (19/11/2026)
/*Before Baro Update -> pz: 50.0000, P[2][2]: 4.0000
After Baro Update  -> pz: 51.9406, P[2][2]: 0.1189
*/

// int main(){
//     float x[STATE_DIM] = {10.0f, 0.0f, 50.0f, 1.0f, 0.0f, 0.0f, 0.0f};

//     float P[STATE_DIM][STATE_DIM] = {0};
//     for (int i = 0; i < STATE_DIM; i++) P[i][i] = 1.0f;
//     //high uncertainty in pz = 4.0
//     P[2][2] = 4.0f;

//     // Factorize initial P into U and D
//     float U[STATE_DIM][STATE_DIM];
//     float D[STATE_DIM];
//     cov_to_ud(P, U, D);

//     // --- Test Step 2: Bierman Barometer Scalar Update ---
//     float z_baro = 52.0f;
//     float R_baro = 0.35f * 0.35f;
    
//     printf("--- STEP 2 TEST (Bierman UD Update) ---\n");
//     ud_to_cov(U, D, P);
//     printf("Before Baro Update -> pz: %.4f, P[2][2]: %.4f\n", x[2], P[2][2]);
//     update_scalar_bierman(x, U, D, z_baro, R_baro, CH_BARO_Z, true, MAHALANOBIS_THRESH_1DOF);
//     ud_to_cov(U, D, P);
//     printf("After Baro Update  -> pz: %.4f, P[2][2]: %.4f\n\n", x[2], P[2][2]);

//     // --- Test Step 3: Asynchronous Gating & Pre-Replacement Outlier Rejection ---
//     printf("--- STEP 3 TEST (Gating Before Error Replacement) ---\n");
//     bool ok_skip = update_scalar_bierman(x, U, D, 10.5f, 2.25f, CH_GNSS_X, false, MAHALANOBIS_THRESH_1DOF);
//     printf("Async Skip (valid=false) -> Applied: %s | px: %.4f | D[0]: %.4f\n",
//            ok_skip ? "YES" : "NO", x[0], D[0]);

//     bool ok_outlier = update_scalar_bierman(x, U, D, 99.0f, 2.25f, CH_GNSS_X, true, MAHALANOBIS_THRESH_1DOF);
//     printf("Outlier Spike (z=99.0m)  -> Applied: %s | px: %.4f | D[0]: %.4f\n\n",
//            ok_outlier ? "YES" : "NO", x[0], D[0]);

//     // --- Test Step 4: Delayed-Aiding Ring Buffer (150 ms GNSS Transport Latency) ---
//     printf("--- STEP 4 TEST (150ms Delayed GNSS Rewind/Replay with UD) ---\n");
//     DelayBuffer_t delay_buf;
//     buffer_init(&delay_buf);

//     // Simulate 20 steps of 100Hz IMU propagation (t = 0 ms to 190 ms, moving at vx = 1.0 m/s)
//     float dt = 0.01f;
//     for (int step = 0; step < 20; step++) {
//         float t_ms = step * 10.0f;
//         buffer_push(&delay_buf, t_ms, x, U, D, 0.0f, 0.0f, 0.0f, 0.0f, dt);
//         propagate_imu_step(x, U, D, 0.0f, 0.0f, 0.0f, 0.0f, dt);
//     }

//     printf("Current Live State at t=200ms (before delayed GNSS): px = %.4f\n", x[0]);

//     // GNSS packet arrives at t=200ms, stamped from t=50ms (150ms latency) reporting px = 11.50m
//     bool ok_delayed = apply_delayed_gnss_update(
//         &delay_buf, x, U, D,
//         50.0f,              // Timestamp when GNSS was captured (150ms ago)
//         11.50f, 0.0f, x[2], // Measured X, Y, Z
//         2.25f, 7.84f,       // R_xy = 1.5^2, R_z = 2.8^2
//         true
//     );

//     printf("Delayed GNSS Applied: %s -> Replayed Live State at t=200ms: px = %.4f\n",
//            ok_delayed ? "YES" : "NO", x[0]);

//     return 0;
// }

/*UPDATE [23/11/2026] Testing artificial 3D ascending helix  trajectory .csv files into the functions, such as get_observation_model
and cov_to_ud*/

int main() {
    FILE *fin = fopen("flight_input.csv", "r");
    if (!fin) {
        printf("Error: Could not open flight_input.csv. Run prepare_trajectory.py first.\n");
        return 1;
    } else {
        printf("Successfully loggin in the trajectories dataset.\n");
    }

    FILE *fout = fopen("ekf_output.csv", "w");
    fprintf(fout, "t_ms,est_px,est_py,est_pz,est_yaw,gt_px,gt_py,gt_pz,gt_yaw,P00,P01,P02,P11,P12,P22,nis_baro,nis_gnss_x,gnss_valid,gnss_applied\n");

    char line[512];
    // Skip CSV header line
    if (!fgets(line, sizeof(line), fin)) {
        fclose(fin);
        fclose(fout);
        return 1;
    }

    float x[STATE_DIM] = {0};
    float P[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) P[i][i] = 1.0f;

    float U[STATE_DIM][STATE_DIM];
    float D[STATE_DIM];
    cov_to_ud(P, U, D);

    DelayBuffer_t delay_buf;
    buffer_init(&delay_buf);

    // Sensor noise variances matching your project specs
    const float R_baro    = 0.35f * 0.35f; // sigma = 0.35 m
    const float R_mag     = 0.05f * 0.05f; // sigma = 0.05 rad
    const float R_gnss_xy = 1.50f * 1.50f; // sigma = 1.50 m
    const float R_gnss_z  = 2.80f * 2.80f; // sigma = 2.80 m

    int step_count = 0;
    int gnss_accepted_count = 0;
    int gnss_total_count = 0;
    double sq_err_3d = 0.0;
    double sq_err_z  = 0.0;

    float t_ms, dt, ax, ay, az, wz;
    int baro_valid, mag_valid, gnss_valid;
    float baro_z, mag_yaw, gnss_ts_ms, gnss_x, gnss_y, gnss_z;
    float gt_px, gt_py, gt_pz, gt_yaw;

    while (fgets(line, sizeof(line), fin)) {
        int parsed = sscanf(
            line,
            "%f,%f,%f,%f,%f,%f,%d,%f,%d,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f",
            &t_ms, &dt, &ax, &ay, &az, &wz,
            &baro_valid, &baro_z, &mag_valid, &mag_yaw,
            &gnss_valid, &gnss_ts_ms, &gnss_x, &gnss_y, &gnss_z,
            &gt_px, &gt_py, &gt_pz, &gt_yaw
        );
        if (parsed != 19) continue;

        // Initialize state x on the very first frame from ground truth
        if (step_count == 0) {
            x[0] = gt_px;
            x[1] = gt_py;
            x[2] = gt_pz;
            x[3] = 0.0f;
            x[4] = 6.0f; // Initial tangential velocity for helix (R*omega = 15*0.4)
            x[5] = 0.5f;
            x[6] = gt_yaw;
        }
        
        float nis_baro = 1.0f;
        float nis_gnss_x = -1.0f;

        // 1. Log current state & IMU input to ring buffer, then propagate forward
        buffer_push(&delay_buf, t_ms, x, U, D, ax, ay, az, wz, dt);
        propagate_imu_step(x, U, D, ax, ay, az, wz, dt);

        // 2. Asynchronous fast sensor updates (Barometer & Magnetometer)
        update_scalar_bierman(x, U, D, baro_z,  R_baro, CH_BARO_Z,  baro_valid == 1, MAHALANOBIS_THRESH_1DOF, &nis_baro);
        update_scalar_bierman(x, U, D, mag_yaw, R_mag,  CH_MAG_YAW, mag_valid == 1,  MAHALANOBIS_THRESH_1DOF, NULL);

        // 3. Delayed GNSS Rewind & Replay update (100-200 ms transport latency)
        bool gnss_applied = false;
        if (gnss_valid == 1) {
            gnss_total_count++;
            gnss_applied = apply_delayed_gnss_update(
                &delay_buf, x, U, D,
                gnss_ts_ms, gnss_x, gnss_y, gnss_z,
                R_gnss_xy, R_gnss_z, true, &nis_gnss_x
            );
            if (gnss_applied) gnss_accepted_count++;
        }

        ud_to_cov(U, D, P);

        // 4. Accumulate RMSE metrics against Ground Truth
        double ex = x[0] - gt_px;
        double ey = x[1] - gt_py;
        double ez = x[2] - gt_pz;
        sq_err_3d += (ex * ex + ey * ey + ez * ez);
        sq_err_z  += (ez * ez);
        step_count++;

        // 5. Log trajectory to CSV for Python/MATLAB plotting
        fprintf(fout, "%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.4f,%.4f,%d,%d\n",
        t_ms, x[0], x[1], x[2], x[6], gt_px, gt_py, gt_pz, gt_yaw,
        P[0][0], P[0][1], P[0][2], P[1][1], P[1][2], P[2][2], //the NIS columns haven't been added
        nis_baro, nis_gnss_x, gnss_valid, gnss_applied ? 1 : 0);
    }

    fclose(fin);
    fclose(fout);

    if (step_count > 0) {
        double rmse_3d = sqrt(sq_err_3d / step_count);
        double rmse_z  = sqrt(sq_err_z / step_count);
        
        printf("=== TRAJECTORY VERIFICATION SUMMARY ===\n");
        printf("Total IMU Steps Processed : %d\n", step_count);
        printf("GNSS Packets Accepted     : %d / %d (Outliers rejected by Mahalanobis gate)\n",
               gnss_accepted_count, gnss_total_count);
        printf("3D Position RMSE          : %.4f m\n", rmse_3d);
        printf("Altitude (Z) RMSE         : %.4f m\n", rmse_z);
        printf("Detailed log saved to     : ekf_output.csv\n");
    }

    fflush(stdout);

    return 0;
}