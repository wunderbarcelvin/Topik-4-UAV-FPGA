#include "ekf_update_ip.h"
//#include <hls_math.h>

static data_t x_reg[STATE_DIM];
static data_t U_reg[STATE_DIM][STATE_DIM];
static data_t D_reg[STATE_DIM];
//internal persistent hardware state

typedef struct {
    data_t timestamp_ms;
    data_t x[STATE_DIM];
    data_t U[STATE_DIM][STATE_DIM];
    data_t D[STATE_DIM];
    data_t ax, ay, az;
    data_t wz;
    data_t dt;
} HistorySlot_t;

static HistorySlot_t delay_buf[BUFFER_CAPACITY];
static int buf_head = 0;
static int buf_count = 0;

static data_t wrap_pi(data_t angle){
    const data_t PI = 3.14159265f;
    const data_t TWO_PI = 6.28318530f;
    if (angle > PI)  angle -= TWO_PI;
    if (angle < -PI) angle += TWO_PI;
    return angle;
}

static void get_observation_model(
    const data_t x[STATE_DIM],
    uint8_t channel,
    data_t *h_x,
    data_t H_row[STATE_DIM]
){
    #pragma HLS INLINE
    for (int i = 0; i < STATE_DIM; i++)
    {
        #pragma HLS UNROLL
        H_row[i] = 0.00f;
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
        default:
            *h_x = 0.0f;
            break;
    }
    
}

//PART 2 & 3 : Bierman scalar update with Pre-Replacement Outlier Gating

static bool update_scalar_bierman(
    data_t x[STATE_DIM],
    data_t U [STATE_DIM][STATE_DIM],
    data_t D[STATE_DIM],
    data_t z_meas,
    data_t R_var,
    uint8_t channel,
    bool data_valid,
    data_t gate_threshold,
    data_t *nis_val
){
    *nis_val = -1.0f;
    if (!data_valid)
    {
        return false;
    }
    
    data_t h_x = 0.0f;
    data_t H[STATE_DIM];
    data_t f[STATE_DIM];
    data_t v[STATE_DIM];
    #pragma HLS ARRAY_PARTITION variable=H complete
    #pragma HLS ARRAY_PARTITION variable=f complete
    #pragma HLS ARRAY_PARTITION variable=v complete

    get_observation_model(x, channel, &h_x, H);

    // Step 1: f = U^T * H^T and v = D * f
    for (int j = 0; j < STATE_DIM; j++)
    {
        #pragma HLS UNROLL
        data_t sum_f = H[j];
        for (int i = 0; i < j; i++)
        {
            #pragma HLS UNROLL
            sum_f += U[i][j] * H[i];
        }
        f[j] = sum_f;
        v[j] = D[j]*sum_f;
        
    }

    // Step 2: Innovation Covariance S = R + f^T * v
    data_t S = R_var;
    for (int j = 0; j < STATE_DIM; j++) {
        #pragma HLS UNROLL
        S += f[j] * v[j];
    }

    if (S <= 1e-9f || R_var <= 1e-9f) {
        return false;
    }
    
    //residual y = z - h(x)
    data_t y_residual = z_meas - h_x;
    if (channel == CH_MAG_YAW)
    {   
        y_residual = wrap_pi(y_residual);
    }

    data_t y_sq = y_residual * y_residual;
    *nis_val = y_sq/S;

    if (y_sq > (gate_threshold*S))
    {
        return false;
    }
    
    //Bierman Rank-1 UD Covariance Update
    data_t alpha = R_var;
    for (int j = 0; j < STATE_DIM; j++)
    {
        data_t alpha_prev = alpha;
        data_t fj = f[j];
        data_t vj = v[j];

        alpha = alpha_prev + fj*vj;
        D[j] = D[j] * (alpha_prev/alpha);

        data_t lambda = -fj/alpha_prev;
        for (int i = 0; i < j; i++)
        {
            #pragma HLS UNROLL
            data_t u_ij_old = U[i][j];
            U[i][j] = u_ij_old + v[i]*lambda;
            v[i] = v[i] + u_ij_old * vj;
        }
        
    }
    
    //state correction x = x + (v/alpha) * y
    data_t scale = y_residual/alpha;
    for (int i = 0; i < STATE_DIM; i++)
    {
        #pragma HLS UNROLL
        x[i] += v[i]*scale;
    }

    return true;   
}


//kinematic propagation step
static void propagate_imu_step(
    data_t x[STATE_DIM],
    data_t D[STATE_DIM],
    data_t ax, data_t ay, data_t az, data_t wz, data_t dt
){
    #pragma HLS INLINE
    data_t half_dt2 = 0.5f * dt * dt;
    x[0] += x[3] * dt + ax * half_dt2;
    x[1] += x[4] * dt + ay * half_dt2;
    x[2] += x[5] * dt + az * half_dt2;
    x[3] += ax * dt;
    x[4] += ay * dt;
    x[5] += az * dt;
    x[6] = wrap_pi(x[6] + wz * dt);

    const data_t q_pos = 0.001f * dt;
    const data_t q_vel = 0.01f * dt;
    const data_t q_yaw = 0.005f * dt;
    D[0] += q_pos; D[1] += q_pos; D[2] += q_pos; 
    D[3] += q_vel; D[4] += q_vel; D[5] += q_vel; 
    D[6] += q_yaw;
}

//push IMU step into BRAM ring buffer
static void buffer_push_hw(
    data_t timestamp_ms,
    data_t ax, data_t ay, data_t az, data_t wz, data_t dt
){
    delay_buf[buf_head].ax = ax;
    delay_buf[buf_head].ay = ay;
    delay_buf[buf_head].az = az;
    delay_buf[buf_head].wz = wz;
    delay_buf[buf_head].dt = dt;

    for (int i = 0; i < STATE_DIM; i++)
    {
        #pragma HLS UNROLL
        delay_buf[buf_head].x[i] = x_reg[i];
        delay_buf[buf_head].D[i] = D_reg[i];

        for (int j = 0; j < STATE_DIM; j++)
        {
            #pragma HLS UNROLL
            delay_buf[buf_head].U[i][j] = U_reg[i][j];
        }
        
    }

    buf_head = (buf_head + 1) % BUFFER_CAPACITY;
    if (buf_count < BUFFER_CAPACITY) {
        buf_count++;        
    }
}

static bool apply_delayed_gnss_hw(
    data_t meas_timestamp_ms,
    data_t gnss_x, data_t gnss_y, data_t gnss_z,
    data_t R_gnss_xy, data_t R_gnss_z,
    bool data_valid,
    data_t *nis_x_val
){
    *nis_x_val = -1.0f;
    if (!data_valid || buf_count == 0) {
        return false;
    }

    int best_idx = -1;
    int steps_back = 0;
    data_t min_time_diff = 1e9f;
    
    for (int k = 0; k < BUFFER_CAPACITY; k++) {
        if (k < buf_count) {
            int idx = (buf_head - 1 - k + BUFFER_CAPACITY) % BUFFER_CAPACITY;
            data_t diff = hls::fabsf(delay_buf[idx].timestamp_ms - meas_timestamp_ms);
            if (diff < min_time_diff) {
                min_time_diff = diff;
                best_idx = idx;
                steps_back = k;
            }
        }
    }

    if (best_idx < 0 || min_time_diff > 20.0f)
    {
        return false;
    }
    
    //rewind snapshots into local registers
    data_t x_work[STATE_DIM];
    data_t U_work[STATE_DIM][STATE_DIM];
    data_t D_work[STATE_DIM];
    #pragma HLS ARRAY_PARTITION variable=x_work complete
    #pragma HLS ARRAY_PARTITION variable=D_work complete
    #pragma HLS ARRAY_PARTITION variable=U_work complete dim=2

    for (int i = 0; i < STATE_DIM; i++)
    {
        #pragma HLS UNROLL
        x_work[i] = delay_buf[best_idx].x[i];
        D_work[i] = delay_buf[best_idx].D[i];
        for (int j = 0; j < STATE_DIM; j++)
        {
            #pragma HLS UNROLL
            U_work[i][j] = delay_buf[best_idx].U[i][j];
        }
    }
    
    //sequential bierman updates at delayed epoch
    data_t dummy_nis;
    bool ux = update_scalar_bierman(x_work, U_work, D_work, gnss_x, R_gnss_xy, CH_GNSS_X, true, MAHALANOBIS_THRESH_1DOF, nis_x_val);
    bool uy = update_scalar_bierman(x_work, U_work, D_work, gnss_x, R_gnss_xy, CH_GNSS_Y, true, MAHALANOBIS_THRESH_1DOF, &dummy_nis);
    bool uz = update_scalar_bierman(x_work, U_work, D_work, gnss_x, R_gnss_z, CH_GNSS_Z, true, MAHALANOBIS_THRESH_1DOF, &dummy_nis);

    if (!ux && !uy && !uy)
    {
        return false;
    }

    //replay stored IMU steps back to the present
    for (int k = steps_back; k >= 0; k--) {
        int idx = (buf_head - 1 - k + BUFFER_CAPACITY) % BUFFER_CAPACITY;
        for (int i = 0; i < STATE_DIM; i++) {
            #pragma HLS UNROLL
            delay_buf[idx].x[i] = x_work[i];
            delay_buf[idx].D[i] = D_work[i];
            for (int j = 0; j < STATE_DIM; j++) {
                #pragma HLS UNROLL
                delay_buf[idx].U[i][j] = U_work[i][j];
            }
        }
        propagate_imu_step(
            x_work, D_work,
            delay_buf[idx].ax, delay_buf[idx].ay, delay_buf[idx].az,
            delay_buf[idx].wz, delay_buf[idx].dt
        );
    }

    //commit replayed trajectory to live registers
    for (int i = 0; i < STATE_DIM; i++)
    {
        #pragma HLS UNROLL
        x_reg[i] = x_work[i];
        D_reg[i] = D_work[i];

        for (int j = 0; j < STATE_DIM; j++)
        {
            #pragma HLS UNROLL
            U_reg[i][j] = U_work[i][j];
        }
        
    }
    
    return true;
    
}


void ekf_meas_top(
    uint8_t mode,
    uint8_t channel,
    bool data_valid,
    data_t timestamp_ms,
    data_t dt,
    data_t z_meas,
    data_t R_var,
    data_t ax, data_t ay, data_t az, data_t wz,
    data_t gnss_x, data_t gnss_y, data_t gnss_z,
    data_t R_gnss_xy, data_t R_gnss_z,
    const data_t x_init[STATE_DIM],
    data_t x_out[STATE_DIM],
    data_t P_diag_out[STATE_DIM],
    data_t *nis_out,
    bool *update_applied
){
    / AXI4-Lite Slave Interfaces for ARM PS Control & Register I/O
    #pragma HLS INTERFACE s_axilite port=return bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=mode bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=channel bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=data_valid bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=timestamp_ms bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=dt bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=z_meas bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=R_var bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=ax bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=ay bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=az bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=wz bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=gnss_x bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=gnss_y bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=gnss_z bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=R_gnss_xy bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=R_gnss_z bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=x_init bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=x_out bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=P_diag_out bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=nis_out bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=update_applied bundle=CTRL

    // Memory Partitioning & On-Chip BRAM Mapping
    #pragma HLS ARRAY_PARTITION variable=x_reg complete
    #pragma HLS ARRAY_PARTITION variable=D_reg complete
    #pragma HLS ARRAY_PARTITION variable=U_reg complete dim=2
    #pragma HLS BIND_STORAGE variable=delay_buf type=ram_2p impl=bram
    
    data_t local_nis = 1.0f;
    bool local_applied = false;

    switch (mode)
    {
    case MODE_INIT:
        buf_head = 0;
        buf_count = 0;
        for (int i = 0; i < STATE_DIM; i++)
        {
            #pragma HLS UNROLL
            x_reg[i] = x_init[i];
            D_reg[i] = 1.0f;
            for (int j = 0; j < STATE_DIM; j++) {
                #pragma HLS UNROLL
                U_reg[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        local_applied = true;
        
        break;
    case MODE_PUSH_IMU:
        buffer_push_hw(timestamp_ms, ax, ay, az, wz, dt);
        propagate_imu_step(x_reg, D_reg, ax, ay, az, wz, dt);
        local_applied = true;
        break;
    
    case MODE_UPDATE_SCALAR:
        localdw
    
    default:
        break;
    }
}