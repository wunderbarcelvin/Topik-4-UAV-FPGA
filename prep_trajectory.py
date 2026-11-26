import numpy as np
import pandas as pd

#PURPOSE: CONVERT 3D ASCENDING HELIX TRAJECTORY INTO SINGLE STREAM, TIME-ALIGNED  FLIGHT INPUT .CSV FILE containing 100 Hz IMU data, 25 Hz Barometer, 
# magnetometer, delayed 10 Hz GNSS (with 150 ms latency, outlier spikes, and dropouts), and ground truth.

def generate_synthetic_helix(duration_s=20.0, fs_imu=100.0):
    dt = 1.0 / fs_imu
    N = int(duration_s * fs_imu)
    t = np.arange(N) * dt
    t_ms = t * 1000.0

    # 3D Ascending Helix Ground Truth (matching 7-state UAV scenario)
    R_helix, omega, vz_climb = 15.0, 0.4, 0.5
    gt_px = R_helix * np.cos(omega * t)
    gt_py = R_helix * np.sin(omega * t)
    gt_pz = 10.0 + vz_climb * t
    gt_vx = -R_helix * omega * np.sin(omega * t)
    gt_vy =  R_helix * omega * np.cos(omega * t)
    gt_vz = np.full(N, vz_climb)
    gt_yaw = np.arctan2(gt_vy, gt_vx)

    # World-frame accelerations & yaw rate + IMU noise
    ax = -R_helix * (omega**2) * np.cos(omega * t) + np.random.normal(0, 0.05, N)
    ay = -R_helix * (omega**2) * np.sin(omega * t) + np.random.normal(0, 0.05, N)
    az = np.random.normal(0, 0.05, N)
    wz = np.full(N, omega) + np.random.normal(0, 0.005, N)

    rows = []
    gnss_queue = [] # Simulates 150 ms transport latency buffer

    for k in range(N):
        # Barometer at 25 Hz (every 4 IMU steps), sigma = 0.35 m
        baro_valid = 1 if (k % 4 == 0) else 0
        baro_z = gt_pz[k] + np.random.normal(0, 0.35) if baro_valid else 0.0

        # Magnetometer at 20 Hz (every 5 IMU steps), sigma = 0.05 rad
        mag_valid = 1 if (k % 5 == 0) else 0
        mag_yaw = gt_yaw[k] + np.random.normal(0, 0.05) if mag_valid else 0.0

        # Capture GNSS at 10 Hz (every 10 IMU steps), delayed by 15 steps (150 ms)
        if k % 10 == 0:
            # Inject GNSS dropout between t = 8.0s and 10.0s
            if not (8.0 <= t[k] <= 10.0):
                gx = gt_px[k] + np.random.normal(0, 1.5)
                gy = gt_py[k] + np.random.normal(0, 1.5)
                gz = gt_pz[k] + np.random.normal(0, 2.8)
                # Inject a massive multipath outlier spike at t = 14.0s
                if abs(t[k] - 14.0) < 1e-3:
                    gx += 80.0
                gnss_queue.append((k + 15, t_ms[k], gx, gy, gz))

        # Check if a delayed GNSS packet arrives at step k
        gnss_valid, gnss_ts, gnss_x, gnss_y, gnss_z = 0, 0.0, 0.0, 0.0, 0.0
        if gnss_queue and gnss_queue[0][0] == k:
            _, gnss_ts, gnss_x, gnss_y, gnss_z = gnss_queue.pop(0)
            gnss_valid = 1

        rows.append([
            t_ms[k], dt, ax[k], ay[k], az[k], wz[k],
            baro_valid, baro_z, mag_valid, mag_yaw,
            gnss_valid, gnss_ts, gnss_x, gnss_y, gnss_z,
            gt_px[k], gt_py[k], gt_pz[k], gt_yaw[k]
        ])

    cols = [
        "t_ms", "dt", "ax", "ay", "az", "wz",
        "baro_valid", "baro_z", "mag_valid", "mag_yaw",
        "gnss_valid", "gnss_ts_ms", "gnss_x", "gnss_y", "gnss_z",
        "gt_px", "gt_py", "gt_pz", "gt_yaw"
    ]
    pd.DataFrame(rows, columns=cols).to_csv("flight_input.csv", index=False)
    print(f"Saved {N} steps to flight_input.csv")


#if wanna use datases with real error https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f

def convert_euroc_to_csv(imu_csv_path, gt_csv_path):
    """Converts EuRoC MAV imu0/data.csv & state_groundtruth_estimate0/data.csv to flight_input.csv"""
    imu = pd.read_csv(imu_csv_path)
    gt = pd.read_csv(gt_csv_path)
    # Rename standard EuRoC headers (strip leading '#')
    imu.columns = [c.strip().lstrip('#') for c in imu.columns]
    gt.columns = [c.strip().lstrip('#') for c in gt.columns]

    # Align ground truth timestamps to IMU timestamps via nearest-neighbor merge
    imu = imu.sort_values('timestamp [ns]')
    gt = gt.sort_values('timestamp')
    merged = pd.merge_asof(imu, gt, left_on='timestamp [ns]', right_on='timestamp', direction='nearest')

    # Downsample 200 Hz EuRoC IMU to 100 Hz (every 2nd sample)
    df = merged.iloc[::2].reset_index(drop=True)
    t_ms = (df['timestamp [ns]'] - df['timestamp [ns]'].iloc[0]) * 1e-6
    N = len(df)
    dt = 0.01

    # Extract EuRoC ground-truth position & compute yaw from quaternion (w, x, y, z)
    gt_px = df['p_RS_R_x [m]'].values
    gt_py = df['p_RS_R_y [m]'].values
    gt_pz = df['p_RS_R_z [m]'].values
    qw, qx, qy, qz = df['q_RS_w []'].values, df['q_RS_x []'].values, df['q_RS_y []'].values, df['q_RS_z []'].values
    gt_yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

    # Rotate body-frame specific force to world frame & subtract gravity (9.81 m/s^2)
    ax_b, ay_b, az_b = df['a_RS_S_x [m s^-2]'].values, df['a_RS_S_y [m s^-2]'].values, df['a_RS_S_z [m s^-2]'].values
    wz = df['w_RS_S_z [rad s^-1]'].values
    ax = ax_b * np.cos(gt_yaw) - ay_b * np.sin(gt_yaw)
    ay = ax_b * np.sin(gt_yaw) + ay_b * np.cos(gt_yaw)
    az = az_b - 9.81

    # Synthesize asynchronous Baro (25Hz), Mag (20Hz), and 150ms-delayed GNSS (10Hz) from ground truth
    rows, gnss_queue = [], []
    for k in range(N):
        baro_valid = 1 if (k % 4 == 0) else 0
        baro_z = gt_pz[k] + np.random.normal(0, 0.35) if baro_valid else 0.0
        mag_valid = 1 if (k % 5 == 0) else 0
        mag_yaw = gt_yaw[k] + np.random.normal(0, 0.05) if mag_valid else 0.0

        if k % 10 == 0:
            gx = gt_px[k] + np.random.normal(0, 1.5)
            gy = gt_py[k] + np.random.normal(0, 1.5)
            gz = gt_pz[k] + np.random.normal(0, 2.8)
            gnss_queue.append((k + 15, t_ms.iloc[k], gx, gy, gz))

        gnss_valid, gnss_ts, gnss_x, gnss_y, gnss_z = 0, 0.0, 0.0, 0.0, 0.0
        if gnss_queue and gnss_queue[0][0] == k:
            _, gnss_ts, gnss_x, gnss_y, gnss_z = gnss_queue.pop(0)
            gnss_valid = 1

        rows.append([
            t_ms.iloc[k], dt, ax[k], ay[k], az[k], wz[k],
            baro_valid, baro_z, mag_valid, mag_yaw,
            gnss_valid, gnss_ts, gnss_x, gnss_y, gnss_z,
            gt_px[k], gt_py[k], gt_pz[k], gt_yaw[k]
        ])

    cols = [
        "t_ms", "dt", "ax", "ay", "az", "wz",
        "baro_valid", "baro_z", "mag_valid", "mag_yaw",
        "gnss_valid", "gnss_ts_ms", "gnss_x", "gnss_y", "gnss_z",
        "gt_px", "gt_py", "gt_pz", "gt_yaw"
    ]
    pd.DataFrame(rows, columns=cols).to_csv("flight_input.csv", index=False)
    print(f"Converted {N} EuRoC steps to flight_input.csv")

if __name__ == "__main__":
    np.random.seed(42)
    generate_synthetic_helix()
    # To use downloaded EuRoC MAV data instead, uncomment below:
    # convert_euroc_to_csv("mav0/imu0/data.csv", "mav0/state_groundtruth_estimate0/data.csv")