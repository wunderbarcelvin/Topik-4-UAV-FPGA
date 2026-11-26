import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import chi2

# 1. Load C testbench output 
df = pd.read_csv("ekf_output.csv")
t_sec = df["t_ms"] / 1000.0

# 2. Compute RMSE (3D Position & Altitude Z)
err_x = df["est_px"] - df["gt_px"]
err_y = df["est_py"] - df["gt_py"]
err_z = df["est_pz"] - df["gt_pz"]

rmse_3d = np.sqrt(np.mean(err_x**2 + err_y**2 + err_z**2))
rmse_z  = np.sqrt(np.mean(err_z**2))

# 3. Compute 3-DOF Position NEES: e^T * P_pos^{-1} * e
nees_list = []
for _, row in df.iterrows():
    e = np.array([
        row["est_px"] - row["gt_px"],
        row["est_py"] - row["gt_py"],
        row["est_pz"] - row["gt_pz"]
    ], dtype=np.float64)

    P_pos = np.array([
        [row["P00"], row["P01"], row["P02"]],
        [row["P01"], row["P11"], row["P12"]],
        [row["P02"], row["P12"], row["P22"]]
    ], dtype=np.float64)


    P_pos = 0.5 * (P_pos + P_pos.T) + 1e-12 * np.eye(3)
    sol, _, _, _ = np.linalg.lstsq(P_pos, e, rcond=None)
    nees_list.append(float(e.T @ sol))

df["nees_3d"] = nees_list
mean_nees = np.mean(df["nees_3d"])

# 95% confidence bounds for Chi-Square with 3 degrees of freedom (3D position)
nees_lower, nees_upper = chi2.ppf([0.025, 0.975], df=3)

# 4. Analyze NIS & Mahalanobis Gate Rejection Rate (gamma = 9.0 -> 3-sigma / 99.7%)
gamma = 9.0
baro_nis = df[df["nis_baro"] >= 0]["nis_baro"]
gnss_nis = df[df["nis_gnss_x"] >= 0]["nis_gnss_x"]

# Separate nominal GNSS samples from injected outlier spikes (NIS > 50)
gnss_nominal = gnss_nis[gnss_nis <= 50.0]
gnss_spikes  = gnss_nis[gnss_nis > 50.0]

baro_rej_pct = 100.0 * np.mean(baro_nis > gamma)
gnss_nom_rej_pct = 100.0 * np.mean(gnss_nominal > gamma)
gnss_spike_rej_pct = 100.0 * np.mean(gnss_spikes > gamma) if len(gnss_spikes) > 0 else 100.0

print("==========================================")
print("   EKF STATISTICAL CONSISTENCY REPORT     ")
print("==========================================")
print(f"3D Position RMSE          : {rmse_3d:.4f} m")
print(f"Altitude (Z) RMSE         : {rmse_z:.4f} m")
print(f"Mean 3-DOF NEES (ideal=3) : {mean_nees:.4f} (95% CI: [{nees_lower:.2f}, {nees_upper:.2f}])")
print(f"Baro Nominal Rejection    : {baro_rej_pct:.2f}% (Theoretical @ gamma=9.0: 0.27%)")
print(f"GNSS Nominal Rejection    : {gnss_nom_rej_pct:.2f}% (Theoretical @ gamma=9.0: 0.27%)")
print(f"GNSS Injected Spikes Rej. : {gnss_spike_rej_pct:.1f}% ({len(gnss_spikes)}/{len(gnss_spikes)} caught)")
print("==========================================")

# 5. Generate Verification Plots
fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

# Plot A: Altitude (Z) & 3D Position Error
axes[0].plot(t_sec, df["gt_pz"], "k--", label="Ground Truth Z (m)")
axes[0].plot(t_sec, df["est_pz"], "b-", label=f"EKF Estimated Z (RMSE={rmse_z:.3f} m)")
axes[0].axvspan(8.0, 10.0, color="orange", alpha=0.2, label="GNSS Dropout Window")
axes[0].set_ylabel("Altitude Z [m]")
axes[0].set_title("Altitude Tracking & GNSS Dropout Resilience")
axes[0].legend(loc="upper left")
axes[0].grid(True, alpha=0.3)

# Plot B: 3-DOF Position NEES Consistency
axes[1].plot(t_sec, df["nees_3d"], "g-", linewidth=1.2, label=f"3-DOF Position NEES (Mean={mean_nees:.2f})")
axes[1].axhline(nees_upper, color="r", linestyle="--", label=f"95% Chi2_3 Upper Bound ({nees_upper:.2f})")
axes[1].axhline(nees_lower, color="r", linestyle=":", label=f"95% Chi2_3 Lower Bound ({nees_lower:.2f})")
axes[1].set_ylabel("NEES (3-DOF)")
axes[1].set_title("Normalized Estimation Error Squared (NEES) Consistency")
axes[1].legend(loc="upper right")
axes[1].grid(True, alpha=0.3)

# Plot C: NIS & Mahalanobis Gate (gamma = 9.0)
baro_df = df[df["nis_baro"] >= 0]
gnss_df = df[df["nis_gnss_x"] >= 0]
axes[2].scatter(baro_df["t_ms"]/1000.0, baro_df["nis_baro"], s=10, alpha=0.6, label="Baro NIS")
axes[2].scatter(gnss_df["t_ms"]/1000.0, np.clip(gnss_df["nis_gnss_x"], 0, 15), s=22, color="purple", label="GNSS X NIS (clipped at 15)")
axes[2].axhline(gamma, color="r", linestyle="--", linewidth=1.5, label="Mahalanobis Gate Threshold (gamma = 9.0)")
axes[2].set_xlabel("Time [s]")
axes[2].set_ylabel("NIS (y^2 / S)")
axes[2].set_title("Normalized Innovation Squared (NIS) & Outlier Rejection")
axes[2].legend(loc="upper right")
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("ekf_verification_plots.png", dpi=200)
plt.show()