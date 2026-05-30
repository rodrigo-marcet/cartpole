import numpy as np
import matplotlib.pyplot as plt

WHEEL_RADIUS = 0.01
CART_MASS = 0.2

def load_file(path):
    times, velocities = [], []
    t = 0.0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            dt = float(parts[0])
            vel_tps = float(parts[1])
            t += dt
            vel_ms = abs(vel_tps) * 2 * np.pi * WHEEL_RADIUS
            times.append(t)
            velocities.append(vel_ms)
    return np.array(times), np.array(velocities)

def exponential(t, v0, tau):
    return v0 * np.exp(-t / tau)

files = {"10ts": "10ts.txt", "20ts": "20ts.txt"}

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

for ax, (label, path) in zip(axes, files.items()):
    times, velocities = load_file(path)

    # Trim tail below 0.05 m/s
    mask = velocities > 0.05
    t_fit = times[mask] - times[mask][0]
    v_fit = velocities[mask]

    # Linear fit on ln(v) — robust, no curve_fit needed
    ln_v = np.log(v_fit)
    slope, intercept = np.polyfit(t_fit, ln_v, 1)

    tau = -1.0 / slope
    v0 = np.exp(intercept)
    b = CART_MASS / tau

    print(f"[{label}]")
    print(f"  v0  = {v0:.4f} m/s")
    print(f"  tau = {tau:.4f} s")
    print(f"  b   = {b:.4f} N·s/m")
    print()

    # Plot
    t_plot = times - times[mask][0]
    ax.plot(t_plot, velocities, 'o', markersize=4, label="raw data")
    t_curve = np.linspace(0, t_fit.max(), 200)
    ax.plot(t_curve, exponential(t_curve, v0, tau), '--', label=f"fit: b={b:.4f}")
    ax.axhline(0.05, color='red', linestyle=':', linewidth=0.8, label="cutoff")
    ax.set_title(label)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("coast_down_plot.png", dpi=150)
plt.show()
