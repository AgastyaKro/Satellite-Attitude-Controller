import matplotlib.pyplot as plt
import re

# Parse SatelliteDebug.csv manually
times = []
errors_y = []

with open("SatelliteDebug.csv", "r") as f:
    for line in f:
        if "t=" in line and "error=[" in line:
            try:
                # Extract time
                t_match = re.search(r't=([\d\.]+)', line)
                t = float(t_match.group(1))

                # Extract error (6D), take 2nd component (e.g., y-axis orientation error)
                err_match = re.search(r'error=\[([^\]]+)\]', line)
                err_vals = list(map(float, err_match.group(1).split()))
                error_y = err_vals[1]  # or index 4 if you're using [ω; θ_err] order

                times.append(t)
                errors_y.append(error_y)
            except Exception as e:
                print(f"Failed to parse line: {line.strip()} | {e}")

# Plot
plt.figure(figsize=(10, 5))
plt.plot(times, errors_y, label="Orientation Error (Y-axis)")
plt.xlabel("Time (s)")
plt.ylabel("Angle Error (rad)")
plt.title("Satellite Orientation Error Over Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("orientation_error_plot.png")
plt.show()
