import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

df = pd.read_csv("output.csv")

# Plot angle error
plt.figure()
plt.plot(df['time'], df['angleError'], label='Angle Error (rad)')
plt.xlabel("Time (s)")
plt.ylabel("Angle Error (rad)")
plt.title("Angle Error Over Time")
plt.grid(True)
plt.legend()
plt.tight_layout()

# Plot angular velocity
plt.figure()
plt.plot(df['time'], df['wx'], label='ω_x')
plt.plot(df['time'], df['wy'], label='ω_y')
plt.plot(df['time'], df['wz'], label='ω_z')
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Angular Velocity Components")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()
