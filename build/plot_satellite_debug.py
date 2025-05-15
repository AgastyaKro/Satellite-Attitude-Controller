import pandas as pd
import matplotlib.pyplot as plt
import re

# Parse debug log
def parse_debug_log(file):
    with open(file, 'r') as f:
        lines = [line.strip() for line in f if line.startswith('t=')]

    data = {
        'time': [], 'qw': [], 'qx': [], 'qy': [], 'qz': [],
        'wx': [], 'wy': [], 'wz': [],
        'torque_x': [], 'torque_y': [], 'torque_z': [],
        'error_norm': []
    }

    for line in lines:
        try:
            t_match = re.search(r't=([0-9.]+)', line)
            q_match = re.search(r'q=\[([^\]]+)\]', line)
            w_match = re.search(r'w=\[([^\]]+)\]', line)
            torque_match = re.search(r'torque=\[([^\]]+)\]', line)
            err_match = re.search(r'error=\[([^\]]+)\]', line)

            time = float(t_match.group(1))
            qw, qx, qy, qz = map(float, q_match.group(1).split(','))
            wx, wy, wz = map(float, w_match.group(1).split(','))
            tx, ty, tz = map(float, torque_match.group(1).split())
            error_vec = list(map(float, err_match.group(1).split()))
            error_norm = sum(e**2 for e in error_vec) ** 0.5

            data['time'].append(time)
            data['qw'].append(qw)
            data['qx'].append(qx)
            data['qy'].append(qy)
            data['qz'].append(qz)
            data['wx'].append(wx)
            data['wy'].append(wy)
            data['wz'].append(wz)
            data['torque_x'].append(tx)
            data['torque_y'].append(ty)
            data['torque_z'].append(tz)
            data['error_norm'].append(error_norm)
        except Exception as e:
            print("Skipping line:", line, "\nError:", e)
            continue

    return pd.DataFrame(data)


df = parse_debug_log("SatelliteDebug.csv")

# Plot orientation quaternion components
plt.figure()
plt.plot(df['time'], df['qw'], label='qw')
plt.plot(df['time'], df['qx'], label='qx')
plt.plot(df['time'], df['qy'], label='qy')
plt.plot(df['time'], df['qz'], label='qz')
plt.title('Quaternion Components Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Quaternion')
plt.legend()
plt.grid()

# Plot angular velocity
plt.figure()
plt.plot(df['time'], df['wx'], label='wx')
plt.plot(df['time'], df['wy'], label='wy')
plt.plot(df['time'], df['wz'], label='wz')
plt.title('Angular Velocity Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()
plt.grid()

# Plot torque
plt.figure()
plt.plot(df['time'], df['torque_x'], label='torque_x')
plt.plot(df['time'], df['torque_y'], label='torque_y')
plt.plot(df['time'], df['torque_z'], label='torque_z')
plt.title('Torque Applied Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Torque (Nm)')
plt.legend()
plt.grid()

# Plot error norm
plt.figure()
plt.plot(df['time'], df['error_norm'], label='error_norm')
plt.title('Total State Error Norm Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Error Norm')
plt.legend()
plt.grid()

plt.show()
