import pandas as pd
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# 1. Load your CSV (skip the noise)
df = pd.read_csv("pid_dump.csv", skiprows=5, delimiter=",")  # Adjust separator if needed

# 2. Cost function
def cost_function(params):
    Kp, Ki, Kd = params
    iTerm = 0
    last_error = 0
    last_time = df['timestamp'].iloc[0]
    total_error = 0

    for idx in range(1, len(df)):
        current_time = df['timestamp'].iloc[idx]
        dT = (current_time - last_time) / 1000.0  # assuming timestamp is in ms, convert to seconds
        if dT <= 0:
            dT = 0.001  # avoid division by zero

        target = df['target'].iloc[idx]
        current = df['current'].iloc[idx]
        error = target - current

        iTerm += error * dT
        dError = (error - last_error) / dT

        output = (Kp * error) + (Ki * iTerm) + (Kd * dError)

        # Instead of real plant simulation, compare with logged output
        logged_output = df['output'].iloc[idx]

        total_error += np.abs(output - logged_output)

        last_error = error
        last_time = current_time

    return total_error

# 3. Initial guess
initial_guess = [1406.024, 0.0016, 0.00]  # Initial guess for Kp, Ki, Kd

# 4. Optimize!
result = minimize(cost_function, initial_guess, method='Powell')  # Powell works well for small dimensions

optimal_Kp, optimal_Ki, optimal_Kd = result.x

print(f"Optimized PID:")
print(f"Kp: {optimal_Kp:.4f}")
print(f"Ki: {optimal_Ki:.4f}")
print(f"Kd: {optimal_Kd:.4f}")

# 5. Plot comparison (optional)
def simulate_with_pid(Kp, Ki, Kd):
    iTerm = 0
    last_error = 0
    last_time = df['timestamp'].iloc[0]
    simulated_outputs = []

    for idx in range(1, len(df)):
        current_time = df['timestamp'].iloc[idx]
        dT = (current_time - last_time) / 1000.0
        if dT <= 0:
            dT = 0.001

        target = df['target'].iloc[idx]
        current = df['current'].iloc[idx]
        error = target - current

        iTerm += error * dT
        dError = (error - last_error) / dT

        output = (Kp * error) + (Ki * iTerm) + (Kd * dError)
        simulated_outputs.append(output)

        last_error = error
        last_time = current_time

    return simulated_outputs

simulated = simulate_with_pid(optimal_Kp, optimal_Ki, optimal_Kd)

plt.figure(figsize=(12, 6))
plt.plot(df['timestamp'].iloc[1:], df['output'].iloc[1:], label='Logged Output', color='blue')
plt.plot(df['timestamp'].iloc[1:], simulated, label='Simulated with Optimized PID', color='red', linestyle='--')
plt.legend()
plt.title("PID Output Comparison")
plt.xlabel("Timestamp (ms)")
plt.ylabel("Output")
plt.grid()
plt.show()
