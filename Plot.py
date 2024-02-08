import matplotlib.pyplot as plt
import pandas as pd

# Load data
data = pd.read_csv('simulation_data.csv')

# Plotting
plt.figure(figsize=(10, 6))

# Plot each q and qdot
plt.plot(data['Time'], data['q1'], label='q1')
plt.plot(data['Time'], data['q2'], label='q2')
plt.plot(data['Time'], data['q3'], label='q3')
plt.plot(data['Time'], data['qdot1'], label='qdot1', linestyle='--')
plt.plot(data['Time'], data['qdot2'], label='qdot2', linestyle='--')
plt.plot(data['Time'], data['qdot3'], label='qdot3', linestyle='--')

# Adding labels and legend
plt.xlabel('Time')
plt.ylabel('Values')
plt.title('q and qdot vs. Time')
plt.legend()

# Show plot
plt.show()