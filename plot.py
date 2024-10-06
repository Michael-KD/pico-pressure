import csv
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

# Function to convert milliseconds to hours
def ms_to_hours(ms):
    return ms / (1000 * 60 * 60)

# Ask for file size input
file_size_kb = float(input("Enter the file size in KB: "))

# Read the CSV file
times, pressures, temperatures = [], [], []
cumulative_time = 0
time_diffs = []

with open('D:\TEST2.CSV', 'r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)  # Skip the header row
    first_row = True
    for row in csv_reader:
        time, pressure, temp = map(float, row)
        if first_row:
            first_row = False
            cumulative_time = time
        else:
            time_diffs.append(time)
        cumulative_time += time
        times.append(ms_to_hours(cumulative_time))
        pressures.append(pressure / 100)  # Divide pressure by 100
        temperatures.append(temp / 100)  # Divide temperature by 100

# Calculate statistics (excluding the first time value)
avg_frequency_hz = 1000 / np.mean(time_diffs)
median_frequency_hz = 1000 / np.median(time_diffs)
pressure_std = np.std(pressures[1:])
total_time_hours = times[-1]
file_size_per_24h_mb = (file_size_kb / total_time_hours) * 24 / 1000  # Convert to MB
max_time_between_readings_ms = np.max(time_diffs)
min_time_between_readings_ms = np.min(time_diffs)

# Create three separate plots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))

# Plot Pressure
ax1.plot(times, pressures, color='tab:blue')
ax1.set_ylabel('Pressure (mbar)', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.set_title('Pressure over Time')

# Plot Temperature
ax2.plot(times, temperatures, color='tab:orange')
ax2.set_ylabel('Temperature (°C)', color='tab:orange')
ax2.tick_params(axis='y', labelcolor='tab:orange')
ax2.set_title('Temperature over Time')
ax2.set_xlabel('Time (hours)')

# Plot 300-sample window of Pressure
window_start = 2  # Start from the 3rd row
window_end = min(window_start + 300, len(pressures))
windowed_pressures = pressures[window_start:window_end]
windowed_times = np.cumsum(time_diffs[window_start-1:window_end-1]) / 1000  # Convert to seconds

ax3.plot(windowed_times, windowed_pressures, color='tab:green')
ax3.set_xlabel('Time (seconds)')
ax3.set_ylabel('Pressure (mbar)', color='tab:green')
ax3.tick_params(axis='y', labelcolor='tab:green')
ax3.set_title('300-Sample Window of Pressure')

# Add statistics textbox
stats_text = "Sensor Info\n-----------\n"
stats_text += f"Sensor name: MS5802-02BA\n"

stats_text += "\nTest Info\n-----------\n"
stats_text += f"Over Sampling Rate: 512\n"
stats_text += f"SD Care Logging: Stores 50 readings, then logs all at once\n"
stats_text += f"File Size: {file_size_kb} KB\n"
stats_text += f"Average voltage: 5 V\n"
stats_text += f"Average current: roughly 36 mA\n"

stats_text += "\nData Stats\n-----------\n"
stats_text += f"Avg Frequency: {avg_frequency_hz:.2f} Hz\n"
stats_text += f"Median Frequency: {median_frequency_hz:.2f} Hz\n"
stats_text += f"Pressure Std Dev: {pressure_std:.4f} mbar\n"
stats_text += f"File Size per 24h: {file_size_per_24h_mb:.2f} MB\n"
stats_text += f"Max Time Between Readings: {max_time_between_readings_ms:.2f} ms\n"
stats_text += f"Min Time Between Readings: {min_time_between_readings_ms:.2f} ms\n"
stats_text += f"Total Time: {total_time_hours:.2f} hours"

# Create a new axes for the text box
ax_text = fig.add_axes([0.72, 0.02, 0.26, 0.25])  # [left, bottom, width, height]
ax_text.axis('off')
ax_text.text(0, 0.5, stats_text, verticalalignment='center', fontsize=10)

# Add a border around the text
rect = Rectangle((0, 0), 1, 1, fill=False, edgecolor='black', lw=1)
ax_text.add_patch(rect)

# Adjust layout
plt.tight_layout()
plt.subplots_adjust(right=0.7)  # Make room for the text box

# Save the plot as a PNG file
plt.savefig('pressure_temperature_plot.png', dpi=300, bbox_inches='tight')
print("Plot saved as 'pressure_temperature_plot.png'")