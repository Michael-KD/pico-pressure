import csv
import matplotlib.pyplot as plt
import os
import glob

def ms_to_hours(ms):
    return ms / (1000 * 60 * 60)

base_filename = input("Enter the base filename for the test (e.g., 'DATA' for DATA_A.csv, DATA_B.csv): ")
file_pattern = f"{base_filename}_*.csv"

all_times, all_pressures, all_temperatures = [], [], []

for filename in sorted(glob.glob(file_pattern)):

    times, pressures, temperatures = [], [], []
    cumulative_time = 0

    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader)  # Skip header
        first_row = True
        for row in csv_reader:
            time, pressure, temp = map(float, row)
            if first_row:
                first_row = False
                cumulative_time = time
            cumulative_time += time
            times.append(ms_to_hours(cumulative_time))
            pressures.append(pressure / 100)
            temperatures.append(temp / 100)

    if all_times:
        time_offset = all_times[-1][-1]
        times = [t + time_offset for t in times]

    all_times.append(times)
    all_pressures.append(pressures)
    all_temperatures.append(temperatures)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

for i, (times, pressures, temperatures) in enumerate(zip(all_times, all_pressures, all_temperatures)):
    label = f"{base_filename}_{chr(97 + i)}"
    ax1.plot(times, pressures, label=label)
    ax2.plot(times, temperatures, label=label)

ax1.set_ylabel('Pressure (mbar)')
ax1.set_title('Pressure over Time')
ax1.legend()

ax2.set_ylabel('Temperature (°C)')
ax2.set_title('Temperature over Time')
ax2.set_xlabel('Time (hours)')
ax2.legend()

plt.tight_layout()
output_filename = f"{base_filename}_combined_plot.png"
plt.savefig(output_filename, dpi=300, bbox_inches='tight')
print(f"Plot saved as '{output_filename}'")
